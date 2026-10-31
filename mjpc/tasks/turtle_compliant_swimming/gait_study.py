#!/usr/bin/env python3
"""Open-loop study of how stroke rate and amplitude set the turtle's swimming speed.

  python gait_study.py --turtle-sim ../../../../turtle_simulator grid
  python gait_study.py --turtle-sim ... search --budget 150 --vmax 4.3

The gait family is turtle_simulator's reference gait (Cornelia trajectories, as in
make_reference_gait.py) with four knobs:
  cadence  stroke-rate multiplier (1 = turtle_sim.py's 1.911 s stroke)
  yaw, roll, twist  amplitude of controller joints 1-3 relative to the biological data
           (turtle_sim.py --servo uses 1/2, 5/6, 1/3; see SERVO_STROKE_SCALE)
Rear hips are held at zero.

Each gait runs on turtle_compliant_mjpc.xml twice:
  locked   shell translation only (rotation locked): thrust-limited speed, with no
           stability problem. This is the upper bound a stabilizing controller works towards.
  free     the real free-floating robot, open loop (pitches up, see RESULTS.md)

Reported per run: forward speed along world -y over whole strokes after the first two,
peak joint speed and peak actuator torque (to compare with motor limits), positive
shaft power (with --motor: actuator torque minus back-EMF torque, times joint speed), and
energy per metre.
"""
import argparse
import itertools
import json
import os
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ACTUATORS = ["shoulderL1", "shoulderL2", "shoulderL3", "shoulderR1", "shoulderR2", "shoulderR3",
             "hip_L1", "hip_L2", "hip_R1", "hip_R2"]
N = 128
SW = 1.5
BASE_PERIOD = 4.3 / SW**2


def biological_table(turtle_sim_path):
  """Unscaled biological joint targets, controller convention, N samples over one stroke."""
  sys.path.insert(0, str(Path(turtle_sim_path).resolve()))
  from turtle_ctrl.turtle_ctrl_factory import cornelia_joint_space_trajectory_tracking_control_factory
  import turtle_sim
  fn = cornelia_joint_space_trajectory_tracking_control_factory(kp=[2.0] * 6, sw=SW)
  t_new0 = (4.3 / SW) * 0.32
  q = np.zeros((N, 6))
  for k in range(N):
    _, aux = fn(t=t_new0 + SW * BASE_PERIOD * k / N, q=np.zeros(6))
    q[k] = aux["q_des"]
  return q, turtle_sim.FRONT_JOINT_MAP, np.array(turtle_sim.SERVO_STROKE_SCALE)


def gait_table(bio, joint_map, yaw, roll, twist, twist_shift=0.0):
  """Servo targets (N x 10, actuator order). Controller joint order is [roll, yaw, twist] per
  arm in FRONT_JOINT_MAP's convention: index 0 -> model joint 2, 1 -> joint 1, 2 -> joint 3."""
  scale = np.array([roll, yaw, twist, roll, yaw, twist])
  q = bio * scale
  if twist_shift:  # delay the twist (feathering) by this fraction of a stroke
    phase = np.arange(N) / N
    for j in (2, 5):
      q[:, j] = np.interp((phase - twist_shift) % 1.0, phase, bio[:, j] * twist, period=1.0)
  table = np.zeros((N, 10))
  table[:, :6] = q @ joint_map  # (M^T q)^T = q^T M
  return table


# Dynamixel XW540-T260 (ROBOTIS e-manual): no-load speed [rad/s], stall torque [N m]
MOTORS = {"12V": (40 * 2 * np.pi / 60, 9.5), "14.8V": (48 * 2 * np.pi / 60, 11.2)}


def load_model(locked, kp=None, force=None, motor=None):
  """motor: None (turtle_sim --servo placeholder: +-10 N m, no speed limit) or a MOTORS key.
  A DC motor's torque falls linearly with speed, tau <= tau_stall * (1 - w / w_noload). That
  is the servo's force limit (tau_stall) plus joint damping tau_stall / w_noload (back-EMF)."""
  import mujoco
  xml = (HERE / "turtle_compliant_mjpc.xml").read_text()
  if locked:
    xml, n = re.subn(r'<joint name="world_frame_to_base" type="free"[^>]*/>',
                     '<joint name="slide_x" type="slide" axis="1 0 0"/>'
                     '<joint name="slide_y" type="slide" axis="0 1 0"/>'
                     '<joint name="slide_z" type="slide" axis="0 0 1"/>', xml)
    assert n == 1
  # write next to the original so meshdir="assets" resolves
  tmp = HERE / f"_gait_study_{'locked' if locked else 'free'}_{os.getpid()}.xml"
  tmp.write_text(xml)
  try:
    model = mujoco.MjModel.from_xml_path(str(tmp))
  finally:
    tmp.unlink()
  if kp is not None:
    for i in range(model.nu):
      model.actuator_gainprm[i, 0] = kp
      model.actuator_biasprm[i, 1] = -kp
  if force is not None:
    model.actuator_forcerange[:] = [-force, force]
  if motor is not None:
    w0, tau_s = MOTORS[motor]
    model.actuator_forcerange[:] = [-tau_s, tau_s]
    for name in ACTUATORS:
      model.dof_damping[model.joint(name).dofadr[0]] += tau_s / w0
  return model


def simulate(model, table, cadence, strokes=6, skip=2):
  import mujoco
  data = mujoco.MjData(model)
  period = BASE_PERIOD / cadence
  act = [model.actuator(n).id for n in ACTUATORS]
  # damping on the actuated joints (back-EMF in the motor model): shaft torque = force - b*qd
  damp = np.array([model.dof_damping[model.joint(n).dofadr[0]] for n in ACTUATORS])
  root = model.body("fr13_s105k").id
  # start on the gait
  for j, name in enumerate(ACTUATORS):
    data.qpos[model.joint(name).qposadr[0]] = table[0, j]
  mujoco.mj_forward(model, data)

  def ref(t):
    x = (t / period % 1.0) * N
    k = int(x)
    f = x - k
    return (1 - f) * table[k % N] + f * table[(k + 1) % N]

  t_end = strokes * period
  t0 = skip * period
  com0 = None
  work = 0.0
  vmax = np.zeros(model.nu)
  fmax = np.zeros(model.nu)
  dt = model.opt.timestep
  while data.time < t_end - 1e-9:
    data.ctrl[act] = ref(data.time)
    mujoco.mj_step(model, data)
    if not np.isfinite(data.qpos).all():
      return None
    if data.time >= t0 - 1e-9:
      if com0 is None:
        com0 = data.subtree_com[root].copy()
        tc0 = data.time
      w = data.actuator_velocity[act]
      p = (data.actuator_force[act] - damp * w) * w  # shaft power
      work += np.clip(p, 0, None).sum() * dt
      vmax = np.maximum(vmax, np.abs(data.actuator_velocity))
      fmax = np.maximum(fmax, np.abs(data.actuator_force))
  disp = data.subtree_com[root] - com0
  T = data.time - tc0
  speed = -disp[1] / T
  return dict(speed=speed, net=float(np.linalg.norm(disp) / T), climb=disp[2] / T,
              power=work / T, jpm=work / max(-disp[1], 1e-6) if speed > 0 else np.nan,
              vmax=float(vmax[:6].max()), fmax=float(fmax[:6].max()),
              vmax_joint=ACTUATORS[int(vmax[:6].argmax())])


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--turtle-sim", required=True)
  ap.add_argument("--kp", type=float, default=None, help="override servo kp (N m/rad)")
  ap.add_argument("--force", type=float, default=None, help="override servo force limit (N m)")
  ap.add_argument("--motor", choices=sorted(MOTORS), default=None,
                  help="XW540-T260 torque-speed limit at this supply voltage (default: placeholder servo)")
  sub = ap.add_subparsers(dest="cmd", required=True)
  g = sub.add_parser("grid")
  g.add_argument("--cadence", type=float, nargs="+", default=[1.0, 1.25, 1.5, 2.0])
  g.add_argument("--yaw", type=float, nargs="+", default=[0.5])
  g.add_argument("--roll", type=float, nargs="+", default=[5 / 6])
  g.add_argument("--twist", type=float, nargs="+", default=[1 / 3])
  g.add_argument("--free", action="store_true", help="also run the free body")
  s = sub.add_parser("search", help="CMA-ES on the locked body: max speed (soft joint speed / torque limits)")
  s.add_argument("--budget", type=int, default=150)
  s.add_argument("--vmax", type=float, default=4.3, help="joint speed limit [rad/s]")
  s.add_argument("--fmax", type=float, default=10.0, help="torque limit [N m] (servo force limit)")
  s.add_argument("--max-cadence", type=float, default=3.0)
  s.add_argument("--out", default="gait_search.json")
  e = sub.add_parser("export", help="write one gait as a turtle_trajopt --gait_csv table")
  e.add_argument("--cadence", type=float, default=1.0, help="printed only; pass it to the runner")
  e.add_argument("--yaw", type=float, required=True)
  e.add_argument("--roll", type=float, required=True)
  e.add_argument("--twist", type=float, required=True)
  e.add_argument("--twist-shift", type=float, default=0.0)
  e.add_argument("--out", required=True)
  args = ap.parse_args()

  if args.cmd == "export":
    bio, joint_map, _ = biological_table(args.turtle_sim)
    tab = gait_table(bio, joint_map, args.yaw, args.roll, args.twist, args.twist_shift)
    np.savetxt(args.out, tab, delimiter=",", header=",".join(ACTUATORS), comments="", fmt="%.6f")
    print(f"wrote {args.out}; run with --gait_csv={args.out} --param=Cadence={args.cadence:g}")
    return

  bio, joint_map, servo_scale = biological_table(args.turtle_sim)
  locked = load_model(True, args.kp, args.force, args.motor)
  free = load_model(False, args.kp, args.force, args.motor)

  if args.cmd == "grid":
    print("cadence  yaw   roll  twist | locked: speed  peak|qd| (joint)   peak|tau|  power  J/m"
          + ("  | free: along -y  climb" if args.free else ""))
    for c, y, r, tw in itertools.product(args.cadence, args.yaw, args.roll, args.twist):
      tab = gait_table(bio, joint_map, y, r, tw)
      a = simulate(locked, tab, c)
      line = (f"{c:6.2f} {y:5.2f} {r:5.2f} {tw:5.2f} | {a['speed']:12.4f} {a['vmax']:6.2f} ({a['vmax_joint']}) "
              f"{a['fmax']:8.2f} {a['power']:6.2f} {a['jpm']:5.1f}")
      if args.free:
        b = simulate(free, tab, c)
        line += f"  | {b['speed']:9.4f} {b['climb']:+7.4f}"
      print(line, flush=True)
    return

  # ---- CMA-ES over (cadence, yaw, roll, twist) on the locked body ----
  import cma  # pip install cma
  # knobs: cadence, yaw, roll, twist amplitude, twist delay (fraction of a stroke)
  lo = np.array([0.5, 0.0, 0.3, 0.0, -0.25])
  hi = np.array([args.max_cadence, 1.2, 1.2, 1.5, 0.25])
  x0 = np.array([1.0, 0.5, 5 / 6, 1 / 3, 0.0])
  to_x = lambda u: lo + (hi - lo) * np.clip(u, 0, 1)
  history = []

  def objective(u):
    x = to_x(u)
    res = simulate(locked, gait_table(bio, joint_map, *x[1:]), x[0], strokes=5)
    if res is None:
      return 1.0
    # soft limits: penalize joint speed / torque beyond the motor limits
    pen = 2.0 * max(res["vmax"] - args.vmax, 0) / args.vmax + 2.0 * max(res["fmax"] - args.fmax, 0) / args.fmax
    history.append(dict(x=x.tolist(), **res))
    print(f"cad {x[0]:.2f} yaw {x[1]:.2f} roll {x[2]:.2f} twist {x[3]:.2f} shift {x[4]:+.2f} -> {res['speed']:.4f} m/s, "
          f"|qd| {res['vmax']:.2f}, |tau| {res['fmax']:.2f}, {res['power']:.1f} W", flush=True)
    return -res["speed"] + pen

  es = cma.CMAEvolutionStrategy((x0 - lo) / (hi - lo), 0.15,
                                {"bounds": [0, 1], "maxfevals": args.budget, "popsize": 10, "verbose": -9})
  while not es.stop():
    us = es.ask()
    es.tell(us, [objective(u) for u in us])
  ok = [h for h in history if h["vmax"] <= args.vmax and h["fmax"] <= args.fmax]
  best = max(ok, key=lambda h: h["speed"]) if ok else None
  Path(args.out).write_text(json.dumps(dict(best=best, history=history), indent=1))
  print("best within limits:", json.dumps(best))


if __name__ == "__main__":
  main()
