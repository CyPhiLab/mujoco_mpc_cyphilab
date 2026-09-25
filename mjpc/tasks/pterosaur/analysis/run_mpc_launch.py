#!/usr/bin/env python3
"""Run the MJPC Launch Track mode on design variants and evaluate the launch.

Wraps the headless launch_driver (launch_driver.cc, linked against libmjpc):
for each variant (actuator torque scale, parallel spring energy) it runs
closed-loop MPC from the reference's deepest crouch, then reports takeoff
velocity, flight, and whether the body stayed upright, and optionally renders
a video.

Usage:
  run_mpc_launch.py --driver path/to/launch_driver --torque 2 --spring 500 \
      [--speed 5 --angle 30] [--video out.mp4] [--out traj.csv]
"""
import argparse
import os
import subprocess

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TASK_DIR = os.path.dirname(HERE)

SHOULDER2 = [1, 4]
HIP2 = [7, 10]
JOINT_MARGIN = 0.3  # joint limits: reference range +- margin (model has none)


def reference():
  return np.load(os.path.join(TASK_DIR, 'reference', 'launch.npz'))


def push_start(ref):
  """Deepest crouch (lowest base) before takeoff, as in pterosaur.cc."""
  airborne = np.flatnonzero(~ref['hands_contact'] & ~ref['feet_contact'])
  takeoff = airborne[0]
  return ref['time'][np.argmin(ref['qpos'][:takeoff, 2])]


def spring_design(ref, energy):
  """Parallel springs on shoulder2/hip2 storing energy/4 each at the crouch.

  Rest angle is the limb's liftoff angle in the reference, so the springs
  push through the whole extension.
  """
  q = ref['qpos'][:, 7:]
  i_crouch = int(round(push_start(ref) / (ref['time'][1] - ref['time'][0])))
  hands_off = int(np.argmin(ref['hands_contact']))
  feet_off = int(np.argmin(ref['feet_contact']))
  stiffness = np.zeros(12)
  springref = q[0].copy()
  for j in SHOULDER2 + HIP2:
    rest = q[hands_off if j in SHOULDER2 else feet_off, j]
    springref[j] = rest
    if energy > 0:
      stiffness[j] = 2 * (energy / 4) / (q[i_crouch, j] - rest) ** 2
  return stiffness, springref


def joint_limits(ref):
  q = ref['qpos'][:, 7:]
  lo, hi = q.min(axis=0) - JOINT_MARGIN, q.max(axis=0) + JOINT_MARGIN
  return np.stack([lo, hi], axis=1).ravel()


def run(driver, torque, spring, speed, angle, out, duration=2.0, sim_dt=0.002,
        iters=1, threads=4, horizon=None, weights=None, plan_dt=None,
        foot_solimp=None, foot_solref=None, push_time=None):
  ref = reference()
  stiffness, springref = spring_design(ref, spring)
  fmt = lambda a: ','.join(f'{x:.6g}' for x in a)
  cmd = [driver, '--out', out, '--gain_scale', str(torque),
         '--stiffness', fmt(stiffness), '--springref', fmt(springref),
         '--limits', fmt(joint_limits(ref)), '--speed', str(speed),
         '--angle', str(angle), '--ref_start', str(push_start(ref)),
         '--duration', str(duration), '--sim_dt', str(sim_dt),
         '--iters_per_step', str(iters), '--threads', str(threads)]
  if horizon:
    cmd += ['--horizon', str(horizon)]
  if plan_dt:
    cmd += ['--plan_dt', str(plan_dt)]
  if push_time:
    cmd += ['--push_time', str(push_time)]
  if foot_solimp:
    cmd += ['--foot_solimp', foot_solimp]
  if foot_solref:
    cmd += ['--foot_solref', foot_solref]
  if weights:
    cmd += ['--weights', weights]
  proc = subprocess.run(cmd, capture_output=True, text=True)
  if proc.returncode != 0:
    raise RuntimeError(proc.stderr)
  return proc.stderr.strip()


def load(csv):
  data = np.genfromtxt(csv, delimiter=',', names=True)
  return data


def evaluate(traj, min_flight=0.15):
  """Takeoff = start of the first airborne segment lasting >= min_flight."""
  t = traj['time']
  contact = traj['floor_contact'] > 0
  dt = t[1] - t[0]
  need = int(round(min_flight / dt))
  takeoff = None
  run_len = 0
  for k in range(len(t)):
    run_len = 0 if contact[k] else run_len + 1
    if run_len >= need:
      takeoff = k - need + 1
      break
  quat = np.stack([traj[f'qpos{i}'] for i in range(3, 7)], axis=1)
  up_z = 1 - 2 * (quat[:, 1] ** 2 + quat[:, 2] ** 2)
  result = {
      'took_off': takeoff is not None,
      'min_up_axis_z': float(up_z.min()),
      'max_com_rise_m': float(traj['com_z'].max() - traj['com_z'][0]),
  }
  if takeoff is not None:
    v = np.array([traj['comvel_x'][takeoff], traj['comvel_y'][takeoff],
                  traj['comvel_z'][takeoff]])
    # flight: until next contact or end of run
    land = np.flatnonzero(contact[takeoff:])
    flight = (land[0] if len(land) else len(t) - takeoff) * dt
    result.update({
        'takeoff_time': float(t[takeoff]),
        'takeoff_speed': float(np.linalg.norm(v)),
        'takeoff_angle_deg': float(np.degrees(np.arctan2(v[2], np.hypot(v[0], v[1])))),
        'takeoff_vel': v.tolist(),
        'flight_time': float(flight),
        'landed_in_run': bool(len(land)),
    })
  return result


def diagnose(traj, end_time=None):
  """Push quality until end_time (default: takeoff): non-foot ground
  contact, left/right mismatch, and actuator use."""
  import mujoco
  m = mujoco.MjModel.from_xml_path(os.path.join(TASK_DIR, 'task.xml'))
  d = mujoco.MjData(m)
  floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
  feet = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
          for n in ['FL', 'FR', 'HL', 'HR']}
  t = traj['time']
  if end_time is None:
    end_time = evaluate(traj).get('takeoff_time', t[-1])
  push = t <= end_time
  qpos = np.stack([traj[f'qpos{i}'] for i in range(m.nq)], axis=1)
  ctrl = np.stack([traj[f'ctrl{i}'] for i in range(m.nu)], axis=1)
  dt = t[1] - t[0]
  touching = {}
  for k in np.flatnonzero(push)[::5]:
    d.qpos[:] = qpos[k]
    mujoco.mj_forward(m, d)
    bodies = set()
    for c in d.contact[:d.ncon]:
      if floor not in (c.geom1, c.geom2) or c.dist > 0:
        continue
      g = c.geom2 if c.geom1 == floor else c.geom1
      if g not in feet:
        bodies.add(m.body(m.geom_bodyid[g]).name)
    for name in bodies:
      touching[name] = touching.get(name, 0) + 5 * dt
  sign = np.array([-1, 1, 1, 1, 1, 1])
  left = qpos[push][:, 7 + np.array([0, 1, 2, 6, 7, 8])]
  right = qpos[push][:, 7 + np.array([3, 4, 5, 9, 10, 11])] * sign
  mismatch = np.abs(left - right)
  u = np.abs(ctrl[push])
  return {
      'push_end': float(end_time),
      'nonfoot_contact_s': {k: round(v, 2) for k, v in sorted(touching.items())},
      'arm_LR_mismatch_rad': [round(float(mismatch[:, :3].mean()), 3),
                              round(float(mismatch[:, :3].max()), 3)],
      'leg_LR_mismatch_rad': [round(float(mismatch[:, 3:].mean()), 3),
                              round(float(mismatch[:, 3:].max()), 3)],
      'mean_abs_ctrl': round(float(u.mean()), 2),
      'saturated_frac': round(float((u > 0.95).mean()), 2),
  }


def render(traj, title, path):
  import mujoco
  from render_launch import Recorder, FPS
  m = mujoco.MjModel.from_xml_path(os.path.join(TASK_DIR, 'task.xml'))
  d = mujoco.MjData(m)
  rec = Recorder(m)
  t = traj['time']
  frame_times = np.arange(t[0], t[-1], 1 / FPS)
  for ft in frame_times:
    k = int(np.searchsorted(t, ft))
    d.qpos[:] = [traj[f'qpos{i}'][k] for i in range(m.nq)]
    mujoco.mj_forward(m, d)
    v = np.linalg.norm([traj['comvel_x'][k], traj['comvel_y'][k], traj['comvel_z'][k]])
    rec.capture(d, f'{title}  t={ft:.2f}s  |v_com|={v:.2f} m/s')
  rec.save(path)


# MPC settings that work for launch (see README notes in the commit):
# 5 ms planning timestep (10 ms rollouts diverge), 0.5 s horizon, several
# planner iterations per control step, and the takeoff objective weighted
# above joint tracking so the MPC can reshape the push.
TUNED = dict(iters=3, plan_dt=0.005, horizon=0.5,
             weights='RefTakeoff=10,RefJointPos=0.5,RefJointVel=0.01')


def grid(driver, torques, springs, speed, angle, outdir, videos=True):
  import json
  os.makedirs(outdir, exist_ok=True)
  results = []
  for ts in torques:
    for es in springs:
      name = f't{ts:g}_s{es:g}'
      csv = os.path.join(outdir, name + '.csv')
      log = run(driver, ts, es, speed, angle, csv, **TUNED)
      traj = load(csv)
      row = {'torque_scale': ts, 'spring_energy_J': es,
             'diverged': 'divergence' in log, **evaluate(traj)}
      results.append(row)
      print(name, {k: round(v, 3) if isinstance(v, float) else v
                   for k, v in row.items() if k != 'takeoff_vel'}, flush=True)
      if videos:
        render(traj, f'MPC torque x{ts:g}, spring {es:g} J',
               os.path.join(outdir, name + '.mp4'))
      with open(os.path.join(outdir, 'results.json'), 'w') as f:
        json.dump(results, f, indent=1)
  return results


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--driver', required=True)
  parser.add_argument('--torque', type=float, default=1.0)
  parser.add_argument('--spring', type=float, default=0.0)
  parser.add_argument('--speed', type=float, default=5.0)
  parser.add_argument('--angle', type=float, default=30.0)
  parser.add_argument('--duration', type=float, default=2.0)
  parser.add_argument('--iters', type=int, default=1)
  parser.add_argument('--horizon', type=float, default=None)
  parser.add_argument('--weights', default=None)
  parser.add_argument('--plan_dt', type=float, default=None)
  parser.add_argument('--push_time', type=float, default=None,
                      help='compress the push to this many seconds')
  parser.add_argument('--foot_solimp', default=None, help='e.g. 0.9,0.95,0.001')
  parser.add_argument('--foot_solref', default=None, help='e.g. 0.005,1')
  parser.add_argument('--out', default='mpc_launch.csv')
  parser.add_argument('--video', default=None)
  parser.add_argument('--grid', default=None,
                      help='output dir: run torque x spring design grid')
  args = parser.parse_args()

  if args.grid:
    grid(args.driver, [1.0, 1.5, 2.0, 3.0], [0.0, 250.0, 500.0, 1000.0],
         args.speed, args.angle, args.grid)
    return

  log = run(args.driver, args.torque, args.spring, args.speed, args.angle,
            args.out, duration=args.duration, iters=args.iters,
            horizon=args.horizon, weights=args.weights, plan_dt=args.plan_dt,
            foot_solimp=args.foot_solimp, foot_solref=args.foot_solref,
            push_time=args.push_time)
  if log:
    print(log)
  traj = load(args.out)
  result = evaluate(traj)
  print(result)
  print(diagnose(traj))
  if args.video:
    title = (f'MPC  torque x{args.torque:g}, spring {args.spring:g} J, '
             f'target {args.speed:g} m/s @ {args.angle:g} deg')
    render(traj, title, args.video)


if __name__ == '__main__':
  main()
