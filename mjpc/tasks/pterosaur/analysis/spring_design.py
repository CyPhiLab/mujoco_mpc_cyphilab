#!/usr/bin/env python3
"""Size latched springs from a tracked launch (launch_track.py output).

Replays one step of a run and takes the positive work each sagittal joint
(shoulder2, elbow, hip2, knee) receives from all actuators until its limb
leaves the ground: motors, with their torque-speed damping integrated
implicitly as MuJoCo does (force x start-of-step speed is badly off when the
damping time constant is near the time step), and any springs. Each spring
joint gets that work's share of the budget, is released when the joint
starts its 5-95% positive-work phase and travels from the joint angle there
to the angle at its end. Writes a JSON for launch_track.py --spring_design.

Usage:
  spring_design.py RUN_DIR STEP_NAME [--out FILE]
"""
import argparse
import json
import os

import mujoco
import numpy as np

import launch_track as LT
import launch_trajopt as T

# spring joints: name, (left, right) qpos[7:] index, limb contact sensor
SPRING_JOINTS = [('shoulder2', (1, 4), ('to_hand_l', 'to_hand_r')),
                 ('elbow', (2, 5), ('to_hand_l', 'to_hand_r')),
                 ('hip2', (7, 10), ('to_foot_l', 'to_foot_r')),
                 ('knee', (8, 11), ('to_foot_l', 'to_foot_r'))]
MIN_SHARE = 0.05


def joint_power(solver, U):
  """Per-step limb joint power (n, 12) from all actuators, joint angles
  (n + 1, 12)."""
  m, d = solver.m, solver.d
  act_map = T.actuator_joint_map(m)
  slope = m.actuator_biasprm[:12, 2]
  d.qpos[:], d.qvel[:] = solver.q0, solver.v0
  power, q = [], [solver.q0[7:].copy()]
  for t in range(len(U)):
    solver.set_ctrl(np.clip(U[t], -1, 1), t)
    mujoco.mj_forward(m, d)
    tau, a0, v0 = d.qfrc_actuator[6:].copy(), d.actuator_velocity[:12].copy(), d.qvel[6:].copy()
    free = np.abs(d.actuator_force[:12]) < m.actuator_forcerange[:12, 1] - 1e-9
    mujoco.mj_step(m, d)
    mujoco.mj_fwdPosition(m, d)
    mujoco.mj_fwdVelocity(m, d)
    tau += act_map.T @ np.where(free, slope * (d.actuator_velocity[:12] - a0), 0)
    power.append(tau * 0.5 * (v0 + d.qvel[6:]))
    q.append(d.qpos[7:].copy())
  return np.array(power), np.array(q)


def main():
  p = argparse.ArgumentParser()
  p.add_argument('run')
  p.add_argument('step')
  p.add_argument('--out', default=None)
  a = p.parse_args()
  args = LT.parser().parse_args([])
  vars(args).update(json.load(open(os.path.join(a.run, 'args.json'))))
  design = LT.make_design(args, verbose=False)
  row = next(r for r in json.load(open(os.path.join(a.run, 'summary.json')))
             if r['name'] == a.step)
  solver = LT.TrackILQR(row['push_time'], args.torque, row['target_speed'],
                        track_scale=row['track_scale'],
                        min_hand_force=args.min_hand_force,
                        reference=args.reference, angle=args.angle,
                        window=args.window or None, free_hands=args.free_hands,
                        track_index=args.track_index, hand_load=args.hand_load,
                        hand_load_weight=args.hand_load_weight,
                        **dict(design, contact_smoothing=row.get('contact_smoothing', 0.0)))
  U = np.load(os.path.join(a.run, a.step + '_controls.npy'))[:solver.N]
  power, q = joint_power(solver, U)
  dt = solver.dt
  takeoff = int(round(row['takeoff_time'] / dt))
  out = []
  for name, sides, sensors in SPRING_JOINTS:
    # until the limb's last ground contact before takeoff
    last = max(max((i for i, c in enumerate(row['contacts'][s][:takeoff]) if c == '#'),
                   default=0) for s in sensors)
    pos = np.maximum(power[:last + 1, list(sides)].mean(axis=1), 0)
    work = pos.sum() * dt
    cum = np.cumsum(pos) / max(pos.sum(), 1e-9)
    i0, i1 = int(np.searchsorted(cum, 0.05)), int(np.searchsorted(cum, 0.95))
    out.append({'name': name, 'sides': list(sides), 'work_J': float(work),
                'release': round(i0 * dt, 3), 'end': round(i1 * dt, 3),
                'q0': float(q[i0, sides[0]]), 'q1': float(q[i1, sides[0]]),
                'limb_off': round(last * dt, 3)})
  total = sum(r['work_J'] for r in out)
  kept = [r for r in out if r['work_J'] / total >= MIN_SHARE]
  total_kept = sum(r['work_J'] for r in kept)
  for r in out:
    r['share'] = r['work_J'] / total_kept if r in kept else 0.0
    print(f"{r['name']:10s} work {r['work_J']:5.0f} J/side  share {r['share']:.2f}  "
          f"release {r['release']:.3f}-{r['end']:.3f} s (limb off {r['limb_off']:.3f})  "
          f"q {r['q0']:+.3f} -> {r['q1']:+.3f}" + ('' if r in kept else '  [dropped]'))
  path = a.out or os.path.join(a.run, a.step + '_springs.json')
  json.dump(kept, open(path, 'w'), indent=1)
  print('wrote', path)


if __name__ == '__main__':
  main()
