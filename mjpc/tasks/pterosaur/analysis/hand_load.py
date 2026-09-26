#!/usr/bin/env python3
"""Check whether a launch plants its hands and feet or strikes with them.

Replays optimized controls (from launch_ilqr.py / launch_trajopt.py) on the
same physics and reports, per time window of the push, the left hand's
height range, mean normal force and the fraction of steps with no load,
plus the same load numbers for the left foot. A planted limb carries load
continuously; a hammering one alternates between zero and large forces.

Usage:
  hand_load.py controls.npy --push_time 0.45 [--hand_radius 0.04 ...]
"""
import argparse

import mujoco
import numpy as np

import launch_ilqr as L


def hand_load(solver, U, windows=None):
  m, d = solver.m, solver.d
  d.qpos[:], d.qvel[:] = solver.q0, solver.v0
  mujoco.mj_forward(m, d)
  floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
  arm = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'radius_and_ulna')
  leg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'tibia')
  height, hand_f, foot_f = [], [], []
  force = np.zeros(6)
  for t in range(solver.n_push):
    d.ctrl[:] = L.E @ np.clip(U[t], -1, 1)
    mujoco.mj_step(m, d)
    fh = ff = 0.0
    for i, c in enumerate(d.contact[:d.ncon]):
      if floor not in (c.geom1, c.geom2):
        continue
      body = m.geom_bodyid[c.geom2 if c.geom1 == floor else c.geom1]
      mujoco.mj_contactForce(m, d, i, force)
      if body == arm:
        fh += force[0]
      elif body == leg:
        ff += force[0]
    height.append(solver.limb(solver.hands[0])[0] * 1000)
    hand_f.append(fh)
    foot_f.append(ff)
  height, hand_f, foot_f = map(np.array, (height, hand_f, foot_f))
  t = np.arange(solver.n_push) * solver.dt
  push = solver.n_push * solver.dt
  if windows is None:
    edges = np.linspace(0, push, 6)
    windows = list(zip(edges[:-1], edges[1:]))
  rows = []
  for a, b in windows:
    w = (t >= a) & (t < b)
    rows.append({
        'window': (round(a, 3), round(b, 3)),
        'hand_height_mm': (round(height[w].min(), 1), round(height[w].max(), 1)),
        'hand_force_N': round(hand_f[w].mean()),
        'hand_unloaded_frac': round(float(np.mean(hand_f[w] <= 0)), 2),
        'foot_force_N': round(foot_f[w].mean()),
        'foot_unloaded_frac': round(float(np.mean(foot_f[w] <= 0)), 2),
    })
  return rows


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('controls')
  parser.add_argument('--push_time', type=float, default=0.45)
  parser.add_argument('--torque', type=float, default=2.0)
  parser.add_argument('--hand_radius', type=float, default=0)
  parser.add_argument('--hand_timeconst', type=float, default=0.02)
  parser.add_argument('--hand_dampratio', type=float, default=1.0)
  args = parser.parse_args()
  hand = (dict(radius=args.hand_radius, timeconst=args.hand_timeconst,
               dampratio=args.hand_dampratio) if args.hand_radius > 0 else None)
  solver = L.LaunchILQR(args.push_time, args.torque, hand=hand)
  U = np.load(args.controls)[:solver.N]
  for row in hand_load(solver, U):
    print(row)


if __name__ == '__main__':
  main()
