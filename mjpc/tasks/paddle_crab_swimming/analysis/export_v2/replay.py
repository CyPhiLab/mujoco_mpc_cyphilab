#!/usr/bin/env python3
"""Replays an exported gait open loop from rest: python replay.py [NAME] [CYCLES]

Needs only the mujoco and numpy packages. The servos in model.xml carry the
setpoint filter (tau = 0.08 s) as MuJoCo "filter" dynamics, so ctrl is the
raw periodic command from gaits.npz.
"""
import os
import sys

import mujoco
import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
m = mujoco.MjModel.from_xml_path(os.path.join(here, 'model.xml'))
d = mujoco.MjData(m)
g = np.load(os.path.join(here, 'gaits.npz'))
names = list(g['names'])
name = sys.argv[1] if len(sys.argv) > 1 else names[0]
cycles = float(sys.argv[2]) if len(sys.argv) > 2 else 6
i = names.index(name)
period, k = float(g['period'][i]), int(g['k'][i])
c, a, b = g['c'][i], g['a'][i][:, :k], g['b'][i][:, :k]
hi = g['half'][i]       # the command limits this gait was optimized with
lo = -hi
ks = np.arange(1, k + 1)


def command(t):
  w = 2 * np.pi * ks * t / period
  return np.clip(c + a @ np.cos(w) + b @ np.sin(w), lo, hi)


direction = g['direction'][i]
p0 = d.qpos[:3].copy()
while d.time < cycles * period:
  d.ctrl[:] = command(d.time)
  mujoco.mj_step(m, d)
print(f'{name}: {cycles:g} cycles of T = {period:.3f} s, '
      f'mean speed {(d.qpos[:3] - p0) @ direction / d.time:.4f} m/s '
      f'along {direction} (from rest)')
