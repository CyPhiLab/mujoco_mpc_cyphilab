#!/usr/bin/env python3
"""Replay the exported launch: python replay.py [--video out.mp4]"""
import os, sys
import mujoco
import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
m = mujoco.MjModel.from_xml_path(os.path.join(here, 'model.xml'))
z = np.load(os.path.join(here, 'trajectory.npz'))
d = mujoco.MjData(m)
d.qpos[:] = z['start_qpos']                 # starts at rest
mujoco.mj_forward(m, d)
err = 0.0
for t in range(len(z['ctrl'])):
  d.ctrl[:] = z['ctrl'][t]
  mujoco.mj_step(m, d)
  err = max(err, np.abs(d.qpos - z['qpos'][t]).max())   # qpos[t]: after step t
k = int(z['takeoff_step'])
v = z['comvel'][k]
print(f'max |qpos - recorded| {err:.2e}; takeoff t={z["time"][k]:.3f} s, '
      f'{np.linalg.norm(v):.2f} m/s at {np.degrees(np.arctan2(v[2], -v[0])):.1f} deg')
