#!/usr/bin/env python3
"""Re-render launch_track.py steps (video and frame strip) with the current
render settings.

Usage:
  MUJOCO_GL=egl rerender_track.py RUN_DIR [STEP_NAME ...]
"""
import json
import os
import sys

import numpy as np

import launch_track as LT


def main():
  run, names = sys.argv[1], sys.argv[2:]
  for row in json.load(open(os.path.join(run, 'summary.json'))):
    if names and row['name'] not in names:
      continue
    solver, _ = LT.solver_for_step(run, row)
    traj = dict(np.load(os.path.join(run, row['name'] + '.npz')))
    LT.render(solver.opt, traj, row,
              f"{row['name']}: {row['speed']:.1f} m/s @ {row['angle_deg']:.0f} deg",
              os.path.join(run, row['name']))


if __name__ == '__main__':
  main()
