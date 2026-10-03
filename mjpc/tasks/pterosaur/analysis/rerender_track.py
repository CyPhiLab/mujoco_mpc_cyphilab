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
  args = LT.parser().parse_args([])
  vars(args).update(json.load(open(os.path.join(run, 'args.json'))))
  design = LT.make_design(args, verbose=False)
  for row in json.load(open(os.path.join(run, 'summary.json'))):
    if names and row['name'] not in names:
      continue
    solver = LT.TrackILQR(row['push_time'], args.torque, row['target_speed'],
                          track_scale=row['track_scale'],
                          min_hand_force=args.min_hand_force,
                          reference=args.reference, angle=args.angle,
                        window=args.window or None, free_hands=args.free_hands,
                        track_index=args.track_index, hand_load=args.hand_load,
                        hand_load_weight=args.hand_load_weight,
                        **dict(design, contact_smoothing=row.get('contact_smoothing', 0.0)))
    traj = dict(np.load(os.path.join(run, row['name'] + '.npz')))
    LT.render(solver.opt, traj, row,
              f"{row['name']}: {row['speed']:.1f} m/s @ {row['angle_deg']:.0f} deg",
              os.path.join(run, row['name']))


if __name__ == '__main__':
  main()
