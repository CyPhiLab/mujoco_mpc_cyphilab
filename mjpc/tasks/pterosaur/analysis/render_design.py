#!/usr/bin/env python3
"""Render a design_search.py result: slow-motion side-view video and a
frame strip of the push.

Usage (headless: MUJOCO_GL=osmesa):
  render_design.py designs_scaled KEY [--slow 3]
"""
import argparse
import json
import os

import mujoco
import numpy as np
from PIL import Image

import design_search as DS
import render_launch as RL


def main():
  p = argparse.ArgumentParser()
  p.add_argument('dir')
  p.add_argument('key')
  p.add_argument('--slow', type=float, default=3.0)
  p.add_argument('--frames', type=int, default=16)
  args = p.parse_args()
  info = json.load(open(os.path.join(args.dir, 'summary.json')))[args.key]
  design = DS.Design(**info['design'])
  opt = DS.DesignOpt(design)
  tr = np.load(os.path.join(args.dir, args.key, 'trajectory.npz'))
  r = info['result']
  title = (f"{args.key}: {r.get('speed', 0):.1f} m/s @ {r.get('angle_deg', 0):.0f} deg "
           f"(target {design.target_speed:g} @ 30)")
  out = os.path.join(args.dir, args.key)
  # video: the push and the first part of the flight
  end = min(len(tr['time']), int((r.get('takeoff_time', 0.5) + 0.3) / 0.002))
  RL.render_trajectory(tr['time'][:end], tr['qpos'][:end], title,
                       os.path.join(out, 'launch.mp4'), tr['comvel'][:end],
                       slow=args.slow, m=opt.m)
  # frame strip from the crouch to 0.1 s after takeoff
  t_end = r.get('takeoff_time', 0.5) + 0.1
  m, d = opt.m, mujoco.MjData(opt.m)
  rec = RL.Recorder(m)
  rec.cam.distance = 4.5 * design.body_scale
  for t in np.linspace(0, t_end, args.frames):
    k = min(int(np.searchsorted(tr['time'], t)), len(tr['time']) - 1)
    d.qpos[:] = tr['qpos'][k]
    mujoco.mj_forward(m, d)
    rec.capture(d, f"t={t:.3f} v={np.linalg.norm(tr['comvel'][k]):.1f} "
                   f"hands {int(tr['hands_contact'][k])} feet {int(tr['feet_contact'][k])}")
  W, H = RL.WIDTH // 2, RL.HEIGHT // 2
  cols = 4
  rows = (len(rec.frames) + cols - 1) // cols
  sheet = Image.new('RGB', (cols * W, rows * H))
  for i, f in enumerate(rec.frames):
    sheet.paste(Image.fromarray(f).resize((W, H)), ((i % cols) * W, (i // cols) * H))
  sheet.save(os.path.join(out, 'frames.png'))
  print('wrote', out)


if __name__ == '__main__':
  main()
