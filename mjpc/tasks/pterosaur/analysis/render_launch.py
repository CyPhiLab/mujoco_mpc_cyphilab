#!/usr/bin/env python3
"""Render launch videos: the reference and optimized design variants.

Usage (headless: MUJOCO_GL=osmesa or egl):
  render_launch.py reference [--out ref.mp4]
  render_launch.py optimize --torque 2 --spring 500 [--no-load-speed 15]
  render_launch.py results launch_design_results.json [--index 3]

'optimize' runs the design-study optimizer for one variant and renders the
best launch; 'results' renders controls saved by launch_design_study.py.
Simulated clips continue with zero control after the push to show flight.
"""
import argparse
import os

import imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from launch_design_study import (HORIZON, REF_DT, SIM_DT, T_START, Study,
                                 expand_controls)

FPS = 30
WIDTH, HEIGHT = 640, 480
FLIGHT = 0.8  # seconds rendered after the push horizon


def make_camera():
  cam = mujoco.MjvCamera()
  cam.type = mujoco.mjtCamera.mjCAMERA_FREE
  cam.azimuth = 90      # side view of the sagittal (x-z) plane
  cam.elevation = -10
  cam.distance = 4.5
  return cam


def label(frame, text):
  img = Image.fromarray(frame)
  draw = ImageDraw.Draw(img)
  draw.rectangle([0, 0, WIDTH, 24], fill=(0, 0, 0))
  draw.text((8, 6), text, fill=(255, 255, 255))
  return np.asarray(img)


class Recorder:

  def __init__(self, m):
    self.m = m
    self.renderer = mujoco.Renderer(m, HEIGHT, WIDTH)
    self.cam = make_camera()
    self.frames = []

  def capture(self, d, text):
    self.cam.lookat[:] = d.subtree_com[1]
    self.cam.lookat[2] = max(self.cam.lookat[2], -0.5)
    self.renderer.update_scene(d, self.cam)
    self.frames.append(label(self.renderer.render(), text))

  def save(self, path):
    imageio.mimsave(path, self.frames, fps=FPS, macro_block_size=1)
    print('wrote', path, f'({len(self.frames)} frames)')


def render_reference(study, path):
  m = study.base
  d = mujoco.MjData(m)
  rec = Recorder(m)
  n = len(study.ref_qpos)
  for t in np.arange(0, (n - 1) * REF_DT + 1e-9, 1 / FPS):
    i = int(round(t / REF_DT))
    d.qpos[:] = study.ref_qpos[i]
    d.qvel[:] = study.ref_qvel[i]
    mujoco.mj_forward(m, d)
    rec.capture(d, f'reference (kinematic)  t={t:.2f}s')
  rec.save(path)


def render_simulated(study, m, params, title, path):
  x0 = study.initial_state(m)
  ctrl = expand_controls(np.asarray(params)[None], study.signs, study.nstep)[0]
  d = mujoco.MjData(m)
  mujoco.mj_setState(m, d, x0, mujoco.mjtState.mjSTATE_FULLPHYSICS)
  mujoco.mj_forward(m, d)
  rec = Recorder(m)
  total = int(round((HORIZON + FLIGHT) / SIM_DT))
  every = int(round(1 / (FPS * SIM_DT)))
  for k in range(total):
    d.ctrl[:] = ctrl[k] if k < len(ctrl) else 0
    if k % every == 0:
      v = np.linalg.norm(d.subtree_linvel[1])
      rec.capture(d, f'{title}  t={T_START + k * SIM_DT:.2f}s  |v_com|={v:.2f} m/s')
    mujoco.mj_step(m, d)
  rec.save(path)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('what', choices=['reference', 'optimize', 'results'])
  parser.add_argument('results_file', nargs='?')
  parser.add_argument('--index', type=int, default=None)
  parser.add_argument('--torque', type=float, default=1.0)
  parser.add_argument('--spring', type=float, default=0.0)
  parser.add_argument('--no-load-speed', type=float, default=None)
  parser.add_argument('--iters', type=int, default=45)
  parser.add_argument('--out', default=None)
  args = parser.parse_args()

  study = Study()
  if args.what == 'reference':
    render_reference(study, args.out or 'launch_reference.mp4')
    return

  if args.what == 'optimize':
    variants = [(args.torque, args.spring, args.no_load_speed, None)]
  else:
    import json
    with open(args.results_file) as f:
      rows = json.load(f)
    if args.index is not None:
      rows = [rows[args.index]]
    variants = [(r['torque_scale'], r['spring_energy_J'],
                 None if r['motor'] == 'ideal' else float(r['motor'][3:-5]),
                 r['controls']) for r in rows]

  for ts, es, no_load, params in variants:
    m, _ = study.variant(ts, es, no_load)
    if params is None:
      runs = [study.optimize(m, iters=args.iters, seed=s) for s in range(2)]
      params, detail = max(runs, key=lambda r: r[1]['score'])
      print(f"takeoff {detail['speed']:.2f} m/s @ {detail['angle_deg']:.1f} deg")
    motor = 'ideal' if not no_load else f'DC {no_load:g} rad/s'
    title = f'torque x{ts:g}, spring {es:g} J, {motor}'
    name = f"launch_t{ts:g}_s{es:g}{'_dc' if no_load else ''}.mp4"
    render_simulated(study, m, params, title, args.out or name)


if __name__ == '__main__':
  main()
