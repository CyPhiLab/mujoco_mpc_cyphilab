#!/usr/bin/env python3
"""Offscreen video rendering for pterosaur launch analysis.

Provides a CoM-tracking side-view Recorder (used by run_mpc_launch.py) and
renders the kinematic reference trajectory.

Usage (headless: MUJOCO_GL=osmesa or egl):
  render_launch.py reference [--out launch_reference.mp4]
  render_launch.py npz trajectory.npz [--out launch.mp4] [--title text]
    (any npz with time and qpos arrays, e.g. from launch_trajopt.py)
"""
import argparse
import os

import imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
TASK_DIR = os.path.dirname(HERE)

FPS = 30
WIDTH, HEIGHT = 640, 480


def make_camera():
  cam = mujoco.MjvCamera()
  cam.type = mujoco.mjtCamera.mjCAMERA_FREE
  cam.azimuth = 90      # side view of the sagittal (x-z) plane
  cam.elevation = -2      # near-horizontal, so body pitch reads true
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


def render_reference(path):
  m = mujoco.MjModel.from_xml_path(os.path.join(TASK_DIR, 'task.xml'))
  d = mujoco.MjData(m)
  ref = np.load(os.path.join(TASK_DIR, 'reference', 'launch.npz'))
  dt = ref['time'][1] - ref['time'][0]
  rec = Recorder(m)
  for t in np.arange(0, ref['time'][-1] + 1e-9, 1 / FPS):
    i = int(round(t / dt))
    d.qpos[:] = ref['qpos'][i]
    d.qvel[:] = ref['qvel'][i]
    mujoco.mj_forward(m, d)
    rec.capture(d, f'reference (kinematic)  t={t:.2f}s')
  rec.save(path)


def render_trajectory(time, qpos, title, path, comvel=None):
  m = mujoco.MjModel.from_xml_path(os.path.join(TASK_DIR, 'task.xml'))
  d = mujoco.MjData(m)
  rec = Recorder(m)
  for ft in np.arange(time[0], time[-1], 1 / FPS):
    k = min(int(np.searchsorted(time, ft)), len(time) - 1)
    d.qpos[:] = qpos[k]
    mujoco.mj_forward(m, d)
    text = f'{title}  t={ft - time[0]:.2f}s'
    if comvel is not None:
      text += f'  |v_com|={np.linalg.norm(comvel[k]):.2f} m/s'
    rec.capture(d, text)
  rec.save(path)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('what', choices=['reference', 'npz'])
  parser.add_argument('npz', nargs='?')
  parser.add_argument('--out', default='launch.mp4')
  parser.add_argument('--title', default='trajectory')
  args = parser.parse_args()
  if args.what == 'reference':
    render_reference(args.out)
  else:
    data = np.load(args.npz)
    render_trajectory(data['time'], data['qpos'], args.title, args.out,
                      data['comvel'] if 'comvel' in data else None)


if __name__ == '__main__':
  main()
