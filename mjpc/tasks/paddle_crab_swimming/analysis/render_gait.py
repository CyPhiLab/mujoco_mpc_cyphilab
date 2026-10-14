#!/usr/bin/env python3
"""Video and plots of an optimized gait.

Usage:
  MUJOCO_GL=osmesa render_gait.py STEP.npz [--cycles 6] [--dt 0.001]
      [--camera track] [--slow 1]

Writes STEP.mp4 (the gait from rest, camera following the body) and
STEP_plots.png (xy path, speed, and the joint/command limit cycle).
"""
import argparse
import os

import mujoco
import numpy as np

import crab_model as C
import gait_opt as G


def rollout_states(theta, k, dt, cycles, x0=None, half=None, design=None,
                   tau=C.TAU):
  layout = G.Layout(k)
  m = C.load_model(dt=dt, design=design, tau=tau)
  if half is not None:     # the command ranges the gait was optimized with
    m.actuator_ctrlrange[:] = np.stack([-half, half], axis=1)
  d = mujoco.MjData(m)
  if x0 is not None:
    mujoco.mj_setState(m, d, x0, C.STATE_SPEC)
  period = layout.period(theta)
  n = int(np.ceil(cycles * period / dt))
  half = m.actuator_ctrlrange[:, 1]
  ctrl, _ = G.controls(layout, theta, half, n, dt)
  return m, d, ctrl


def design_of(z):
  """The design variant and servo filter a saved step was optimized with."""
  import json
  return dict(design=json.loads(str(z['design'])) if 'design' in z else None,
              tau=float(z['tau']) if 'tau' in z else C.TAU)


def render(path, theta, k, dt=C.SIM_DT, cycles=6, camera='track',
           fps=30, slow=1.0, width=640, height=480, half=None, design=None,
           tau=C.TAU):
  import imageio
  m, d, ctrl = rollout_states(theta, k, dt, cycles, half=half, design=design,
                              tau=tau)
  renderer = mujoco.Renderer(m, height, width)
  plate = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'plate')
  cam = mujoco.MjvCamera()
  if camera == 'track':
    # Three-quarter view from the side, following the body; forward (-y)
    # runs right to left across the frame.
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.azimuth, cam.elevation = 1.4, 150.0, -28.0
  else:
    cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    cam.fixedcamid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, camera)
  mujoco.mj_forward(m, d)
  frame_dt = slow / fps
  next_frame = 0.0
  frames = []
  for u in ctrl:
    if d.time >= next_frame:
      cam.lookat[:] = d.xpos[plate]
      renderer.update_scene(d, cam)
      frames.append(renderer.render())
      next_frame += frame_dt
    d.ctrl[:] = u
    mujoco.mj_step(m, d)
  imageio.mimsave(path, frames, fps=fps, macro_block_size=1)
  return frames


def plot(path, npz):
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  z = np.load(npz, allow_pickle=True)
  theta, k, dt = z['theta'], int(z['k']), float(z['dt'])
  m, d, ctrl = rollout_states(theta, k, C.SIM_DT, 8,
                              half=z['half'] if 'half' in z else None,
                              **design_of(z))
  pos, t = [], []
  for u in ctrl:
    d.ctrl[:] = u
    mujoco.mj_step(m, d)
    pos.append(d.qpos[:3].copy())
    t.append(d.time)
  pos, t = np.array(pos), np.array(t)
  fig, ax = plt.subplots(2, 2, figsize=(11, 8))
  ax[0, 0].plot(pos[:, 0], pos[:, 1])
  direction = z['direction'] if 'direction' in z else C.FORWARD
  ax[0, 0].set(xlabel='x (m)', ylabel='y (m)',
               title=f'path (swim direction {np.round(direction, 2)})')
  ax[0, 0].axis('equal')
  period = float(np.exp(theta[0]))
  w = max(1, int(round(period / C.SIM_DT)))
  y = pos @ direction
  v = (y[w:] - y[:-w]) / (t[w:] - t[:-w])
  ax[0, 1].plot(t[w:], v)
  ax[0, 1].set(xlabel='t (s)', ylabel='cycle-mean speed along direction (m/s)')
  phase = z['phase']
  for i, name in enumerate(C.JOINTS):
    if name.startswith('d') and not name.startswith('dsb'):
      continue
    line, = ax[1, 0].plot(phase, z['qpos'][:, 7 + i], label=name)
    ax[1, 1].plot(phase, z['ctrl'][:, i], color=line.get_color())
  ax[1, 0].set(xlabel='phase', ylabel='joint angle (rad)')
  ax[1, 1].set(xlabel='phase', ylabel='servo command (rad)')
  ax[1, 0].legend(fontsize=7, ncol=2)
  fig.suptitle(os.path.basename(npz))
  fig.tight_layout()
  fig.savefig(path, dpi=90)
  plt.close(fig)


def main():
  p = argparse.ArgumentParser()
  p.add_argument('npz')
  p.add_argument('--cycles', type=float, default=6)
  p.add_argument('--dt', type=float, default=C.SIM_DT)
  p.add_argument('--camera', default='track',
                 help="'track' (follows the body) or a model camera name")
  p.add_argument('--slow', type=float, default=1.0)
  args = p.parse_args()
  z = np.load(args.npz, allow_pickle=True)
  base = os.path.splitext(args.npz)[0]
  render(base + '.mp4', z['theta'], int(z['k']), args.dt, args.cycles,
         args.camera, slow=args.slow, half=z['half'] if 'half' in z else None,
         **design_of(z))
  plot(base + '_plots.png', args.npz)
  print('wrote', base + '.mp4', base + '_plots.png')


if __name__ == '__main__':
  main()
