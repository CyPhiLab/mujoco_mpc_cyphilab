#!/usr/bin/env python3
"""What it takes to swing the paddles through a kinematic reference stroke
at a given stroke period: inverse dynamics of the prescribed motion.

  stroke_dynamics.py REF.json --out DIR [--periods 0.3 0.5 0.8 1.1 2.0]

The joints follow the reference (kinematic_ref.py) exactly and the body
glides at the mode's speed; mj_inverse gives the joint torques that motion
needs, split into
  inertia    M(q) qacc (the legs' own mass and inertia, and the body's)
  coriolis   velocity-product terms (gravity is off: the robot floats)
  damping    joint damping x joint speed (the servo's speed limit: with a
             3.1 N m stall torque, damping 1 N m s/rad caps it at 3.1 rad/s)
  fluid      the fluid model's drag and lift on the moving legs (MuJoCo's
             ellipsoid model has no added mass)
and compared with the 3.1 N m servo limit. The same is done for design
variants of the paddle legs (crab_model design: paddle_mass, paddle_damping,
paddle_area) to see which ones bring a crab-rate stroke within reach.

Writes DIR/stroke_dynamics.json and DIR/stroke_dynamics.png.
"""
import argparse
import json
import os

import mujoco
import numpy as np

import crab_model as C
import kinematic_ref as K

PADDLE_JOINTS = ['psbr', 'msbr', 'dsbr', 'psbl', 'msbl', 'dsbl']
PARTS = ('inertia', 'coriolis', 'damping', 'fluid')

# name: (description, crab_model design overrides)
VARIANTS = {
    'robot': ('the robot as modelled', {}),
    'paddle_mass_x0.25': ('paddle legs\' mid and distal segments x0.25 mass '
                          'and inertia (motors moved inboard)',
                          {'paddle_mass': 0.25}),
    'paddle_damping_x0': ('paddle-joint damping off (ideal, unlimited-speed '
                          'servos)', {'paddle_damping': 0.0}),
    'paddle_damping_x0.25': ('paddle-joint damping x0.25 (4x faster servos, '
                             'same torque)', {'paddle_damping': 0.25}),
    'paddle_damping_x0.125': ('paddle-joint damping x0.125 (8x faster '
                              'servos)', {'paddle_damping': 0.125}),
    'fast_and_light': ('paddle damping x0.125 and paddle legs x0.25',
                       {'paddle_damping': 0.125, 'paddle_mass': 0.25}),
    'paddle_area_x0.5': ('paddle blade area x0.5', {'paddle_area': 0.5}),
}


def load(design):
  return C.load_model(dt=0.0005, ground=False, ctrl_limit=np.pi,
                      design=design)


def stroke_torques(m, pose, period, n=400):
  """Joint torques (n, nu) per part over one stroke at this period."""
  d = mujoco.MjData(m)
  dt = period / n
  t = np.arange(-2, n + 2) * dt
  qpos = np.array([K.qpos_at(pose, m, ti, period) for ti in t])
  qvel = np.zeros((len(t), m.nv))
  for i in range(len(t)):
    mujoco.mj_differentiatePos(m, qvel[i], 1.0, qpos[i - 1 if i else 0],
                               qpos[i])
  qvel[1:-1] = 0.5 * (qvel[1:-1] + qvel[2:]) / dt
  qvel[0] = qvel[-1] = 0
  qacc = np.gradient(qvel, dt, axis=0)
  out = {p: np.zeros((n, C.NU)) for p in PARTS + ('total',)}
  for k in range(n):
    i = k + 2
    d.qpos[:], d.qvel[:], d.qacc[:] = qpos[i], qvel[i], qacc[i]
    mujoco.mj_inverse(m, d)
    mq = np.zeros(m.nv)
    mujoco.mj_mulM(m, d, mq, d.qacc)
    out['inertia'][k] = mq[6:]
    out['coriolis'][k] = d.qfrc_bias[6:]
    out['damping'][k] = -d.qfrc_damper[6:]
    out['fluid'][k] = -d.qfrc_fluid[6:]
    out['total'][k] = d.qfrc_inverse[6:]
  return out, qvel[2:-2, 6:]


def summarize(torques, speeds):
  idx = [C.JOINTS.index(j) for j in PADDLE_JOINTS]
  rows = {}
  for j, i in zip(PADDLE_JOINTS, idx):
    rows[j] = dict(
        peak=float(np.abs(torques['total'][:, i]).max()),
        rms=float(np.sqrt(np.mean(torques['total'][:, i] ** 2))),
        peak_speed=float(np.abs(speeds[:, i]).max()),
        **{f'rms_{p}': float(np.sqrt(np.mean(torques[p][:, i] ** 2)))
           for p in PARTS})
  worst = max(rows.values(), key=lambda r: r['peak'])
  return dict(joints=rows, worst_peak=worst['peak'],
              worst_rms=max(r['rms'] for r in rows.values()),
              over_limit=float(np.mean(np.abs(torques['total'][:, idx])
                                       > C.FORCE_LIMIT)))


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawTextHelpFormatter)
  p.add_argument('reference')
  p.add_argument('--out', required=True)
  p.add_argument('--periods', type=float, nargs='+',
                 default=[0.3, 0.5, 0.8, 1.1, 2.0])
  args = p.parse_args()
  with open(args.reference) as f:
    ref = json.load(f)
  os.makedirs(args.out, exist_ok=True)
  result = dict(limit=C.FORCE_LIMIT, periods=args.periods, modes={})
  for mode in ref['modes']:
    pose = K.pose_spec(ref, mode)
    result['modes'][mode] = {}
    for name, (text, design) in VARIANTS.items():
      m = load(design)
      rows = {}
      for period in args.periods:
        torques, speeds = stroke_torques(m, pose, period)
        rows[str(period)] = summarize(torques, speeds)
        s = rows[str(period)]
        parts = {q: max(r[f'rms_{q}'] for r in s['joints'].values())
                 for q in PARTS}
        print(f'{mode:14s} {name:18s} T {period:4.2f}  peak {s["worst_peak"]:6.2f}'
              f' rms {s["worst_rms"]:6.2f} N m  over limit '
              f'{100 * s["over_limit"]:5.1f}%  worst-joint rms by part: '
              + ' '.join(f'{q} {v:5.2f}' for q, v in parts.items()),
              flush=True)
      result['modes'][mode][name] = dict(description=text, periods=rows)
  with open(os.path.join(args.out, 'stroke_dynamics.json'), 'w') as f:
    json.dump(result, f, indent=1)
  plot(result, os.path.join(args.out, 'stroke_dynamics.png'))


def plot(result, path):
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  modes = list(result['modes'])
  fig, axes = plt.subplots(1, len(modes), figsize=(7 * len(modes), 4.5),
                           squeeze=False)
  for ax, mode in zip(axes[0], modes):
    for name, row in result['modes'][mode].items():
      periods = [float(t) for t in row['periods']]
      ax.plot(periods, [r['worst_peak'] for r in row['periods'].values()],
              'o-', label=name)
    ax.axhline(result['limit'], color='k', ls='--', lw=1,
               label='servo limit 3.1 N m')
    ax.axvspan(0.26, 0.32, color='0.85', label='crab stroke period')
    ax.set_yscale('log')
    ax.set_xlabel('stroke period (s)')
    ax.set_ylabel('peak paddle-joint torque needed (N m)')
    ax.set_title(f'{mode}: torque to follow the reference stroke')
    ax.legend(fontsize=7)
  fig.tight_layout()
  fig.savefig(path, dpi=90)
  plt.close(fig)


if __name__ == '__main__':
  main()
