#!/usr/bin/env python3
"""What the walking legs do in a gait: drag share, forces, moments, work, and
what happens without their motion.

  leg_audit.py STEP.npz [--out runs/leg_audit_NAME]

On the steady limit cycle (STEADY_WARM cycles from rest, 1 ms):

  forces    the fluid force on the robot from each part (body = plate, servo
            housings, cylinder; each walking leg R1 R2 L1 L2; each paddle leg
            R3 L3), split into the swim direction (+ = thrust), lateral and
            vertical, and the fluid moment about the base (roll, pitch, yaw
            in the body frame); cycle mean and RMS oscillation. MuJoCo's
            ellipsoid fluid model sums per geom, so a part's share is the force
            with only its geoms interacting.
  tow       the whole robot held at the gait's mean posture, towed at the
            gait's speed: each part's share of the passive drag
  work      positive servo work per cycle, per leg
  freeze    the same gait with the walking legs' commands held at their mean
            (no oscillation), or folded (a tucked pose), on the same physics:
            speed, drift, yaw, tilt
"""
import argparse
import json

import mujoco
import numpy as np

import crab_model as C
import gait_opt as G
from primitive_report import problem_for

STEADY_WARM = 12
LEGS = {'R1': 0, 'R2': 3, 'R3': 6, 'L1': 9, 'L2': 12, 'L3': 15}
WALKING = ['R1', 'R2', 'L1', 'L2']
# Body name fragments of each leg's segments in robot_xdomain.xml.
LEG_BODY = {'R1': ('front', 'right'), 'R2': ('center', 'right'),
            'R3': ('back', 'right'), 'L1': ('front', 'left'),
            'L2': ('center', 'left'), 'L3': ('back', 'left')}


def part_of_geom(m):
  """Part label of every geom: 'body' or a leg name."""
  out = []
  for g in range(m.ngeom):
    name = (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
            or '').lower()
    label = 'body'
    if 'segment' in name:
      for leg, (row, side) in LEG_BODY.items():
        if row in name and side in name:
          label = leg
    out.append(label)
  return np.array(out)


def leg_of_joint_body(m):
  """Sanity check: each leg's joints' bodies carry that leg's label."""
  parts = part_of_geom(m)
  labels = {}
  for leg, i0 in LEGS.items():
    for j in range(3):
      jid = m.actuator_trnid[i0 + j, 0]
      b = m.jnt_bodyid[jid]
      geoms = np.where(m.geom_bodyid == b)[0]
      labels[C.JOINTS[i0 + j]] = sorted(set(parts[geoms]))
  return labels


def forces(m, cycle, direction):
  """Per part: mean and RMS of the fluid force (along, lateral, vertical; N)
  and moment about the base (roll, pitch, yaw, body frame; N m)."""
  d = mujoco.MjData(m)
  parts = part_of_geom(m)
  lateral, vertical = G.drift_axes(direction)
  axes = np.stack([direction, lateral, vertical])
  fluid = m.geom_fluid.copy()
  out = {}
  for part in ['all', 'body'] + list(LEGS):
    m.geom_fluid[:] = fluid
    if part != 'all':
      m.geom_fluid[parts != part, 0] = 1e-12
    f, t = [], []
    for q, v, a in zip(cycle['qpos'], cycle['qvel'], cycle['act']):
      d.qpos[:], d.qvel[:], d.act[:] = q, v, a
      mujoco.mj_forward(m, d)
      f.append(axes @ d.qfrc_fluid[:3])        # free joint: world force
      t.append(d.qfrc_fluid[3:6].copy())       # body-frame moment at base
    f, t = np.array(f), np.array(t)
    out[part] = dict(force_mean=f.mean(0).tolist(),
                     force_rms=f.std(0).tolist(),
                     moment_mean=t.mean(0).tolist(),
                     moment_rms=t.std(0).tolist())
  m.geom_fluid[:] = fluid
  return out


def tow(m, cycle, direction, speed):
  """Passive drag per part, robot at its mean posture towed at speed."""
  d = mujoco.MjData(m)
  parts = part_of_geom(m)
  fluid = m.geom_fluid.copy()
  out = {}
  for part in ['all', 'body'] + list(LEGS):
    m.geom_fluid[:] = fluid
    if part != 'all':
      m.geom_fluid[parts != part, 0] = 1e-12
    d.qpos[:] = cycle['qpos'].mean(axis=0)
    d.qpos[3:7] = cycle['qpos'][0, 3:7]
    d.qvel[:] = 0
    d.qvel[:3] = speed * direction
    mujoco.mj_forward(m, d)
    out[part] = -float(d.qfrc_fluid[:3] @ direction)
  m.geom_fluid[:] = fluid
  return out


def work(problem, cycle, period):
  q, qd = cycle['qpos'][:, 7:], cycle['qvel'][:, 6:]
  force = C.actuator_force(problem.m, q, cycle['act'])
  dt = period / len(q)
  pos = np.maximum(force * qd, 0).sum(0) * dt
  stroke = np.ptp(q, axis=0)
  return {leg: dict(work=float(pos[i0:i0 + 3].sum()),
                    stroke=np.round(stroke[i0:i0 + 3], 3).tolist())
          for leg, i0 in LEGS.items()}


def frozen(layout, theta, half, pose=None):
  """theta with the walking legs' oscillation removed: held at their mean
  command, or at pose (radians, 12 numbers for R1 R2 L1 L2) if given."""
  log_t, c, a, b = layout.split(theta)
  c, a, b = c.copy(), a.copy(), b.copy()
  for n, leg in enumerate(WALKING):
    i0 = LEGS[leg]
    a[i0:i0 + 3] = 0
    b[i0:i0 + 3] = 0
    if pose is not None:
      c[i0:i0 + 3] = np.asarray(pose[3 * n:3 * n + 3]) / half[i0:i0 + 3]
  return layout.join(np.exp(log_t), c, a, b)


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawTextHelpFormatter)
  p.add_argument('step')
  p.add_argument('--out')
  a = p.parse_args()
  z = np.load(a.step, allow_pickle=True)
  C.set_robot(C.step_robot(z))
  problem = problem_for(z, C.SIM_DT, STEADY_WARM, measure=2)
  theta = z['theta']
  _, info = problem.evaluate(theta[None])
  base = info[0]
  cycle = problem.limit_cycle(theta)
  direction = problem.direction
  period = base['period']
  m = problem.m
  print('joint -> part check:', leg_of_joint_body(m))

  F = forces(m, cycle, direction)
  T = tow(m, cycle, direction, base['speed'])
  W = work(problem, cycle, period)

  # Freeze tests on the same physics and window.
  tests = {'as optimized': theta,
           'walking legs held at their mean': frozen(problem.layout, theta,
                                                     problem.half)}
  # Folded: proximal swept back toward the body's long axis, mid raised
  # flat against the plate edge, distal folded in (task ranges clip it).
  fold = [0.6, 1.2, -1.0,  0.0, 1.2, -1.0,   # R1 R2 (proximal, mid, distal)
          -0.6, 1.2, -1.0, 0.0, 1.2, -1.0]   # L1 L2
  tests['walking legs folded'] = frozen(problem.layout, theta, problem.half,
                                        fold)
  freeze = {}
  for label, th in tests.items():
    _, inf = problem.evaluate(th[None])
    mm = inf[0]
    freeze[label] = {k: mm[k] for k in ('speed', 'lateral_rate', 'yaw_rate',
                                        'depth_rate', 'tilt_mean', 'tilt_rms',
                                        'effort', 'work_per_cycle', 'cot')}

  # Report.
  print(f'\n{a.step}: steady speed {base["speed"]:.4f} m/s, T {period:.3f} s')
  total_drag = T['all']
  print(f'\nTow at {base["speed"]:.3f} m/s, mean posture (passive drag, N):'
        f' total {total_drag:.3f}')
  for part in ['body'] + list(LEGS):
    print(f'  {part:5s} {T[part]:+.3f}  ({100 * T[part] / total_drag:4.1f}%)')
  walk = sum(T[l] for l in WALKING)
  print(f'  walking legs together {walk:+.3f} ({100 * walk / total_drag:.1f}%)')
  print('\nFluid force on the swimming robot, cycle mean (RMS):')
  print('  part   along(N)        lateral(N)      vertical(N)     '
        'roll(Nm)        pitch(Nm)       yaw(Nm)')
  for part, r in F.items():
    cols = [f'{mu:+.3f} ({sd:.3f})' for mu, sd in
            zip(r['force_mean'] + r['moment_mean'],
                r['force_rms'] + r['moment_rms'])]
    print(f'  {part:5s} ' + '  '.join(f'{c:14s}' for c in cols))
  print('\nPositive servo work per cycle (J) and joint strokes (rad):')
  for leg, r in W.items():
    print(f'  {leg}  {r["work"]:6.2f}  {r["stroke"]}')
  wl = sum(W[l]['work'] for l in WALKING)
  wt = sum(r['work'] for r in W.values())
  print(f'  walking legs {wl:.2f} of {wt:.2f} J ({100 * wl / wt:.0f}%)')
  print('\nWithout the walking legs\' motion (same physics, steady window):')
  for label, r in freeze.items():
    print(f'  {label:32s} v {r["speed"]:.4f}  lat {r["lateral_rate"]:+.4f}  '
          f'yaw {r["yaw_rate"]:+.4f}  tilt {r["tilt_mean"]:.3f} '
          f'(osc {r["tilt_rms"]:.3f})  effort {r["effort"]:.3f}  '
          f'work {r["work_per_cycle"]:.1f} J')
  if a.out:
    with open(a.out + '.json', 'w') as f:
      json.dump(dict(step=a.step, speed=base['speed'], period=period,
                     tow=T, forces=F, work=W, freeze=freeze), f, indent=1)


if __name__ == '__main__':
  main()
