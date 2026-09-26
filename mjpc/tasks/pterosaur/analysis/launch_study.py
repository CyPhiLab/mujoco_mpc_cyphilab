#!/usr/bin/env python3
"""Launch design study: what does a 10 m/s, 30 deg launch take?

Runs launch_ilqr.py's optimizer for a grid of designs (torque scale,
arm/leg torque split, latched springs, hand pads, contact force cap) in
parallel, and reports for each the takeoff and the checks that tell a real
launch from an impact artifact: hand/foot contact episodes before takeoff,
peak limb contact force, hand sink, spin, pitch, and the energy budget
(positive motor work, spring energy released, CoM energy at takeoff).

Usage:
  launch_study.py GRID [--iters 150] [--workers 4] [--out DIR]
  GRIDs: actuation (torque x springs, plain hands), hands (see GRIDS)
"""
import argparse
import itertools
import json
import os
import time
from multiprocessing import Pool

import mujoco
import numpy as np

import launch_ilqr as L

HERE = os.path.dirname(os.path.abspath(__file__))

PAD = dict(radius=0.04, timeconst=0.0067, dampratio=3.0)


def grid(name):
  if name == 'actuation':
    return [dict(torque=t, spring_energy=e)
            for t, e in itertools.product([2, 4, 6, 8], [0, 1000, 2000, 4000])]
  if name == 'hands':
    configs = []
    for t, e in [(4, 2000), (6, 0), (6, 2000), (8, 0)]:
      configs += [dict(torque=t, spring_energy=e, hand=PAD, vault_time=0.08),
                  dict(torque=t, spring_energy=e, force_cap=1000),
                  dict(torque=t, spring_energy=e, hand=PAD, vault_time=0.08,
                       force_cap=1000)]
    return configs
  if name == 'chains':
    # continuation: each step warm-starts from the previous step's solution
    return [
        [dict(torque=t) for t in [2, 3, 4, 5, 6, 8]],
        [dict(torque=4, spring_energy=e) for e in [0, 500, 1000, 2000, 3000, 4000]],
        [dict(torque=6, spring_energy=e) for e in [0, 500, 1000, 2000, 3000, 4000]],
        [dict(torque=t, spring_energy=1000) for t in [2, 3, 4, 5, 6, 8]],
    ]
  if name == 'split':
    return [dict(torque=4, arm_scale=a, leg_scale=l, spring_energy=e)
            for (a, l), e in itertools.product(
                [(0.5, 1.5), (1.5, 0.5), (1, 2), (2, 1)], [0, 2000])]
  raise ValueError(name)


def config_name(c):
  parts = [f"t{c['torque']:g}"]
  if c.get('arm_scale', 1) != 1 or c.get('leg_scale', 1) != 1:
    parts.append(f"a{c.get('arm_scale', 1):g}l{c.get('leg_scale', 1):g}")
  parts.append(f"s{c.get('spring_energy', 0):g}")
  if c.get('hand'):
    parts.append('pad')
  if c.get('vault_time'):
    parts.append('vault')
  if c.get('force_cap'):
    parts.append(f"cap{c['force_cap']:g}")
  return '_'.join(parts)


def launch_metrics(solver, U):
  """Contact episodes, peak forces, sink and energy up to takeoff."""
  m, d = solver.m, solver.d
  d.qpos[:], d.qvel[:] = solver.q0, solver.v0
  mujoco.mj_forward(m, d)
  spring0 = spring_energy(m, d)
  com_z0 = d.subtree_com[1][2]
  forces, hand_h = [], []
  work = 0.0
  for t in range(solver.n_push):
    d.ctrl[:] = L.E @ np.clip(U[t], -1, 1)
    mujoco.mj_step(m, d)
    forces.append(solver.limb_forces())
    hand_h.append(min(solver.limb(g)[0] for g in solver.hands))
    dof = m.actuator_trnid[:, 0]
    power = d.actuator_force * d.qvel[m.jnt_dofadr[dof]]
    work += np.maximum(power, 0).sum() * solver.dt
  forces = np.array(forces)
  episodes = []
  for c in range(4):
    on = np.r_[0, (forces[:, c] > 1.0).astype(int), 0]
    episodes.append(int((np.diff(on) == 1).sum()))
  com_energy = (0.5 * m.body_subtreemass[1] * np.sum(d.subtree_linvel[1] ** 2)
                + m.body_subtreemass[1] * 9.81 *
                (d.subtree_com[1][2] - com_z0))
  return {
      'hand_episodes': episodes[:2], 'foot_episodes': episodes[2:],
      'peak_hand_force_N': float(forces[:, :2].max()),
      'peak_foot_force_N': float(forces[:, 2:].max()),
      'hand_sink_mm': float(-min(hand_h) * 1000),
      'motor_work_J': float(work),
      'spring_released_J': float(spring0 - spring_energy(m, d)),
      'com_energy_J': float(com_energy),
  }


def spring_energy(m, d):
  e = 0.0
  for j in range(1, m.njnt):
    dq = d.qpos[m.jnt_qposadr[j]] - m.qpos_spring[m.jnt_qposadr[j]]
    e += 0.5 * m.jnt_stiffness[j] * dq * dq
  return e


def run(args, init=None):
  config, iters, out_dir, speed, angle, push_time = args
  name = config_name(config)
  kw = dict(config)
  torque = kw.pop('torque')
  t0 = time.time()
  solver = L.LaunchILQR(push_time, torque, speed, angle, **kw)
  U0 = (init[:solver.N] if init is not None
        else solver.opt.pd_warm_start(push_time)[:solver.N])
  U, cost, history = solver.solve(U0, iters=iters, verbose=False)
  half, det = solver.report(U)
  metrics = launch_metrics(solver, U)
  np.save(os.path.join(out_dir, name + '_controls.npy'), half)
  row_controls = half
  solver.opt.record(half, os.path.join(out_dir, name + '.npz'))
  row = {'name': name, 'config': {k: v for k, v in config.items() if k != 'hand'},
         'hand_pad': bool(config.get('hand')), 'cost': float(cost),
         'iterations': len(history) - 1, 'seconds': round(time.time() - t0),
         **{k: det[k] for k in ('took_off', 'speed', 'angle_deg', 'takeoff_time',
                                'spin_per_mass', 'max_pitch_deg')},
         **metrics}
  return (row, row_controls) if init is not None else row


def run_chain(args):
  """Continuation: configs in order, each from the previous solution."""
  chain, iters, out_dir, speed, angle, push_time, init_file = args
  init = np.load(init_file)
  rows = []
  for config in chain:
    row, init = run((config, iters, out_dir, speed, angle, push_time), init)
    row['chain_from'] = rows[-1]['name'] if rows else os.path.basename(init_file)
    rows.append(row)
    print_row(row)
  return rows


def print_row(row):
  print(f"{row['name']:28s} {row['speed']:5.2f} m/s @ {row['angle_deg']:5.1f} deg "
        f"t={row['takeoff_time']:.2f} spin {row['spin_per_mass']:.2f} "
        f"hands {row['hand_episodes']} feet {row['foot_episodes']} "
        f"peak hand {row['peak_hand_force_N']:.0f} N sink {row['hand_sink_mm']:.1f} mm "
        f"work {row['motor_work_J']:.0f}+{row['spring_released_J']:.0f} J -> "
        f"{row['com_energy_J']:.0f} J  [{row['iterations']} it, {row['seconds']} s]",
        flush=True)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('grid')
  parser.add_argument('--iters', type=int, default=150)
  parser.add_argument('--workers', type=int, default=4)
  parser.add_argument('--speed', type=float, default=10.0)
  parser.add_argument('--angle', type=float, default=30.0)
  parser.add_argument('--push_time', type=float, default=0.45)
  parser.add_argument('--out', default=None)
  parser.add_argument('--init', default=os.path.join(
      HERE, 'candidates', 'A_plain_hands', 'controls.npy'),
      help='chains: controls to start every chain from')
  args = parser.parse_args()
  out_dir = args.out or os.path.join(HERE, 'study_' + args.grid)
  os.makedirs(out_dir, exist_ok=True)
  configs = grid(args.grid)
  results = []
  with Pool(args.workers) as pool:
    if args.grid == 'chains':
      jobs = [(chain, args.iters, out_dir, args.speed, args.angle,
               args.push_time, args.init) for chain in configs]
      for rows in pool.imap_unordered(run_chain, jobs):
        results += rows
        with open(os.path.join(out_dir, 'results.json'), 'w') as f:
          json.dump(results, f, indent=1)
      return
    jobs = [(c, args.iters, out_dir, args.speed, args.angle, args.push_time)
            for c in configs]
    for row in pool.imap_unordered(run, jobs):
      results.append(row)
      print_row(row)
      with open(os.path.join(out_dir, 'results.json'), 'w') as f:
        json.dump(results, f, indent=1)


if __name__ == '__main__':
  main()
