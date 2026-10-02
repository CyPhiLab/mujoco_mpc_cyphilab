#!/usr/bin/env python3
"""Spring design study with the reduced-order launch model.

For each motor (torque scale, no-load speed) and spring energy budget, the
springs (see reduced_launch.SPRING_JOINTS) are optimized together with the
push, with progressively more design freedom:
  legacy:  the earlier springs (all four joints, equal energy, linear,
           resting at the far end of the joint range, no stop)
  linear:  energy split and travel (direction and length) per joint, linear,
           all released at push start, with a stop at the end of travel
  shaped:  + torque profile per joint (constant / linear / progressive)
  staged:  + latch release time per limb pair (hind, fore)
  and, at full freedom, springs on subsets of joints only.

The 'robust' grid repeats the no-spring and fully free ('staged') designs
with several starts per design: two cold starts and a warm start from the
next smaller energy budget's solution (a chain per motor and force cap),
keeping the best, since single cold starts sometimes stall.

Usage:
  reduced_springs.py [--grid main|robust|scaling|subsets] [--out FILE]
      [--workers 4]
"""
import argparse
import itertools
import json
import time
from dataclasses import asdict
from multiprocessing import Pool

import numpy as np

import reduced_launch as R

MOTORS = [dict(torque_scale=1, no_load_speed=20),
          dict(torque_scale=2, no_load_speed=20),
          dict(torque_scale=4, no_load_speed=20),
          dict(torque_scale=6, no_load_speed=40)]
ENERGIES = [500, 1000, 2000, 4000]
# peak total ground force caps (N): about 5 and 10 body weights. Without a
# cap the optimizer turns the springs into impulsive kicks through the
# massless limbs.
FORCE_CAPS = [2500, 5000]
VARIANTS = {
    'linear': dict(),
    'shaped': dict(shape=True),
    'staged': dict(shape=True, staged=True),
}
MAIN_VARIANTS = ['legacy', 'linear', 'staged']
SUBSETS = {
    'middle': [0, 1, 0, 1],      # knees and elbows
    'proximal': [1, 0, 1, 0],    # hips and shoulders
    'hind': [1, 1, 0, 0],
    'fore': [0, 0, 1, 1],
}


def jobs(grid, iters):
  out = []
  for motor, cap in itertools.product(MOTORS, FORCE_CAPS):
    motor = dict(motor, force_cap=cap)
    if grid == 'main':
      out.append((motor, 0, 'none', None, iters))
    for e in ENERGIES:
      if grid == 'main':
        out += [(motor, e, name, VARIANTS.get(name), iters)
                for name in MAIN_VARIANTS]
      elif grid == 'subsets':
        out += [(motor, e, name, dict(shape=True, staged=True, mask=mask), iters)
                for name, mask in SUBSETS.items()]
  return out


ROBUST_CAPS = [2500, 5000, 10000]
# scaling: body mass and limb length (geometry scaled about the CoM) with
# the spring designs above
SCALING = dict(motors=[dict(torque_scale=2, no_load_speed=20),
                       dict(torque_scale=6, no_load_speed=40)],
               caps=[2500, 5000], masses=[40, 50], limb_scales=[1.0, 1.25, 1.5],
               energies=[0, 2000, 4000])


def robust_chain(args):
  """Increasing energy budgets for one motor and force cap; each budget
  from two cold starts and a warm start from the previous budget."""
  motor, iters, energies = args
  rows, prev = [], None
  for energy in energies:
    t0 = time.time()
    design = R.Design(**motor)
    model = R.ReducedLaunch(design)
    search = None if energy == 0 else dict(shape=True, staged=True, energy=energy)
    tries = [model.optimize(iters, seed=s, spring_search=search) for s in (0, 1)]
    if prev is not None:
      warm = dict(search)
      if 'spring_unit' in prev:
        warm['init'] = prev['spring_unit']
      tries.append(model.optimize(iters, seed=2, init=prev['params'],
                                  spring_search=warm))
    best = max(tries, key=lambda r: r['speed_along'] - 10 * r['infeasible'])
    prev = best
    row = {k: v for k, v in best.items() if k != 'params'}
    rows.append({**asdict(design), 'budget_J': energy,
                 'variant': 'none' if energy == 0 else 'staged', **row,
                 'tries': [round(r['speed_along'], 2) for r in tries],
                 'params': best['params'].tolist(),
                 'seconds': round(time.time() - t0)})
  return rows


def run(args):
  motor, energy, name, opts, iters = args
  t0 = time.time()
  if opts is None:   # no springs, or the legacy springs
    design = R.Design(**motor, spring_energy=energy)
    out = R.ReducedLaunch(design).optimize(iters)
  else:
    design = R.Design(**motor)
    out = R.ReducedLaunch(design).optimize(
        iters, spring_search=dict(opts, energy=energy))
  out.pop('params')
  return {**asdict(design), 'budget_J': energy, 'variant': name, **out,
          'seconds': round(time.time() - t0)}


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--grid', default='main')
  p.add_argument('--iters', type=int, default=200)
  p.add_argument('--workers', type=int, default=4)
  p.add_argument('--out', default=None)
  args = p.parse_args()
  out = args.out or f'study_results/reduced_springs_{args.grid}.json'
  rows = []
  if args.grid == 'robust':
    tasks = [(dict(m, force_cap=c), args.iters, [0] + ENERGIES)
             for m, c in itertools.product(MOTORS, ROBUST_CAPS)]
    fn, flat = robust_chain, True
  elif args.grid == 'scaling':
    g = SCALING
    tasks = [(dict(m, force_cap=c, mass=ms, limb_scale=ls), args.iters,
              g['energies'])
             for m, c, ms, ls in itertools.product(
                 g['motors'], g['caps'], g['masses'], g['limb_scales'])]
    fn, flat = robust_chain, True
  else:
    tasks, fn, flat = jobs(args.grid, args.iters), run, False
  with Pool(args.workers) as pool:
    for result in pool.imap_unordered(fn, tasks):
     for row in (result if flat else [result]):
      rows.append(row)
      s = row.get('springs')
      print(f"x{row['torque_scale']:g} w0 {row['no_load_speed']:g} "
            f"cap {row['force_cap']:g} N mass {row['mass'] or 'model'} "
            f"limb x{row['limb_scale']:g} "
            f"{row['budget_J']:5.0f} J {row['variant']:8s}: "
            f"{row['speed_along']:5.2f} m/s @ {row['angle_deg']:.0f} deg, "
            f"t {row['takeoff_time']:.3f}, motor {row['work_J']:.0f} J, "
            f"spring {row['spring_work_J']:.0f}/{row['spring_J']:.0f} J, "
            f"KE {row['kinetic_energy_J']:.0f} J, "
            f"peak {row['peak_force_N']:.0f} N, err {row['energy_error']:+.3f}"
            + (f" | E {s['energy']} travel {s['travel']} n {s['exponent']} "
               f"release {s['release']}" if s else '')
            + (' INFEASIBLE' if row['infeasible'] else ''), flush=True)
      with open(out, 'w') as f:
        json.dump(rows, f, indent=1)


if __name__ == '__main__':
  main()
