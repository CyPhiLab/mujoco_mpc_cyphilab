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

Usage:
  reduced_springs.py [--grid main|subsets] [--out FILE] [--workers 4]
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
VARIANTS = {
    'linear': dict(),
    'shaped': dict(shape=True),
    'staged': dict(shape=True, staged=True),
}
SUBSETS = {
    'middle': [0, 1, 0, 1],      # knees and elbows
    'proximal': [1, 0, 1, 0],    # hips and shoulders
    'hind': [1, 1, 0, 0],
    'fore': [0, 0, 1, 1],
}


def jobs(grid, iters):
  out = []
  for motor, e in itertools.product(MOTORS, ENERGIES):
    if grid == 'main':
      out.append((motor, e, 'legacy', None, iters))
      out += [(motor, e, name, opts, iters) for name, opts in VARIANTS.items()]
    elif grid == 'subsets':
      out += [(motor, e, name, dict(shape=True, staged=True, mask=mask), iters)
              for name, mask in SUBSETS.items()]
  return out


def run(args):
  motor, energy, name, opts, iters = args
  t0 = time.time()
  if opts is None:
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
  with Pool(args.workers) as pool:
    for row in pool.imap_unordered(run, jobs(args.grid, args.iters)):
      rows.append(row)
      s = row.get('springs')
      print(f"x{row['torque_scale']:g} w0 {row['no_load_speed']:g} "
            f"{row['budget_J']:5.0f} J {row['variant']:8s}: "
            f"{row['speed_along']:5.2f} m/s @ {row['angle_deg']:.0f} deg, "
            f"t {row['takeoff_time']:.3f}, motor {row['work_J']:.0f} J, "
            f"spring {row['spring_work_J']:.0f}/{row['spring_J']:.0f} J, "
            f"KE {row['kinetic_energy_J']:.0f} J"
            + (f" | E {s['energy']} travel {s['travel']} n {s['exponent']} "
               f"release {s['release']}" if s else '')
            + (' INFEASIBLE' if row['infeasible'] else ''), flush=True)
      with open(out, 'w') as f:
        json.dump(rows, f, indent=1)


if __name__ == '__main__':
  main()
