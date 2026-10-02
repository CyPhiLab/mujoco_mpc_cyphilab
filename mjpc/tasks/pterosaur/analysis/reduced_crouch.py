#!/usr/bin/env python3
"""Is the reference's crouch holding the launch back?

For each design, the reduced-order model's push is optimized from the
reference's deepest crouch, and again with the crouch (CoM height, hand and
foot placement, body pitch) optimized as well, from several random seeds.
The best searched crouch is then re-polished with the crouch fixed.

Usage:
  reduced_crouch.py [--out study_results/reduced_crouch.json] [--workers 4]
"""
import argparse
import itertools
import json
import time
from dataclasses import asdict
from multiprocessing import Pool

import reduced_launch as R

DESIGNS = [dict(torque_scale=2), dict(torque_scale=4),
           dict(torque_scale=4, no_load_speed=20),
           dict(torque_scale=6, no_load_speed=40),
           dict(torque_scale=6, no_load_speed=40, spring_energy=500)]
MARGINS = [0.3, 0.6]
SEEDS = [0, 1, 2]


def run(args):
  design, search, seed, iters = args
  t0 = time.time()
  model = R.ReducedLaunch(design)
  out = model.optimize(iters, seed=seed, search_crouch=search)
  if search:
    # polish the push with the crouch fixed, keeping the better result
    params = out['params']
    crouch = [out['crouch'][k] for k in R.CROUCH_NAMES]
    polished = model.optimize(iters // 2, seed=seed, init=params, crouch=crouch)
    if polished['speed_along'] > out['speed_along']:
      out = polished
  out.pop('params')
  return {**asdict(design), 'search_crouch': search, 'seed': seed, **out,
          'seconds': round(time.time() - t0)}


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--iters', type=int, default=200)
  p.add_argument('--workers', type=int, default=4)
  p.add_argument('--out', default='study_results/reduced_crouch.json')
  args = p.parse_args()
  jobs = []
  for d, m in itertools.product(DESIGNS, MARGINS):
    design = R.Design(**d, joint_margin=m)
    jobs.append((design, False, 0, args.iters))
    jobs += [(design, True, s, args.iters) for s in SEEDS]
  rows = []
  with Pool(args.workers) as pool:
    for row in pool.imap_unordered(run, jobs):
      rows.append(row)
      c = row['crouch']
      print(f"x{row['torque_scale']:g} w0 {row['no_load_speed'] or 'inf'} "
            f"spring {row['spring_energy']:g} margin {row['joint_margin']} "
            f"{'search' if row['search_crouch'] else 'ref   '} s{row['seed']}: "
            f"{row['speed_along']:5.2f} m/s @ {row['angle_deg']:.0f} deg "
            f"t {row['takeoff_time']:.3f} lift {row['liftoff_hind_fore']} | "
            f"h {c['height']:.2f} hand {c['hand_x']:.2f} foot {c['foot_x']:.2f} "
            f"pitch {c['pitch']:.2f} {'INFEASIBLE' if row['infeasible'] else ''}",
            flush=True)
      with open(args.out, 'w') as f:
        json.dump(rows, f, indent=1)


if __name__ == '__main__':
  main()
