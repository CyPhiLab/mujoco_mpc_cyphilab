#!/usr/bin/env python3
"""Design sweeps with the reduced-order launch model (reduced_launch.py).

Grids over torque scale, motor no-load speed and spring energy (and
optionally mass and limb scale), each design with its own optimized push,
run in parallel. Prints and saves takeoff speed along the launch direction.

Usage:
  reduced_sweep.py actuators [--out results.json] [--workers 4]
  reduced_sweep.py validate      (ideal motors, to compare with MuJoCo)
  reduced_sweep.py scaling       (mass and limb length)
"""
import argparse
import itertools
import json
import time
from dataclasses import asdict
from multiprocessing import Pool

import reduced_launch as R


def grid(name):
  if name == 'validate':
    return [R.Design(torque_scale=t) for t in [1, 2, 3, 4, 6, 8]]
  if name == 'actuators':
    return [R.Design(torque_scale=t, no_load_speed=w, spring_energy=e)
            for t, w, e in itertools.product([1, 2, 3, 4, 6],
                                             [10, 20, 40, 0],
                                             [0, 250, 500, 1000, 2000])]
  if name == 'scaling':
    return [R.Design(torque_scale=t, no_load_speed=20, spring_energy=e,
                     mass=m, limb_scale=s)
            for t, e, m, s in itertools.product([2, 4], [0, 500],
                                                [30, 40, 50],
                                                [1.0, 1.25, 1.5])]
  raise ValueError(name)


def run(args):
  design, angle, iters = args
  t0 = time.time()
  model = R.ReducedLaunch(design, angle)
  out = model.optimize(iters)
  out.pop('params')
  return {**asdict(design), **out, 'mass_kg': model.mass,
          'seconds': round(time.time() - t0)}


def run_chain(args):
  """Designs in increasing torque; each also warm-started from the
    previous design's solution (same torques), keeping the better result."""
  designs, angle, iters = args
  rows, prev, prev_params = [], None, None
  for design in designs:
    t0 = time.time()
    model = R.ReducedLaunch(design, angle)
    best = model.optimize(iters)
    if prev_params is not None:
      warm = model.optimize(iters, init=model.rescale(prev_params, prev))
      if warm['speed_along'] > best['speed_along']:
        best = warm
    prev, prev_params = design, best.pop('params')
    rows.append({**asdict(design), **best, 'mass_kg': model.mass,
                 'seconds': round(time.time() - t0)})
  return rows


def main():
  p = argparse.ArgumentParser()
  p.add_argument('grid')
  p.add_argument('--angle', type=float, default=30.0)
  p.add_argument('--iters', type=int, default=80)
  p.add_argument('--workers', type=int, default=4)
  p.add_argument('--out', default=None)
  args = p.parse_args()
  designs = grid(args.grid)
  # chains over torque for each other design setting
  chains = {}
  for d in designs:
    key = (d.no_load_speed, d.spring_energy, d.mass, d.limb_scale)
    chains.setdefault(key, []).append(d)
  jobs = [(sorted(c, key=lambda d: d.torque_scale), args.angle, args.iters)
          for c in chains.values()]
  rows = []
  with Pool(args.workers) as pool:
    for chain_rows in pool.imap_unordered(run_chain, jobs):
     for row in chain_rows:
      rows.append(row)
      print(f"torque x{row['torque_scale']:<3g} w0 {row['no_load_speed'] or 'inf':>4} "
            f"spring {row['spring_energy']:5.0f} J mass {row['mass_kg']:5.1f} "
            f"limb x{row['limb_scale']:<4g}: {row['speed_along']:5.2f} m/s along "
            f"({row['speed']:.2f} m/s @ {row['angle_deg']:.0f} deg, "
            f"t {row['takeoff_time']:.2f} s, work {row['work_J']:.0f} J, "
            f"KE {row['kinetic_energy_J']:.0f} J, {'INFEASIBLE' if row['infeasible'] else 'ok'})",
            flush=True)
      if args.out:
        with open(args.out, 'w') as f:
          json.dump(rows, f, indent=1)


if __name__ == '__main__':
  main()
