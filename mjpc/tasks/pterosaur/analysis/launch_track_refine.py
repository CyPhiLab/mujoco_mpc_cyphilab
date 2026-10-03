#!/usr/bin/env python3
"""Derivative-free refinement of a tracked launch (launch_track.py).

iLQR stalls once contacts start switching (its line search finds no
descent direction from finite-difference derivatives of a contact-rich
push). This refines a launch_track.py solution with the same cost by an elitist
evolution strategy with smooth knot perturbations (launch_refine.py's),
which needs no derivatives: samples are drawn around the best launch so far,
the best sample replaces it only if it is better, and the sampling width
grows after successes and shrinks after failures. (Averaging elite samples,
as the cross-entropy method does, fails here: the cost is so rough that the
average of good samples is usually a bad one.)

Usage:
  launch_track_refine.py tracked_C_bodypath/step1_T0.45_v7_controls.npy \
      --speed 8 --reference bodypath [--iters 150] [--out DIR]
"""
import argparse
import json
import os
import time
from multiprocessing import Pool

import numpy as np

import launch_refine as R
import launch_track as LT

_solver = None


def _init(kwargs):
  global _solver
  _solver = LT.TrackILQR(**kwargs)


def _cost(U):
  return _solver.rollout(U)[2]


def main():
  p = argparse.ArgumentParser()
  p.add_argument('controls')
  p.add_argument('--push_time', type=float, default=0.45)
  p.add_argument('--torque', type=float, default=6.0)
  p.add_argument('--spring_energy', type=float, default=500.0)
  p.add_argument('--speed', type=float, default=8.0)
  p.add_argument('--angle', type=float, default=30.0)
  p.add_argument('--reference', default='bodypath')
  p.add_argument('--iters', type=int, default=150)
  p.add_argument('--pop', type=int, default=64)
  p.add_argument('--sigma', type=float, default=0.02)
  p.add_argument('--workers', type=int, default=4)
  p.add_argument('--out', default=None)
  args = p.parse_args()
  out = args.out or os.path.join(os.path.dirname(args.controls),
                                 f'refined_v{args.speed:g}')
  os.makedirs(out, exist_ok=True)
  kwargs = dict(push_time=args.push_time, torque=args.torque, speed=args.speed,
                angle=args.angle, spring_energy=args.spring_energy,
                reference=args.reference)
  solver = LT.TrackILQR(**kwargs)
  best = np.clip(np.load(args.controls)[:solver.N], -1, 1)
  best_cost = solver.rollout(best)[2]
  print(f'start cost {best_cost:.3f}', flush=True)
  rng = np.random.default_rng(0)
  sigma = args.sigma
  history = []
  t0 = time.time()
  with Pool(args.workers, initializer=_init, initargs=(kwargs,)) as pool:
    for it in range(args.iters):
      samples = np.clip(best + R.perturbation(rng, args.pop, solver.N, sigma, solver.dt),
                        -1, 1)
      costs = np.array(pool.map(_cost, list(samples)))
      order = np.argsort(costs)
      if costs[order[0]] < best_cost:
        best_cost, best = costs[order[0]], samples[order[0]].copy()
        sigma = min(0.1, sigma * 1.3)
      else:
        sigma = max(0.002, sigma * 0.8)
      history.append(float(best_cost))
      if it % 10 == 0 or it == args.iters - 1:
        print(f'  iter {it:3d} best cost {best_cost:.3f} (pop best '
              f'{costs[order[0]]:.3f}, sigma {sigma:.3f}) [{time.time() - t0:.0f} s]',
              flush=True)
  half, det = solver.report(best)
  opt = solver.opt
  opt.debounce_steps = 5
  _, det = opt.evaluate(half[None], detail=True)
  det = det[0]
  strip = LT.contact_strip(opt, half)
  name = f'refined_T{args.push_time:g}_v{args.speed:g}'
  np.save(os.path.join(out, name + '_controls.npy'), half)
  traj = opt.record(half, os.path.join(out, name + '.npz'))
  if os.environ.get('MUJOCO_GL'):
    LT.render(opt, traj, det, f"{name}: {det['speed']:.1f} m/s @ {det['angle_deg']:.0f} deg",
              os.path.join(out, name))
  n = int(round(args.push_time / LT.T.SIM_DT))
  row = {'name': name, 'init': args.controls, 'cost': float(best_cost),
         'history': history, 'seconds': round(time.time() - t0),
         **{k: det[k] for k in ('took_off', 'speed', 'angle_deg', 'takeoff_time',
                                'spin_per_mass', 'max_pitch_deg', 'taps', 'calm_rad_s')},
         'hand_contact_frac': [strip[k][:n].count('#') / n for k in ('to_hand_l', 'to_hand_r')],
         'contacts': strip}
  json.dump(row, open(os.path.join(out, 'summary.json'), 'w'), indent=1)
  print(f"{name}: {det['speed']:.2f} m/s @ {det['angle_deg']:.0f} deg, takeoff "
        f"{det['takeoff_time']:.3f} s, hands {row['hand_contact_frac']}", flush=True)


if __name__ == '__main__':
  main()
