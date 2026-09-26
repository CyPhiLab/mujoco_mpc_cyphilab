#!/usr/bin/env python3
"""Derivative-free refinement of a launch with launch_ilqr.py's cost.

iLQR cannot optimize the contact force cap: contact forces jump at impact,
so finite-difference derivatives are meaningless there. This refines a
solution (e.g. an iLQR result) with the same cost by sampling: smooth
control perturbations (knots every KNOT_DT, linearly interpolated) around
the current controls, evaluated in parallel, updated from the elites
(cross-entropy method).

Usage:
  launch_refine.py controls.npy --torque 6 --spring_energy 500 \
      --force_cap 1500 [--hand_radius 0.04 --vault_time 0.08] \
      [--iters 200] [--out refined.npz]
"""
import argparse
import os
import time
from multiprocessing import Pool

import numpy as np

import launch_ilqr as L
import launch_study as S

KNOT_DT = 0.025

_solver = None


def _init(kwargs):
  global _solver
  _solver = L.LaunchILQR(**kwargs)


def _cost(U):
  return _solver.rollout(U)[2]


def perturbation(rng, n, nstep, sigma, dt):
  knots = int(round(nstep * dt / KNOT_DT)) + 1
  t_knot = np.linspace(0, 1, knots)
  t = np.linspace(0, 1, nstep)
  noise = sigma * rng.standard_normal((n, knots, 6))
  return np.stack([np.stack([np.interp(t, t_knot, noise[i, :, c])
                             for c in range(6)], axis=1) for i in range(n)])


def refine(kwargs, U, iters=200, pop=64, elite=8, sigma0=0.03,
           sigma_min=0.01, workers=4, seed=0, verbose=True):
  rng = np.random.default_rng(seed)
  solver = L.LaunchILQR(**kwargs)
  U = np.clip(U[:solver.N], -1, 1)
  best_cost = solver.rollout(U)[2]
  best = U.copy()
  sigma = sigma0
  with Pool(workers, initializer=_init, initargs=(kwargs,)) as pool:
    for it in range(iters):
      samples = np.clip(U + perturbation(rng, pop, solver.N, sigma, solver.dt),
                        -1, 1)
      samples[0] = U
      costs = np.array(pool.map(_cost, list(samples)))
      order = np.argsort(costs)
      if costs[order[0]] < best_cost:
        best_cost, best = costs[order[0]], samples[order[0]].copy()
      U = np.clip(samples[order[:elite]].mean(axis=0), -1, 1)
      sigma = max(sigma_min, sigma * 0.985)
      if verbose and (it % 10 == 0 or it == iters - 1):
        print(f'  iter {it:3d} best cost {best_cost:.3f} (pop best '
              f'{costs[order[0]]:.3f}, sigma {sigma:.3f})', flush=True)
  return best, best_cost, solver


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('controls')
  parser.add_argument('--push_time', type=float, default=0.45)
  parser.add_argument('--torque', type=float, default=6.0)
  parser.add_argument('--speed', type=float, default=10.0)
  parser.add_argument('--angle', type=float, default=30.0)
  parser.add_argument('--spring_energy', type=float, default=0.0)
  parser.add_argument('--force_cap', type=float, default=1500.0)
  parser.add_argument('--hand_radius', type=float, default=0)
  parser.add_argument('--vault_time', type=float, default=0.0)
  parser.add_argument('--joint_margin', type=float, default=None)
  parser.add_argument('--iters', type=int, default=200)
  parser.add_argument('--workers', type=int, default=4)
  parser.add_argument('--out', default='refined.npz')
  args = parser.parse_args()

  kwargs = dict(push_time=args.push_time, torque=args.torque,
                speed=args.speed, angle=args.angle,
                spring_energy=args.spring_energy, force_cap=args.force_cap,
                vault_time=args.vault_time)
  if args.hand_radius > 0:
    kwargs['hand'] = dict(S.PAD, radius=args.hand_radius)
  if args.joint_margin is not None:
    kwargs['joint_margin'] = args.joint_margin

  U0 = np.load(args.controls)
  solver = L.LaunchILQR(**kwargs)
  print('start:', S.launch_metrics(solver, U0[:solver.N]),
        solver.report(U0[:solver.N])[1]['speed'], flush=True)
  t0 = time.time()
  U, cost, solver = refine(kwargs, U0, iters=args.iters, workers=args.workers)
  half, det = solver.report(U)
  metrics = S.launch_metrics(solver, U)
  print(f'done in {time.time() - t0:.0f}s: cost {cost:.3f}', flush=True)
  print({k: det[k] for k in ('speed', 'angle_deg', 'takeoff_time',
                             'spin_per_mass', 'max_pitch_deg')}, flush=True)
  print(metrics, flush=True)
  solver.opt.record(half, args.out)
  np.save(os.path.splitext(args.out)[0] + '_controls.npy', half)
  print('wrote', args.out)


if __name__ == '__main__':
  main()
