#!/usr/bin/env python3
"""Continuation chains of periodic gaits (the pattern of the pterosaur's
launch_track.py).

  1. search   random first-harmonic strokes (gait_opt.seeds), the best few
              grown by a few Levenberg-Marquardt iterations each, the best of
              those through the elitist ES and LM, all at the first (coarse)
              timestep
  2. speeds   one LM solve per target speed, each warm-started from the
              previous speed's gait (period included)
  3. max      optional: a max-speed solve from the fastest gait
  4. dts      each gait re-polished at the finer timesteps (--dts), ending at
              the 1 ms MJPC simulation timestep; the analogue of the
              pterosaur's contact-margin continuation
  5. check    each final gait is also rolled out at the MJPC planner's 20 ms
              timestep, to see what the planner's own model predicts for it

Every step writes stepK_<name>.npz (theta, metrics, limit cycle samples),
stepK_<name>_controls.npy (one cycle of servo commands at the step's dt) and,
with MUJOCO_GL set, a video and plots; the run writes args.json and
summary.json.

Usage:
  gait_chain.py --speeds 0.04 0.06 0.08 0.10 0.12 --max_speed --out runs/A
  gait_chain.py --init runs/A/step3_v0.08_dt0.005.npz --speeds 0.08 ...
"""
import argparse
import json
import os
import time

import numpy as np

import crab_model as C
import gait_opt as G


def parse_weights(text):
  out = {}
  for item in filter(None, (text or '').split(',')):
    key, value = item.split('=')
    out[key.strip()] = float(value)
  return out


class Chain:

  def __init__(self, args):
    self.args = args
    self.out = args.out
    os.makedirs(self.out, exist_ok=True)
    self.summary_path = os.path.join(self.out, 'summary.json')
    self.summary = []
    if os.path.exists(self.summary_path):     # resume: reuse finished steps
      with open(self.summary_path) as f:
        self.summary = json.load(f)
    self.step = 0
    self.weights = parse_weights(args.weights)
    with open(os.path.join(self.out, 'args.json'), 'w') as f:
      json.dump(vars(args), f, indent=2)

  def log(self, text):
    print(text, flush=True)
    with open(os.path.join(self.out, 'log.txt'), 'a') as f:
      f.write(text + '\n')

  def problem(self, dt, speed, max_speed=False):
    weights = dict(self.weights)
    if max_speed:
      weights.setdefault('effort', self.args.max_speed_effort)
    return G.Problem(dt=dt, k=self.args.k, warm=self.args.warm,
                     measure=self.args.measure, speed=speed,
                     max_speed=max_speed, weights=weights)

  def feasible(self, metrics, speed, max_speed):
    a = self.args
    return bool((max_speed or abs(metrics['speed'] - speed) <= a.speed_tol)
                and abs(metrics['lateral_rate']) <= a.drift_tol
                and metrics['saturated'] <= a.saturation_tol)

  def save(self, name, problem, theta, history, stage, seconds):
    self.step += 1
    tag = f'step{self.step}_{name}'
    res, info = problem.evaluate(theta[None])
    metrics = info[0]
    cycle = problem.limit_cycle(theta)
    n = int(round(problem.layout.period(theta) / problem.dt))
    ctrl, _ = G.controls(problem.layout, theta, problem.half, n, problem.dt)
    path = os.path.join(self.out, tag + '.npz')
    np.savez(path, theta=theta, k=problem.layout.k, dt=problem.dt,
             speed_target=problem.speed, max_speed=problem.max_speed,
             warm=problem.warm, measure=problem.measure,
             metrics=json.dumps(metrics), joints=np.array(C.JOINTS),
             actuators=np.array(C.ACTUATORS), residual=res[0],
             **{key: cycle[key] for key in
                ('phase', 'time', 'qpos', 'qvel', 'act', 'ctrl', 'state')})
    np.save(os.path.join(self.out, tag + '_controls.npy'), ctrl)
    with open(os.path.join(self.out, tag + '_history.json'), 'w') as f:
      json.dump(history, f)
    row = dict(step=self.step, name=name, stage=stage, dt=problem.dt,
               speed_target=None if problem.max_speed else problem.speed,
               feasible=self.feasible(metrics, problem.speed,
                                      problem.max_speed),
               symmetry_index=G.symmetry_index(problem.layout, theta),
               seconds=round(seconds, 1), file=tag + '.npz', **metrics)
    self.summary.append(row)
    with open(self.summary_path, 'w') as f:
      json.dump(self.summary, f, indent=2)
    self.log(f'[{tag}] v {metrics["speed"]:.4f} T {metrics["period"]:.3f} '
             f'lat {metrics["lateral_rate"]:+.4f} yaw {metrics["yaw_rate"]:+.3f}'
             f' eff {metrics["effort"]:.3f} cot {metrics["cot"]:.2f} '
             f'cyc {metrics["cycle_error"]:.3f} feasible {row["feasible"]}')
    if os.environ.get('MUJOCO_GL') and not self.args.no_render:
      import render_gait
      try:
        render_gait.render(os.path.join(self.out, tag + '.mp4'), theta,
                           problem.layout.k, C.SIM_DT, cycles=6)
        render_gait.plot(os.path.join(self.out, tag + '_plots.png'), path)
      except Exception as e:  # rendering is a convenience, not the result
        self.log(f'  render failed: {e}')
    return row

  def solve(self, problem, theta, iters, es_iters=0):
    t0 = time.time()
    history = []
    with G.Evaluator(problem, self.args.processes) as ev:
      if es_iters:
        theta, _, h = G.es(ev, problem.layout, theta, iters=es_iters,
                           pop=self.args.pop, sigma=self.args.sigma,
                           seed=self.args.seed, log=self.log)
        history += h
      theta, _, h = G.lm(ev, problem.layout, theta, iters=iters, log=self.log)
      history += h
    return theta, history, time.time() - t0

  def search(self, speed, max_speed):
    a = self.args
    problem = self.problem(a.dts[0], speed, max_speed)
    t0 = time.time()
    seeds = G.seeds(problem.layout, n=a.seeds, seed=a.seed)
    self.log(f'search: {len(seeds)} seeds at dt {a.dts[0]}')
    with G.Evaluator(problem, a.processes) as ev:
      res, info = ev(seeds)
      cost = 0.5 * np.sum(res ** 2, axis=1)
      order = np.argsort(cost)[:a.starts]
      best, best_cost, history = None, np.inf, []
      for rank, i in enumerate(order):
        self.log(f' start {rank}: seed {i} cost {cost[i]:.2f} '
                 f'v {info[i]["speed"]:+.4f} T {info[i]["period"]:.3f}')
        theta, c, h = G.lm(ev, problem.layout, seeds[i],
                           iters=a.search_iters, log=self.log)
        history.append(h)
        if c < best_cost:
          best, best_cost = theta, c
      self.log(f' polishing the best start (cost {best_cost:.2f})')
      if a.es_iters:
        best, _, h = G.es(ev, problem.layout, best, iters=a.es_iters,
                          pop=a.pop, sigma=a.sigma, seed=a.seed, log=self.log)
        history.append(h)
      best, _, h = G.lm(ev, problem.layout, best, iters=a.lm_iters,
                        log=self.log)
      history.append(h)
    return problem, best, history, time.time() - t0

  def run_step(self, name, stage, compute):
    """Runs compute() -> (problem, theta, history, seconds) and saves it as the
    next step, or, when a previous run of this chain already wrote that step,
    loads it instead. Returns (theta, summary row)."""
    tag = f'step{self.step + 1}_{name}'
    for row in self.summary:
      if row['file'] == tag + '.npz' and os.path.exists(
          os.path.join(self.out, row['file'])):
        self.step += 1
        theta = np.load(os.path.join(self.out, row['file']))['theta']
        self.log(f'[{tag}] reused from an earlier run')
        return theta, row
    problem, theta, history, seconds = compute()
    return theta, self.save(name, problem, theta, history, stage, seconds)

  def run(self):
    a = self.args
    dt0 = a.dts[0]
    speeds = list(a.speeds)
    finals = []   # (name, theta, speed, max_speed) at dt0

    if a.init:
      z = np.load(a.init, allow_pickle=True)
      theta = G.Layout(int(z['k'])).with_k(z['theta'], a.k)
      self.log(f'init from {a.init}')
    else:
      first = speeds[0] if speeds else 0.0
      name = (f'v{first:g}' if speeds else 'max') + f'_dt{dt0:g}'
      theta, _ = self.run_step(name + '_search', 'search',
                           lambda: self.search(first, not speeds))

    def solver(problem, theta, iters, es_iters=0):
      return lambda: (problem,) + self.solve(problem, theta, iters, es_iters)

    for speed in speeds:
      problem = self.problem(dt0, speed)
      theta, row = self.run_step(f'v{speed:g}_dt{dt0:g}', 'speed',
                             solver(problem, theta, a.lm_iters))
      if not row['feasible']:
        self.log(f' v {speed:g} not met, retrying with the ES first')
        theta, row = self.run_step(f'v{speed:g}_dt{dt0:g}_es', 'speed',
                               solver(problem, theta, a.lm_iters, a.es_iters))
        if not row['feasible']:
          self.log(f' v {speed:g} infeasible; ending the speed chain')
          break
      finals.append((f'v{speed:g}', theta.copy(), speed, False))

    if a.max_speed:
      start = max(finals, key=lambda f: f[2])[1] if finals else theta
      problem = self.problem(dt0, 0.0, max_speed=True)
      theta_max, _ = self.run_step(f'max_dt{dt0:g}', 'max',
                               solver(problem, start, a.lm_iters, a.es_iters))
      finals.append(('max', theta_max, 0.0, True))

    for name, theta, speed, max_speed in finals:
      row = None
      for dt in a.dts[1:]:
        problem = self.problem(dt, speed, max_speed)
        theta, row = self.run_step(f'{name}_dt{dt:g}', 'dt',
                               solver(problem, theta, a.refine_iters))
      if row is None or 'plan_dt_check' in row:
        continue
      check = self.problem(C.PLAN_DT, speed, max_speed)
      _, info = check.evaluate(theta[None])
      row['plan_dt_check'] = info[0]
      with open(self.summary_path, 'w') as f:
        json.dump(self.summary, f, indent=2)
      self.log(f' {name}: at the planner dt {C.PLAN_DT} it swims '
               f'{info[0]["speed"]:+.4f} m/s (sim {row["speed"]:.4f})')


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawTextHelpFormatter)
  p.add_argument('--speeds', type=float, nargs='*', default=[0.06])
  p.add_argument('--max_speed', action='store_true')
  p.add_argument('--dts', type=float, nargs='+', default=[0.005, 0.002, 0.001])
  p.add_argument('--k', type=int, default=G.K_DEFAULT)
  p.add_argument('--warm', type=int, default=3)
  p.add_argument('--measure', type=int, default=2)
  p.add_argument('--weights', default='',
                 help='residual weight overrides, e.g. effort=0.5,lateral=3')
  p.add_argument('--max_speed_effort', type=float, default=0.05)
  p.add_argument('--init', help='warm start from a step npz (skips search)')
  p.add_argument('--seeds', type=int, default=256)
  p.add_argument('--starts', type=int, default=4)
  p.add_argument('--search_iters', type=int, default=8,
                 help='LM iterations per search start')
  p.add_argument('--es_iters', type=int, default=20,
                 help='elitist ES iterations (search polish, retries)')
  p.add_argument('--pop', type=int, default=48)
  p.add_argument('--sigma', type=float, default=0.05)
  p.add_argument('--lm_iters', type=int, default=20)
  p.add_argument('--refine_iters', type=int, default=8)
  p.add_argument('--speed_tol', type=float, default=0.003)
  p.add_argument('--drift_tol', type=float, default=0.005)
  p.add_argument('--saturation_tol', type=float, default=0.05)
  p.add_argument('--processes', type=int, default=None)
  p.add_argument('--seed', type=int, default=0)
  p.add_argument('--no_render', action='store_true')
  p.add_argument('--out', required=True)
  Chain(p.parse_args()).run()


if __name__ == '__main__':
  main()
