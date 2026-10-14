#!/usr/bin/env python3
"""Continuation chains of periodic gaits (the pattern of the pterosaur's
launch_track.py).

  1. search   random first-harmonic strokes (gait_opt.seeds) scored against
              the first target speed, the best few grown by a few
              Levenberg-Marquardt iterations each, the best of those through
              the elitist ES and LM, all at the first timestep
  2. speeds   a ladder of target speeds (--speeds, or --speed_start times
              --speed_growth per rung up to --speed_max), one LM solve per
              rung warm-started from the previous rung's gait (period
              included); a missed rung is retried (first through the ES)
              while each retry gains --min_gain; the ladder ends at a rung it
              cannot reach, and that last attempt, if it is faster and still
              straight, is kept as "max"
  3. dts      the fastest gait (or every rung, --refine_all) re-polished at
              the finer timesteps (--dts), ending at the 1 ms MJPC simulation
              timestep; the analogue of the pterosaur's contact-margin
              continuation
  4. check    each final gait is also rolled out at the MJPC planner's 20 ms
              timestep, to see what the planner's own model predicts for it

Target-speed residuals suit Gauss-Newton; a constant "go faster" pull does
not (LM's damping climbs and progress stalls), hence the ladder.

The search starts from rest and scores cycles warm+1..warm+measure. Every
later step starts from the previous gait's steady limit-cycle state, so
--warm_steady cycles suffice and the scored speed is the steady-state speed
(--from_rest starts every step from rest instead: slower, but it catches
gaits that settle into a different, e.g. turning, cycle from rest).

Every step writes stepK_<name>.npz (theta, metrics, limit cycle samples),
stepK_<name>_controls.npy (one cycle of servo commands at the step's dt) and,
with MUJOCO_GL set, a video and plots; the run writes args.json and
summary.json.

Usage:
  gait_chain.py --direction 0 -1 0 --out runs/max_mY        # speed ladder
  gait_chain.py --speeds 0.04 0.06 0.08 --refine_all --out runs/family
  gait_chain.py --init runs/X/step3_v0.1_dt0.002.npz --speeds 0.12 ...

Rerunning a command resumes it: finished steps are reloaded, not redone.
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
    self.x0 = None          # None: start from rest; else a steady state
    model = C.load_model(ground=False, ctrl_limit=args.ctrl_limit,
                         design=parse_weights(args.design))
    self.state_layout = C.StateLayout(model)
    self.rest = C.initial_state(model)
    self.weights = parse_weights(args.weights)
    self.design = parse_weights(args.design)
    with open(os.path.join(self.out, 'args.json'), 'w') as f:
      json.dump(vars(args), f, indent=2)

  def log(self, text):
    print(text, flush=True)
    with open(os.path.join(self.out, 'log.txt'), 'a') as f:
      f.write(text + '\n')

  def problem(self, dt, speed, x0=None):
    a = self.args
    return G.Problem(dt=dt, k=a.k, warm=a.warm if x0 is None else a.warm_steady,
                     measure=a.measure, speed=speed, weights=self.weights,
                     direction=a.direction, x0=x0, ctrl_limit=a.ctrl_limit,
                     design=self.design, tau=a.tau)

  def straight(self, metrics):
    a = self.args
    return bool(abs(metrics['lateral_rate']) <= a.drift_tol
                and metrics['saturated'] <= a.saturation_tol)

  def feasible(self, metrics, speed):
    a = self.args
    tol = max(a.speed_tol, a.speed_rtol * speed)
    return bool(abs(metrics['speed'] - speed) <= tol and self.straight(metrics))

  def steady_state(self, path):
    """The limit-cycle state at phase 0 of a saved step, moved back to the
    start position, as the next problem's start state."""
    L = self.state_layout
    state = np.load(path)['state'].copy()
    state[L.time] = 0.0
    state[L.qpos][:3] = self.rest[L.qpos][:3]
    return state

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
             direction=problem.direction, x0=problem.x0, half=problem.half,
             design=json.dumps(problem.design), tau=problem.tau,
             warm=problem.warm, measure=problem.measure,
             metrics=json.dumps(metrics), joints=np.array(C.JOINTS),
             actuators=np.array(C.ACTUATORS), residual=res[0],
             **{key: cycle[key] for key in
                ('phase', 'time', 'qpos', 'qvel', 'act', 'ctrl', 'state')})
    np.save(os.path.join(self.out, tag + '_controls.npy'), ctrl)
    with open(os.path.join(self.out, tag + '_history.json'), 'w') as f:
      json.dump(history, f)
    row = dict(step=self.step, name=name, stage=stage, dt=problem.dt,
               speed_target=problem.speed,
               feasible=self.feasible(metrics, problem.speed),
               straight=self.straight(metrics),
               from_rest=problem.warm == self.args.warm,
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
                           problem.layout.k, C.SIM_DT, cycles=6,
                           half=problem.half, design=problem.design,
                           tau=problem.tau)
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

  def search(self, speed):
    a = self.args
    problem = self.problem(a.dts[0], speed)
    t0 = time.time()
    families = {
        'random': lambda n: G.seeds(problem.layout, n=n, seed=a.seed),
        'wide': lambda n: G.seeds(problem.layout, n=n, seed=a.seed + 1,
                                  offset=0.8),
        'crab': lambda n: G.crab_seeds(problem.layout, problem.half, n=n,
                                       direction=a.direction, seed=a.seed)}
    styles = a.seed_style.split('+')
    seeds = np.concatenate([families[style](a.seeds // len(styles))
                            for style in styles])
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
        if not self.args.from_rest:
          self.x0 = self.steady_state(os.path.join(self.out, row['file']))
        return theta, row
    problem, theta, history, seconds = compute()
    row = self.save(name, problem, theta, history, stage, seconds)
    if not self.args.from_rest:
      self.x0 = self.steady_state(os.path.join(self.out, row['file']))
    return theta, row

  def ladder(self):
    a = self.args
    if a.speeds:
      return list(a.speeds)
    speeds, v = [], a.speed_start
    while v <= a.speed_max + 1e-9:
      speeds.append(round(v, 4))
      v *= a.speed_growth
    return speeds

  def run(self):
    a = self.args
    dt0 = a.dts[0]
    speeds = self.ladder()
    rungs = []    # (name, theta, speed, x0) of every rung met, at dt0
    best_effort = None

    def solver(problem, theta, iters, es_iters=0):
      return lambda: (problem,) + self.solve(problem, theta, iters, es_iters)

    if a.init:
      z = np.load(a.init, allow_pickle=True)
      theta = G.Layout(int(z['k'])).with_k(z['theta'], a.k)
      # Commands are stored normalized by the command half-ranges they were
      # optimized with; re-express them for this run's ranges.
      old = z['half'] if 'half' in z else \
          C.load_model(ground=False).actuator_ctrlrange[:, 1]
      new = self.problem(dt0, speeds[0]).half
      theta = G.rescale_theta(G.Layout(a.k), theta, old, new)
      if not a.from_rest:
        self.x0 = self.steady_state(a.init)
      self.log(f'init from {a.init}')
    else:
      theta, _ = self.run_step(f'v{speeds[0]:g}_dt{dt0:g}_search', 'search',
                               lambda: self.search(speeds[0]))

    for speed in speeds:
      problem = self.problem(dt0, speed, self.x0)
      theta_v, row = self.run_step(f'v{speed:g}_dt{dt0:g}', 'speed',
                                   solver(problem, theta, a.lm_iters))
      # A missed rung is retried from where it got to (the first retry
      # through the ES) for as long as each attempt still gains speed.
      for attempt in range(1, a.patience + 1):
        if row['feasible']:
          break
        reached = row['speed']
        self.log(f' v {speed:g} not met (reached {reached:.4f}), retry '
                 f'{attempt}')
        problem = self.problem(dt0, speed, self.x0)
        theta_v, row = self.run_step(
            f'v{speed:g}_dt{dt0:g}_r{attempt}', 'speed',
            solver(problem, theta_v, a.lm_iters,
                   a.es_iters if attempt == 1 else 0))
        if row['speed'] < reached * (1 + a.min_gain):
          break
      if not row['feasible']:
        self.log(f' v {speed:g} missed (reached {row["speed"]:.4f}); '
                 'ending the ladder')
        fastest = rungs[-1][2] if rungs else 0.0
        if row['straight'] and row['speed'] > fastest:
          best_effort = ('max', theta_v, speed, self.x0)
        break
      theta = theta_v
      rungs.append((f'v{speed:g}', theta.copy(), speed, self.x0))

    finals = rungs if a.refine_all else rungs[-1:]
    if best_effort:
      finals = finals + [best_effort]
    for name, theta, speed, x0 in finals:
      self.x0 = x0
      row = None
      for dt in a.dts[1:]:
        problem = self.problem(dt, speed, self.x0)
        theta, row = self.run_step(f'{name}_dt{dt:g}', 'dt',
                                   solver(problem, theta, a.refine_iters))
      if row is None or 'plan_dt_check' in row:
        continue
      check = self.problem(C.PLAN_DT, speed, self.x0)
      _, info = check.evaluate(theta[None])
      row['plan_dt_check'] = info[0]
      with open(self.summary_path, 'w') as f:
        json.dump(self.summary, f, indent=2)
      self.log(f' {name}: at the planner dt {C.PLAN_DT} it swims '
               f'{info[0]["speed"]:+.4f} m/s (sim {row["speed"]:.4f})')


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawTextHelpFormatter)
  p.add_argument('--speeds', type=float, nargs='*', default=[],
                 help='explicit target speeds (default: the ladder below)')
  p.add_argument('--speed_start', type=float, default=0.08)
  p.add_argument('--speed_growth', type=float, default=1.25)
  p.add_argument('--speed_max', type=float, default=1.0)
  p.add_argument('--patience', type=int, default=3,
                 help='retries of a missed rung')
  p.add_argument('--min_gain', type=float, default=0.03,
                 help='relative speed gain a retry must make to go on')
  p.add_argument('--refine_all', action='store_true',
                 help='refine every rung met, not just the fastest')
  p.add_argument('--direction', type=float, nargs=3, default=[0, -1, 0],
                 help='swim direction (world); the body keeps its heading')
  p.add_argument('--dts', type=float, nargs='+', default=[0.002, 0.001],
                 help='timesteps: search and ladder at the first, then '
                      'refined at the rest (5 ms is too coarse for the kp 50 '
                      'servos: the same stroke moves ~20%% differently)')
  p.add_argument('--k', type=int, default=G.K_DEFAULT)
  p.add_argument('--design', default='',
                 help='design variant, e.g. paddle_torque=2,body_drag=0.5 '
                      '(crab_model.DESIGN_DEFAULTS)')
  p.add_argument('--tau', type=float, default=C.TAU,
                 help='servo setpoint filter time constant (s)')
  p.add_argument('--ctrl_limit', type=float, default=None,
                 help='servo command range +-this (rad), in place of the '
                      'task ctrlranges')
  p.add_argument('--seed_style', default='random',
                 help="search seed families, '+'-joined: random (first-"
                      "harmonic strokes about the zero pose), wide (the same "
                      "with offsets anywhere in +-0.8 of the range), crab "
                      "(raised sculling paddles, streamlined legs; "
                      "G.crab_seeds)")
  p.add_argument('--warm', type=int, default=4,
                 help='cycles before scoring, from rest (the search)')
  p.add_argument('--warm_steady', type=int, default=2,
                 help='cycles before scoring, from a steady state')
  p.add_argument('--from_rest', action='store_true',
                 help='start every rollout from rest (as the MJPC task '
                      'does), not from the previous gait\'s steady state; a '
                      'gait with a free steady tilt can settle differently '
                      'from rest')
  p.add_argument('--measure', type=int, default=2)
  p.add_argument('--weights', default='',
                 help='residual weight overrides, e.g. effort=0.5,lateral=3')
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
  p.add_argument('--speed_rtol', type=float, default=0.05)
  p.add_argument('--drift_tol', type=float, default=0.005)
  p.add_argument('--saturation_tol', type=float, default=0.2,
                 help='fraction of time a servo may sit at its torque limit')
  p.add_argument('--processes', type=int, default=None)
  p.add_argument('--seed', type=int, default=0)
  p.add_argument('--no_render', action='store_true')
  p.add_argument('--out', required=True)
  Chain(p.parse_args()).run()


if __name__ == '__main__':
  main()
