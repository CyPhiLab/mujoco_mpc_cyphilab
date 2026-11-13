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

With --reference REF.json --mode MODE (a kinematic_ref.py reference) the
chain tracks that gait's stroke instead of searching: the swim direction is
the mode's, the start is a Fourier fit of the reference angles as commands
(--period, led by the servo lag), and an "imitate" step first solves for
tracking alone (no speed term), which shows how fast the reference swims
with real physics; the ladder then starts above that speed and keeps the
tracking term (--weights track=W, default 1) as a regulariser on the
stroke's shape.

Turning primitives: --yaw_ladder W1 W2 ... runs the ladder over yaw-rate
targets (rad/s, + turns left / counterclockwise from above) instead of speed,
with the translation speed held at --turn_speed (default 0: turn on the spot);
rungs are named w<W>. --yaw_rate W holds a yaw target through a speed ladder
(arcs, and with --direction 0 0 +-1 helices; their speed and drift are
measured in the heading frame). Vertical primitives: --direction 0 0 1 (or -1).

Rotations about any body axis: --rate_axis x|y|z --rate_ladder W1 W2 ...
ladders the mean body-frame angular rate about that axis (the other two
held at 0, within --off_axis_tol); rungs are named w<axis><W>. Translation
is held at --turn_speed. Needs --from_rest (a rotated attitude cannot be
reset like a heading). Attitude-held swims: --attitude AX AY holds the body z
axis at its direction after rotating about world x by AX, then world y by AY
(rad), instead of level.

Seeds: --seed_style library adds the gaits in --seed_files (and their
left/right mirror images) to the search batch, e.g. the -Y and +X gaits when
searching a diagonal; splice adds every pairing of their right legs with
another's left legs (seeds for turning).

Every step writes stepK_<name>.npz (theta, metrics, limit cycle samples),
stepK_<name>_controls.npy (one cycle of servo commands at the step's dt) and,
with MUJOCO_GL set, a video and plots; the run writes args.json and
summary.json.

Usage:
  gait_chain.py --direction 0 -1 0 --out runs/max_mY        # speed ladder
  gait_chain.py --speeds 0.04 0.06 0.08 --refine_all --out runs/family
  gait_chain.py --init runs/X/step3_v0.1_dt0.002.npz --speeds 0.12 ...
  gait_chain.py --reference crab_reference.json --mode paddles_first --k 5 \
      --ctrl_limit 3.1416 --out runs/ref_paddles_first

Rerunning a command resumes it: finished steps are reloaded, not redone.
"""
import argparse
import json
import os
import time

import mujoco
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
    C.set_robot(args.robot)
    self.out = args.out
    os.makedirs(self.out, exist_ok=True)
    self.summary_path = os.path.join(self.out, 'summary.json')
    self.summary = []
    if os.path.exists(self.summary_path):     # resume: reuse finished steps
      with open(self.summary_path) as f:
        self.summary = json.load(f)
    self.step = 0
    self.x0 = None          # None: start from rest; else a steady state
    self.rotating = bool(args.rate_ladder)
    self.turning = bool(args.yaw_ladder) or self.rotating
    if args.rate_dir:              # any body axis, e.g. 1 -1 0
      self.axis_vec = np.asarray(args.rate_dir, float)
      self.axis_vec /= np.linalg.norm(self.axis_vec)
    else:
      self.axis_vec = np.eye(3)['xyz'.index(args.rate_axis)]
    if self.rotating and (args.yaw_ladder or not args.from_rest):
      raise SystemExit('--rate_ladder needs --from_rest and no --yaw_ladder')
    model = C.load_model(ground=False, ctrl_limit=args.ctrl_limit,
                         design=parse_weights(args.design))
    self.state_layout = C.StateLayout(model)
    self.rest = C.initial_state(model)
    self.start = None       # rest turned to --attitude (--start_attitude)
    if args.start_attitude:
      if args.attitude is None:
        raise SystemExit('--start_attitude needs --attitude')
      self.start = turned_state(self.rest, self.state_layout, args.attitude)
    self.weights = parse_weights(args.weights)
    self.design = parse_weights(args.design)
    self.reference = None
    if args.reference:
      self.reference = load_reference(args.reference, args.mode)
      args.direction = self.reference['direction']
    with open(os.path.join(self.out, 'args.json'), 'w') as f:
      json.dump(vars(args), f, indent=2)

  def log(self, text):
    print(text, flush=True)
    with open(os.path.join(self.out, 'log.txt'), 'a') as f:
      f.write(text + '\n')

  def problem(self, dt, speed, x0=None, weights=None, yaw=None):
    a = self.args
    yaw = a.yaw_rate if yaw is None else yaw
    body_rate = None
    if self.rotating:             # yaw carries the rate about the axis
      body_rate, yaw = self.axis_vec * yaw, 0.0
    warm = a.warm if x0 is None else a.warm_steady
    if x0 is None and self.start is not None:
      x0 = self.start
    return G.Problem(dt=dt, k=a.k, warm=warm,
                     measure=a.measure, speed=speed,
                     weights=dict(self.weights, **(weights or {})),
                     direction=a.direction, x0=x0, ctrl_limit=a.ctrl_limit,
                     design=self.design, tau=a.tau,
                     reference=None if self.reference is None
                     else self.reference['angles'],
                     fixed_period=a.fix_period, yaw_rate=yaw,
                     hold=held_actuators(a.hold_legs), body_rate=body_rate,
                     attitude=a.attitude,
                     heading_frame=bool(a.yaw_rate) and not self.turning)

  def rate(self, metrics):
    """The rate the ladder targets: body rate about the axis, or yaw."""
    if self.rotating:
      return float(np.dot(metrics['body_rate'], self.axis_vec))
    return float(metrics['yaw_rate'])

  def target_rate(self, problem):
    if self.rotating:
      return float(np.dot(problem.body_rate, self.axis_vec))
    return problem.yaw_rate

  def straight(self, metrics):
    a = self.args
    ok = (abs(metrics['lateral_rate']) <= a.drift_tol
          and metrics['saturated'] <= a.saturation_tol)
    if not self.turning:   # a straight gait also holds its heading
      ok = ok and abs(metrics['yaw_rate'] - a.yaw_rate) <= a.yaw_drift_tol
    if self.weights.get('attitude_rate') and not self.rotating:
      # ... and, when asked (--weights attitude_rate=W), its pitch and roll
      ok = ok and max(abs(w) for w in metrics['body_rate'][:2]) <= \
          a.yaw_drift_tol
    return bool(ok)

  def feasible(self, metrics, speed, yaw=0.0):
    a = self.args
    tol = max(a.speed_tol, a.speed_rtol * speed)
    if self.turning:   # translation held near its target
      ok = abs(metrics['speed'] - speed) <= tol and self.straight(metrics)
    else:              # a speed rung is met by swimming at least that fast
      ok = metrics['speed'] >= speed - tol and self.straight(metrics)
    if self.turning or yaw:
      ok = ok and abs(self.rate(metrics) - yaw) <= max(
          a.yaw_tol, a.speed_rtol * abs(yaw))
    if self.rotating:   # no rotation about the other two axes
      w = np.asarray(metrics['body_rate'])
      off = w - np.dot(w, self.axis_vec) * self.axis_vec
      ok = ok and bool(np.linalg.norm(off) <= a.off_axis_tol)
    return bool(ok)

  def steady_state(self, path):
    """The limit-cycle state at phase 0 of a saved step, moved back to the
    start position, as the next problem's start state."""
    L = self.state_layout
    state = np.load(path)['state'].copy()
    state[L.time] = 0.0
    state[L.qpos][:3] = self.rest[L.qpos][:3]
    if self.turning or self.args.yaw_rate:
      # A turning gait's cycle ends turned; take the yaw out so every step
      # starts at the rest heading (world linear velocity turned with it).
      remove_yaw(state, L)
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
             robot=problem.robot,
             speed_target=problem.speed, max_speed=problem.max_speed,
             direction=problem.direction, x0=problem.x0, half=problem.half,
             design=json.dumps(problem.design), tau=problem.tau,
             warm=problem.warm, measure=problem.measure,
             metrics=json.dumps(metrics), joints=np.array(C.JOINTS),
             actuators=np.array(C.ACTUATORS), residual=res[0],
             **({} if self.start is None else {'start': self.start}),
             **{key: cycle[key] for key in
                ('phase', 'time', 'qpos', 'qvel', 'act', 'ctrl', 'state')})
    np.save(os.path.join(self.out, tag + '_controls.npy'), ctrl)
    with open(os.path.join(self.out, tag + '_history.json'), 'w') as f:
      json.dump(history, f)
    row = dict(step=self.step, name=name, stage=stage, dt=problem.dt,
               speed_target=problem.speed,
               feasible=self.feasible(metrics, problem.speed,
                                      self.target_rate(problem)),
               straight=self.straight(metrics),
               from_rest=problem.warm == self.args.warm,
               symmetry_index=G.symmetry_index(problem.layout, theta),
               seconds=round(seconds, 1), file=tag + '.npz', **metrics)
    self.summary.append(row)
    with open(self.summary_path, 'w') as f:
      json.dump(self.summary, f, indent=2)
    self.log(f'[{tag}] v {metrics["speed"]:.4f} T {metrics["period"]:.3f} '
             f'lat {metrics["lateral_rate"]:+.4f} yaw {metrics["yaw_rate"]:+.3f}'
             + (f' w{"d" if self.args.rate_dir else self.args.rate_axis} {self.rate(metrics):+.3f}'
                if self.rotating else '') +
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

  def checkpoint(self, stage):
    """Checkpoint file of a stage of the step being computed (a rerun after
    a crash or restart continues from it; removed once the step is saved)."""
    return os.path.join(self.out, f'.ckpt_{self.tag}_{stage}.pkl')

  def solve(self, problem, theta, iters, es_iters=0):
    t0 = time.time()
    history = []
    with G.Evaluator(problem, self.args.processes) as ev:
      if es_iters:
        theta, _, h = G.es(ev, problem.layout, theta, iters=es_iters,
                           pop=self.args.pop, sigma=self.args.sigma,
                           seed=self.args.seed, log=self.log,
                           checkpoint=self.checkpoint('es'))
        history += h
      theta, _, h = G.lm(ev, problem.layout, theta, iters=iters, log=self.log,
                         checkpoint=self.checkpoint('lm'))
      history += h
    return problem.fix(theta), history, time.time() - t0

  def search(self, speed, yaw=None):
    a = self.args
    problem = self.problem(a.dts[0], speed, yaw=yaw)
    t0 = time.time()
    families = {
        'random': lambda n: G.seeds(problem.layout, n=n, seed=a.seed),
        'wide': lambda n: G.seeds(problem.layout, n=n, seed=a.seed + 1,
                                  offset=0.8),
        'crab': lambda n: G.crab_seeds(problem.layout, problem.half, n=n,
                                       direction=a.direction, seed=a.seed),
        'library': lambda n: library_seeds(a.seed_files, problem.layout,
                                           problem.half),
        'splice': lambda n: splice_seeds(library_seeds(
            a.seed_files, problem.layout, problem.half), problem.layout)}
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
                           iters=a.search_iters, log=self.log,
                           checkpoint=self.checkpoint(f'start{rank}'))
        history.append(h)
        if c < best_cost:
          best, best_cost = theta, c
      self.log(f' polishing the best start (cost {best_cost:.2f})')
      if a.es_iters:
        best, _, h = G.es(ev, problem.layout, best, iters=a.es_iters,
                          pop=a.pop, sigma=a.sigma, seed=a.seed, log=self.log,
                          checkpoint=self.checkpoint('es'))
        history.append(h)
      best, _, h = G.lm(ev, problem.layout, best, iters=a.lm_iters,
                        log=self.log, checkpoint=self.checkpoint('lm'))
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
    self.tag = tag
    problem, theta, history, seconds = compute()
    row = self.save(name, problem, theta, history, stage, seconds)
    for f in os.listdir(self.out):
      if f.startswith(f'.ckpt_{tag}_'):
        os.remove(os.path.join(self.out, f))
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

  def targets(self, speeds):
    """Rungs as (name, speed, yaw rate)."""
    a = self.args
    if self.rotating:
      axis = a.rate_axis if not a.rate_dir else 'd'
      return [(f'w{axis}{w:g}', a.turn_speed, w)
              for w in a.rate_ladder]
    if self.turning:
      return [(f'w{w:g}', a.turn_speed, w) for w in a.yaw_ladder]
    return [(f'v{v:g}', v, a.yaw_rate) for v in speeds]

  def run(self):
    a = self.args
    dt0 = a.dts[0]
    speeds = self.ladder()
    rungs = []    # (name, theta, speed, x0, yaw) of every rung met, at dt0
    best_effort = None

    def solver(problem, theta, iters, es_iters=0):
      return lambda: (problem,) + self.solve(problem, theta, iters, es_iters)

    def progress(row):
      """What the ladder climbs: speed, or yaw rate (in the target's
      sense) for a turning ladder."""
      if self.turning:
        first = (a.rate_ladder or a.yaw_ladder)[0]
        return float(np.sign(first) * self.rate(row))
      return row['speed']

    if self.reference is not None and not a.init:
      layout = G.Layout(a.k)
      half = self.problem(dt0, speeds[0]).half
      theta = G.fit_reference(layout, self.reference['angles'],
                              a.fix_period or a.period,
                              half, lead=a.tau + a.lead)
      problem = self.problem(dt0, 0.0, weights=dict(speed=0.0))
      theta, row = self.run_step(f'imitate_dt{dt0:g}', 'imitate',
                                 solver(problem, theta, a.imitate_iters))
      self.log(f'imitating {a.mode}: v {row["speed"]:.4f} T '
               f'{row["period"]:.3f} track rms {row["track_rms"]:.3f} rad')
      if not a.speeds:
        v = max(row['speed'], 0.01) * a.speed_growth
        speeds = []
        while v <= a.speed_max + 1e-9:
          speeds.append(round(v, 4))
          v *= a.speed_growth
    elif a.init:
      z = np.load(a.init, allow_pickle=True)
      theta = G.Layout(int(z['k'])).with_k(z['theta'], a.k)
      # Commands are stored normalized by the command half-ranges they were
      # optimized with; re-express them for this run's ranges.
      old = z['half'] if 'half' in z else \
          C.load_model(ground=False).actuator_ctrlrange[:, 1]
      new = self.problem(dt0, speeds[0]).half
      theta = G.rescale_theta(G.Layout(a.k), theta, old, new)
      if not a.from_rest and C.step_robot(z) == C.robot():
        self.x0 = self.steady_state(a.init)
      # (From another robot's gait the commands carry over, its state does
      # not: the first step then starts from rest.)
      self.log(f'init from {a.init} ({C.step_robot(z)} gait'
               f'{"" if C.step_robot(z) == C.robot() else ", from rest"})')
    else:
      name0, speed0, yaw0 = self.targets(speeds)[0]
      theta, _ = self.run_step(f'{name0}_dt{dt0:g}_search', 'search',
                               lambda: self.search(speed0, yaw0))

    for rung, speed, yaw in self.targets(speeds):
      problem = self.problem(dt0, speed, self.x0, yaw=yaw)
      theta_v, row = self.run_step(f'{rung}_dt{dt0:g}', 'speed',
                                   solver(problem, theta, a.lm_iters))
      # A missed rung is retried from where it got to (the first retry
      # through the ES) for as long as each attempt still gains speed.
      attempts = [(theta_v, row, self.x0)]
      for attempt in range(1, a.patience + 1):
        if row['feasible']:
          break
        reached = progress(row)
        self.log(f' {rung} not met (reached v {row["speed"]:.4f} yaw '
                 f'{self.rate(row):+.4f}), retry {attempt}')
        problem = self.problem(dt0, speed, self.x0, yaw=yaw)
        theta_v, row = self.run_step(
            f'{rung}_dt{dt0:g}_r{attempt}', 'speed',
            solver(problem, theta_v, a.lm_iters,
                   a.es_iters if attempt == 1 else 0))
        attempts.append((theta_v, row, self.x0))
        if progress(row) < reached * (1 + a.min_gain):
          break
      if not row['feasible']:
        self.log(f' {rung} missed (reached v {row["speed"]:.4f} yaw '
                 f'{self.rate(row):+.4f}); ending the ladder')
        # "max" is the best straight attempt at this rung (not the last).
        best = rungs[-1][5] if rungs else 0.0
        straight = [t for t in attempts if t[1]['straight']]
        if straight:
          th_b, row_b, x0_b = max(straight, key=lambda t: progress(t[1]))
          if progress(row_b) > best:
            best_effort = ('max', th_b, speed, x0_b, yaw, progress(row_b))
        break
      theta = theta_v
      rungs.append((rung, theta.copy(), speed, self.x0, yaw, progress(row)))

    finals = rungs if a.refine_all else rungs[-1:]
    if best_effort:
      finals = finals + [best_effort]
    for name, theta, speed, x0, yaw, _ in finals:
      self.x0 = x0
      row = None
      for dt in a.dts[1:]:
        problem = self.problem(dt, speed, self.x0, yaw=yaw)
        theta, row = self.run_step(f'{name}_dt{dt:g}', 'dt',
                                   solver(problem, theta, a.refine_iters))
      if row is None or 'plan_dt_check' in row:
        continue
      check = self.problem(C.PLAN_DT, speed, self.x0, yaw=yaw)
      _, info = check.evaluate(theta[None])
      row['plan_dt_check'] = info[0]
      with open(self.summary_path, 'w') as f:
        json.dump(self.summary, f, indent=2)
      self.log(f' {name}: at the planner dt {C.PLAN_DT} it swims '
               f'{info[0]["speed"]:+.4f} m/s (sim {row["speed"]:.4f})')


def held_actuators(legs):
  """Actuator indices of the legs named in 'R1,R2,L1,L2' (all three joints
  of each)."""
  out = []
  for leg in filter(None, (legs or '').split(',')):
    out += [i for i, name in enumerate(C.ACTUATORS)
            if name.startswith(leg.strip() + '_')]
  return out


def turned_state(state, L, attitude):
  """A copy of a rest state turned about world x by attitude[0], then world
  y by attitude[1] (rest is still: no velocities to turn)."""
  state = np.array(state, dtype=float)
  qpos = state[L.qpos]
  hx, hy = 0.5 * attitude[0], 0.5 * attitude[1]
  qx = np.array([np.cos(hx), np.sin(hx), 0.0, 0.0])
  qy = np.array([np.cos(hy), 0.0, np.sin(hy), 0.0])
  q = np.zeros(4)
  mujoco.mju_mulQuat(q, qy, qx)
  q2 = np.zeros(4)
  mujoco.mju_mulQuat(q2, q, qpos[3:7])
  qpos[3:7] = q2
  state[L.qpos] = qpos
  return state


def remove_yaw(state, L):
  """Turns a full-physics state about world z so the body's heading is the
  rest heading (zero yaw); free-joint world linear velocity turns with it,
  the body-frame angular velocity does not change."""
  qpos = state[L.qpos]
  yaw = float(C.yaw_of(C.quat_to_mat(qpos[3:7])))
  half = -0.5 * yaw
  qz = np.array([np.cos(half), 0.0, 0.0, np.sin(half)])
  w1, x1, y1, z1 = qz
  w2, x2, y2, z2 = qpos[3:7]
  qpos[3:7] = [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
               w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
               w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
               w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]
  c, s = np.cos(-yaw), np.sin(-yaw)
  qvel = state[L.qvel]
  vx, vy = qvel[0], qvel[1]
  qvel[0], qvel[1] = c * vx - s * vy, s * vx + c * vy
  state[L.qpos], state[L.qvel] = qpos, qvel


def library_seeds(paths, layout, half):
  """Saved gaits and their left/right mirror images, as seeds for this
  layout (harmonics padded or cut) and these command half-ranges."""
  out = []
  for path in paths or []:
    z = np.load(path, allow_pickle=True)
    theta = G.Layout(int(z['k'])).with_k(z['theta'], layout.k)
    old = z['half'] if 'half' in z else \
        C.load_model(ground=False).actuator_ctrlrange[:, 1]
    theta = G.rescale_theta(layout, theta, old, half)
    out += [G.clip_theta(layout, theta),
            G.clip_theta(layout, G.mirror_theta(layout, theta))]
  return np.array(out).reshape(-1, layout.size)


def splice_seeds(thetas, layout):
  """Right legs from one gait, left legs from another (period of the first),
  for every ordered pair: e.g. right legs rowing toward -Y and left legs
  toward +Y, a turn."""
  out = []
  for i, ti in enumerate(thetas):
    for j, tj in enumerate(thetas):
      if i == j:
        continue
      log_t, c, a, b = layout.split(ti)
      _, c2, a2, b2 = layout.split(tj)
      c, a, b = c.copy(), a.copy(), b.copy()
      c[9:], a[9:], b[9:] = c2[9:], a2[9:], b2[9:]
      out.append(layout.join(np.exp(log_t), c, a, b))
  return np.array(out).reshape(-1, layout.size)


def load_reference(path, mode, nphase=200):
  """A kinematic_ref.py gait: joint angles over one stroke and the swim
  direction (unmirrored)."""
  import kinematic_ref
  with open(path) as f:
    ref = json.load(f)
  pose = kinematic_ref.pose_spec(ref, mode)
  phase = np.arange(nphase) / nphase
  return dict(angles=kinematic_ref.joint_angles(pose, phase),
              direction=kinematic_ref.swim_direction(pose).tolist())


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
  p.add_argument('--yaw_rate', type=float, default=0.0,
                 help='yaw-rate target (rad/s) held through a speed ladder')
  p.add_argument('--yaw_ladder', type=float, nargs='*', default=[],
                 help='turning primitives: ladder over these yaw-rate '
                      'targets (rad/s) with the speed held at --turn_speed')
  p.add_argument('--turn_speed', type=float, default=0.0,
                 help='speed target along --direction during a yaw ladder')
  p.add_argument('--robot', default=C.robot(), choices=C.ROBOTS,
                 help='v1: robot_xdomain.xml (MuJoCo 3.2.3); v2: '
                      'robot_v2/Robot.xml with dcmotor servos (MuJoCo >= 4). '
                      'Default: $CRAB_ROBOT, else v2 (crab_model.ROBOTS)')
  p.add_argument('--hold_legs', default='',
                 help="legs held still, e.g. R1,R2,L1,L2: their joints' "
                      "oscillation is zero, their posture stays free")
  p.add_argument('--rate_axis', default='z', choices=('x', 'y', 'z'),
                 help='body axis of a --rate_ladder')
  p.add_argument('--rate_ladder', type=float, nargs='*', default=[],
                 help='rotation primitives: ladder over these body-frame '
                 'angular rates (rad/s) about --rate_axis; needs --from_rest')
  p.add_argument('--rate_dir', type=float, nargs=3, default=None,
                 help='body axis of a --rate_ladder as a direction (e.g. 1 -1 0 '
                 'for a diagonal tumble); overrides --rate_axis')
  p.add_argument('--off_axis_tol', type=float, default=0.05,
                 help='body rate allowed about the other axes of a '
                 '--rate_ladder rung (rad/s)')
  p.add_argument('--attitude', type=float, nargs=2, default=None,
                 help='attitude-held swims: body z axis held at its '
                 'direction after rotating about world x, then y (rad)')
  p.add_argument('--start_attitude', action='store_true',
                 help='start from rest already at --attitude (e.g. upside '
                 'down) instead of level')
  p.add_argument('--yaw_tol', type=float, default=0.01,
                 help='yaw-rate tolerance of a turning rung (rad/s)')
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
  p.add_argument('--seed_files', nargs='*', default=[],
                 help="gait npz files for --seed_style library (each is "
                      "added with its mirror image)")
  p.add_argument('--reference',
                 help='kinematic reference JSON (kinematic_ref.py) to track')
  p.add_argument('--mode', help='the reference gait (mode) to track')
  p.add_argument('--period', type=float, default=1.0,
                 help='start stroke period with --reference (s)')
  p.add_argument('--fix_period', type=float, default=None,
                 help='hold the stroke period fixed at this (s)')
  p.add_argument('--lead', type=float, default=0.03,
                 help='command lead over the reference beyond --tau (s)')
  p.add_argument('--imitate_iters', type=int, default=25,
                 help='LM iterations of the tracking-only step')
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
  p.add_argument('--yaw_drift_tol', type=float, default=0.015,
                 help='yaw-rate drift a straight gait may have (rad/s)')
  p.add_argument('--saturation_tol', type=float, default=0.2,
                 help='fraction of time a servo may sit at its torque limit')
  p.add_argument('--processes', type=int, default=None)
  p.add_argument('--seed', type=int, default=0)
  p.add_argument('--no_render', action='store_true')
  p.add_argument('--out', required=True)
  Chain(p.parse_args()).run()


if __name__ == '__main__':
  main()
