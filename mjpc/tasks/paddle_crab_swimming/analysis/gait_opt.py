#!/usr/bin/env python3
"""Periodic gait optimization for the paddle crab (the cyclic counterpart of
the pterosaur's launch_trajopt.py / launch_ilqr.py).

A gait is a periodic open-loop servo command: per actuator a Fourier series
in phase, plus the period T,

  u_i(t) = clip(half_i * (c_i + sum_k a_ik cos(2 pi k t/T) + b_ik sin(...)))

theta = [log T, c (18), a (18 x K), b (18 x K)] (127 numbers for K = 3).

A gait is scored on its limit cycle: the robot is rolled out from a start
state for `warm` cycles and then `measure` cycles, and only the measured
window counts. Its residual vector (least squares, everything scaled so 1 is
"this much error") holds
  speed       mean forward speed (world -Y) minus the target
  straight    lateral drift rate, yaw rate, depth rate, mean heading
  level       mean and RMS body tilt
  cycle       state at the start of the last cycle minus at its end
              (joints, servo filter states, body-frame velocities, tilt):
              zero on a limit cycle
  effort      RMS servo torque (fraction of the 3.1 N m limit)
  saturation  servo torque beyond its limit, command beyond its range

There are no contacts (the patch disables them all) and the fluid forces are
smooth, so after a sampling stage (es) the residuals are polished with
Levenberg-Marquardt on theta, with the Jacobian from forward differences done
as one batched rollout (lm). Per-step controls (iLQR) are not used: they
would not stay periodic.
"""
import multiprocessing as mp
import os

import mujoco
import mujoco.rollout
import numpy as np

import crab_model as C

K_DEFAULT = 3
T_MIN, T_MAX = 0.35, 3.0

# Residual scales: a residual of 1 is this much error.
S_SPEED = 0.005       # m/s
S_LATERAL = 0.003     # m/s drift
S_YAW = 0.02          # rad/s drift
S_DEPTH = 0.003       # m/s drift
S_HEADING = 0.1       # rad mean heading off the start heading
S_TILT = 0.1          # rad mean / RMS tilt
S_CYCLE_Q = 0.02      # rad joint / filter state mismatch over one cycle
S_CYCLE_V = 0.01      # m/s body velocity mismatch
S_CYCLE_W = 0.05      # rad/s body angular velocity mismatch
S_SAT = 0.05          # fraction of the force / command range

DEFAULT_WEIGHTS = dict(speed=4.0, lateral=1.0, yaw=1.0, depth=0.3,
                       heading=0.3, tilt=0.3, cycle=0.3, effort=1.0,
                       saturation=3.0)


class Layout:
  """Indexing of theta."""

  def __init__(self, k=K_DEFAULT, nu=C.NU):
    self.k, self.nu = k, nu
    self.size = 1 + nu * (1 + 2 * k)

  def split(self, theta):
    theta = np.asarray(theta)
    nu, k = self.nu, self.k
    log_t = theta[..., 0]
    c = theta[..., 1:1 + nu]
    a = theta[..., 1 + nu:1 + nu + nu * k].reshape(theta.shape[:-1] + (nu, k))
    b = theta[..., 1 + nu + nu * k:].reshape(theta.shape[:-1] + (nu, k))
    return log_t, c, a, b

  def join(self, period, c, a, b):
    return np.concatenate([[np.log(period)], np.ravel(c), np.ravel(a),
                           np.ravel(b)])

  def period(self, theta):
    return float(np.exp(theta[0]))

  def with_k(self, theta, k):
    """theta re-expressed with k harmonics (zero-padded or truncated)."""
    log_t, c, a, b = self.split(theta)
    a2 = np.zeros((self.nu, k))
    b2 = np.zeros((self.nu, k))
    n = min(k, self.k)
    a2[:, :n], b2[:, :n] = a[:, :n], b[:, :n]
    return Layout(k, self.nu).join(np.exp(log_t), c, a2, b2)


def raw_command(layout, theta, phase):
  """Unclipped normalized command (nu, len(phase)) at phases in cycles."""
  _, c, a, b = layout.split(theta)
  ks = np.arange(1, layout.k + 1)
  arg = 2 * np.pi * np.outer(ks, phase)                  # (k, n)
  return c[:, None] + a @ np.cos(arg) + b @ np.sin(arg)


def controls(layout, theta, half, n, dt, phase0=0.0):
  """Servo commands (n, nu) for steps 0..n-1 and the clip excess."""
  period = layout.period(theta)
  phase = phase0 + np.arange(n) * dt / period
  raw = raw_command(layout, theta, phase)
  excess = np.maximum(np.abs(raw) - 1.0, 0.0)
  u = np.clip(raw, -1.0, 1.0) * half[:, None]
  return u.T, excess.T


# ----- Evaluation ----- #

class Problem:
  """Evaluates gaits; one instance per process."""

  def __init__(self, dt=0.005, k=K_DEFAULT, warm=3, measure=2,
               speed=0.05, max_speed=False, weights=None, x0=None,
               tau=C.TAU):
    self.dt, self.warm, self.measure = dt, warm, measure
    self.speed, self.max_speed = speed, max_speed
    self.weights = dict(DEFAULT_WEIGHTS, **(weights or {}))
    self.layout = Layout(k)
    self.m = C.load_model(dt=dt, tau=tau, ground=False)
    C.check_layout(self.m)
    self.d = mujoco.MjData(self.m)
    self.state_layout = C.StateLayout(self.m)
    self.half = self.m.actuator_ctrlrange[:, 1].copy()
    self.mass = C.total_mass(self.m)
    self.x0 = C.initial_state(self.m) if x0 is None else np.asarray(x0)

  def config(self):
    return dict(dt=self.dt, k=self.layout.k, warm=self.warm,
                measure=self.measure, speed=self.speed,
                max_speed=self.max_speed, weights=self.weights,
                x0=self.x0.tolist())

  def nstep(self, period):
    return int(np.ceil((self.warm + self.measure) * period / self.dt)) + 2

  def rollout(self, thetas):
    """States (n, nstep, nstate) and clip excess for a batch of thetas."""
    thetas = np.atleast_2d(thetas)
    nstep = max(self.nstep(self.layout.period(t)) for t in thetas)
    ctrl = np.empty((len(thetas), nstep, C.NU))
    excess = np.empty((len(thetas), nstep, C.NU))
    for i, theta in enumerate(thetas):
      ctrl[i], excess[i] = controls(self.layout, theta, self.half, nstep,
                                    self.dt)
    # rollout's states are after each step; prepend the start state.
    states, _ = mujoco.rollout.rollout(self.m, self.d, self.x0, ctrl)
    x0 = np.broadcast_to(self.x0, (len(thetas), 1, self.x0.size))
    states = np.concatenate([x0, states], axis=1)
    return states, ctrl, excess

  def evaluate(self, thetas):
    """(residuals (n, nres), metrics list) for a batch of thetas."""
    states, ctrl, excess = self.rollout(thetas)
    res, info = [], []
    for i, theta in enumerate(np.atleast_2d(thetas)):
      r, metrics = self.residual_from(theta, states[i], excess[i])
      res.append(r)
      info.append(metrics)
    return np.array(res), info

  def residual_from(self, theta, states, excess):
    L = self.state_layout
    dt, period = self.dt, self.layout.period(theta)
    qpos, qvel, act = states[:, L.qpos], states[:, L.qvel], states[:, L.act]

    def at(x, t):
      """Linear interpolation of rows of x at time t."""
      f = t / dt
      i = min(int(np.floor(f)), len(x) - 2)
      w = f - i
      return (1 - w) * x[i] + w * x[i + 1]

    t0 = self.warm * period
    t1 = (self.warm + self.measure) * period
    tl = t1 - period                                   # last cycle start
    i0, i1 = int(np.ceil(t0 / dt)), int(np.floor(t1 / dt))
    window = slice(i0, i1 + 1)
    duration = t1 - t0

    p0, p1 = at(qpos[:, :3], t0), at(qpos[:, :3], t1)
    disp = p1 - p0
    speed = float(C.FORWARD @ disp) / duration
    lateral = float(C.LATERAL @ disp) / duration
    depth = float(disp[2]) / duration

    R = C.quat_to_mat(qpos[:, 3:7])
    yaw = np.unwrap(C.yaw_of(R))
    yaw_rate = float(at(yaw, t1) - at(yaw, t0)) / duration
    yaw0 = C.yaw_of(C.quat_to_mat(self.x0[L.qpos][3:7]))
    heading = float(np.mean(yaw[window]) - yaw0)
    tilt = R[window][:, :2, 2]                          # body z in world xy
    tilt_mean = tilt.mean(axis=0)
    tilt_rms = float(np.sqrt(np.mean(np.sum((tilt - tilt_mean) ** 2, -1))))

    # Body-frame base velocities: free joint qvel is world linear velocity
    # and body-frame angular velocity.
    lin_body = np.einsum('nji,nj->ni', R, qvel[:, :3])
    ang_body = qvel[:, 3:6]

    def cycle_state(t):
      return np.concatenate([
          at(qpos[:, 7:], t) / S_CYCLE_Q,
          at(act, t) / S_CYCLE_Q,
          at(qvel[:, 6:], t) * period / (2 * np.pi) / S_CYCLE_Q,
          at(lin_body, t) / S_CYCLE_V,
          at(ang_body, t) / S_CYCLE_W,
          at(R[:, :2, 2], t) / S_CYCLE_Q])
    cycle = cycle_state(t1) - cycle_state(tl)

    q, a, qd = qpos[window, 7:], act[window], qvel[window, 6:]
    force = C.actuator_force(self.m, q, a)
    effort = float(np.sqrt(np.mean((force / C.FORCE_LIMIT) ** 2)))
    servo = C.KP * (a - q)
    force_excess = np.maximum(np.abs(servo) - C.FORCE_LIMIT, 0) / C.FORCE_LIMIT
    saturated = float(np.mean(np.abs(servo) > C.FORCE_LIMIT))
    power = force * qd
    work = float(np.sum(np.maximum(power, 0)) * dt)      # positive work
    distance = max(abs(speed) * duration, 1e-9)
    cot = work / (self.mass * 9.81 * distance)

    w = self.weights
    if self.max_speed:
      r_speed = (1.0 - speed) / S_SPEED * 0.05     # constant pull to faster
    else:
      r_speed = (speed - self.speed) / S_SPEED
    r = np.concatenate([
        [np.sqrt(w['speed']) * r_speed,
         np.sqrt(w['lateral']) * lateral / S_LATERAL,
         np.sqrt(w['yaw']) * yaw_rate / S_YAW,
         np.sqrt(w['depth']) * depth / S_DEPTH,
         np.sqrt(w['heading']) * heading / S_HEADING],
        np.sqrt(w['tilt']) * tilt_mean / S_TILT,
        [np.sqrt(w['tilt']) * tilt_rms / S_TILT],
        np.sqrt(w['cycle'] / cycle.size) * cycle,
        [np.sqrt(w['effort']) * effort / 0.1],
        np.sqrt(w['saturation']) * np.sqrt(
            np.mean(force_excess ** 2, axis=0)) / S_SAT,
        np.sqrt(w['saturation']) * np.sqrt(
            np.mean(excess[window] ** 2, axis=0)) / S_SAT,
    ])
    metrics = dict(
        period=period, speed=speed, lateral_rate=lateral, yaw_rate=yaw_rate,
        depth_rate=depth, heading=heading, tilt_mean=float(np.hypot(*tilt_mean)),
        tilt_rms=tilt_rms, cycle_error=float(np.sqrt(np.mean(cycle ** 2))),
        effort=effort, saturated=saturated,
        command_clipped=float(np.mean(excess[window] > 0)),
        work_per_cycle=work / self.measure, cot=cot,
        cost=float(0.5 * r @ r))
    return r, metrics

  def limit_cycle(self, theta, nphase=200):
    """Samples of the last measured cycle at uniform phase.

    Returns dict with phase, time, qpos, qvel, act, ctrl arrays (nphase rows,
    phase 0 at the start of the last cycle) and the state there.
    """
    states, ctrl, _ = self.rollout(theta[None])
    states, ctrl = states[0], ctrl[0]
    L, dt = self.state_layout, self.dt
    period = self.layout.period(theta)
    tl = (self.warm + self.measure - 1) * period
    phase = np.arange(nphase) / nphase
    times = tl + phase * period
    f = times / dt
    i = np.floor(f).astype(int)
    w = (f - i)[:, None]
    x = (1 - w) * states[i] + w * states[i + 1]
    # The command at a state's time is the one applied over the next step.
    cmd = (np.clip(raw_command(self.layout, theta, phase), -1, 1)
           * self.half[:, None]).T
    return dict(phase=phase, time=times, qpos=x[:, L.qpos], qvel=x[:, L.qvel],
                act=x[:, L.act], ctrl=cmd, state=x[0])


# ----- Parallel evaluation ----- #

_PROBLEM = None


def _init(config):
  global _PROBLEM
  _PROBLEM = Problem(**config)


def _evaluate(thetas):
  return _PROBLEM.evaluate(thetas)


class Evaluator:
  """A pool of processes, each holding its own Problem."""

  def __init__(self, problem, processes=None):
    self.problem = problem
    self.processes = processes or os.cpu_count()
    ctx = mp.get_context('fork')
    self.pool = ctx.Pool(self.processes, _init, (problem.config(),))

  def __call__(self, thetas):
    thetas = np.atleast_2d(thetas)
    chunks = np.array_split(thetas, min(len(thetas), self.processes * 2))
    out = self.pool.map(_evaluate, chunks)
    res = np.concatenate([o[0] for o in out])
    info = [i for o in out for i in o[1]]
    return res, info

  def close(self):
    self.pool.close()
    self.pool.join()

  def __enter__(self):
    return self

  def __exit__(self, *exc):
    self.close()


def clip_theta(layout, theta):
  theta = np.array(theta, dtype=float)
  theta[..., 0] = np.clip(theta[..., 0], np.log(T_MIN), np.log(T_MAX))
  theta[..., 1:] = np.clip(theta[..., 1:], -2.0, 2.0)
  return theta


# ----- Optimizers ----- #

def es(evaluate, layout, base, iters=100, pop=48, sigma=0.05,
       sigma_period=0.05, sigma_min=0.005, seed=0, log=print):
  """Elitist evolution strategy around the best gait so far.

  Each generation samples around the current best and moves only when a
  sample beats it; one extra sample continues the last successful step.
  sigma follows the 1/5 success rule. (CEM-style averaging of an elite
  would cancel out strokes of different phase, as the MJPC README notes for
  the Cross-Entropy planner; the pterosaur's launch_track_refine.py uses the
  same elitist scheme.)
  """
  rng = np.random.default_rng(seed)
  base = clip_theta(layout, base)
  scale = np.full(layout.size, 1.0)
  scale[0] = sigma_period / sigma
  res, info = evaluate(base[None])
  best_cost, best_info = 0.5 * res[0] @ res[0], info[0]
  momentum = np.zeros(layout.size)
  history = []
  for it in range(iters):
    samples = base + sigma * scale * rng.standard_normal((pop, layout.size))
    samples[0] = base + momentum
    samples = clip_theta(layout, samples)
    res, info = evaluate(samples)
    cost = 0.5 * np.sum(res ** 2, axis=1)
    i = int(np.argmin(cost))
    success = np.mean(cost < best_cost)
    if cost[i] < best_cost:
      momentum = samples[i] - base
      base, best_cost, best_info = samples[i], cost[i], info[i]
    else:
      momentum[:] = 0
    sigma = max(sigma_min, sigma * (1.2 if success > 0.2 else 0.85))
    m = best_info
    history.append(dict(m, iter=it, cost=float(best_cost), sigma=sigma))
    if log and (it % 10 == 0 or it == iters - 1):
      log(f'  es {it:4d} cost {best_cost:10.3f} v {m["speed"]:+.4f} '
          f'T {m["period"]:.3f} lat {m["lateral_rate"]:+.4f} '
          f'yaw {m["yaw_rate"]:+.3f} cyc {m["cycle_error"]:.2f} '
          f'eff {m["effort"]:.2f} sigma {sigma:.3f}')
  return base, best_cost, history


def lm(evaluate, layout, theta, iters=30, eps=1e-4, mu=1e-2, max_step=0.3,
       log=print):
  """Levenberg-Marquardt on the residuals, forward-difference Jacobian.

  The n + 1 rollouts of one Jacobian go out as one batch. mu adapts like
  launch_ilqr.py: /3 on an accepted step, x10 on a rejected one.
  """
  theta = clip_theta(layout, theta)
  res, info = evaluate(theta[None])
  r, cost = res[0], 0.5 * res[0] @ res[0]
  history = [dict(info[0], iter=0, cost=float(cost), mu=mu)]
  n = layout.size
  for it in range(1, iters + 1):
    probes = theta + eps * np.eye(n)
    res_p, _ = evaluate(np.vstack([theta[None], probes]))
    r = res_p[0]
    cost = 0.5 * r @ r
    J = (res_p[1:] - r).T / eps                        # (nres, n)
    g = J.T @ r
    H = J.T @ J
    accepted = False
    while mu < 1e6:
      step = -np.linalg.solve(H + mu * np.diag(np.diag(H) + 1e-6), g)
      norm = np.max(np.abs(step))
      if norm > max_step:
        step *= max_step / norm
      trial = clip_theta(layout, theta + step)
      res_t, info_t = evaluate(trial[None])
      cost_t = 0.5 * res_t[0] @ res_t[0]
      if cost_t < cost:
        theta, r, cost, info = trial, res_t[0], cost_t, info_t
        mu = max(mu / 3, 1e-6)
        accepted = True
        break
      mu *= 10
    m = info[0]
    history.append(dict(m, iter=it, cost=float(cost), mu=mu))
    if log:
      log(f'  lm {it:4d} cost {cost:10.4f} mu {mu:8.1e} v {m["speed"]:+.4f} '
          f'T {m["period"]:.3f} lat {m["lateral_rate"]:+.4f} '
          f'yaw {m["yaw_rate"]:+.3f} cyc {m["cycle_error"]:.3f} '
          f'eff {m["effort"]:.3f} sat {m["saturated"]:.3f}')
    if not accepted:
      break
  return theta, cost, history


# ----- Seeds ----- #

def seeds(layout, n=256, period=1.0, amplitude=0.6, seed=0):
  """Random first-harmonic strokes: half left/right mirrored, half free.

  A stroke per actuator is c + A cos(phase - phi); amplitudes and phases are
  random, so the batch covers rowing (proximal sweep + paddle feather in
  quadrature), heaving and mixed strokes.
  """
  rng = np.random.default_rng(seed)
  out = []
  for i in range(n):
    amp = amplitude * rng.uniform(0, 1, C.NU) * (rng.uniform(size=C.NU) < 0.6)
    phi = rng.uniform(-np.pi, np.pi, C.NU)
    c = 0.2 * rng.standard_normal(C.NU)
    if i % 2 == 0:  # mirrored: left copies right through MirrorSign
      right = slice(0, 9)
      amp[9:], phi[9:], c[9:] = amp[right], phi[right], c[right]
      flip = C.MIRROR_SIGN[9:] < 0
      phi[9:][flip] += np.pi
      c[9:][flip] *= -1
    a = np.zeros((C.NU, layout.k))
    b = np.zeros((C.NU, layout.k))
    a[:, 0], b[:, 0] = amp * np.cos(phi), amp * np.sin(phi)
    t = period * np.exp(0.4 * rng.standard_normal())
    out.append(clip_theta(layout, layout.join(t, c, a, b)))
  return np.array(out)


def mirror_theta(layout, theta):
  """The left/right mirror image of a gait (same speed, opposite drift)."""
  log_t, c, a, b = layout.split(theta)
  s = C.MIRROR_SIGN[:, None]
  idx = C.MIRROR_INDEX
  return layout.join(np.exp(log_t), C.MIRROR_SIGN * c[idx], s * a[idx],
                     s * b[idx])


def symmetry_index(layout, theta):
  """0 for a left/right mirrored gait, ~1 for an unrelated one."""
  t = np.asarray(theta)[1:]
  mt = mirror_theta(layout, theta)[1:]
  return float(np.linalg.norm(t - mt) / max(np.linalg.norm(t) * 2, 1e-9))
