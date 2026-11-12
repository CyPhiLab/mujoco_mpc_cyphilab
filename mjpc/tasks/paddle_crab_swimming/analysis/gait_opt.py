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
  speed       mean speed along the swim direction minus the target
  straight    lateral drift rate, yaw rate (minus its target), depth rate,
              mean heading (off while turning)
  level       steady body tilt (weight 'tilt') and its oscillation
              about that ('wobble')
  cycle       state at the start of the last cycle minus at its end
              (joints, servo filter states, body-frame velocities, tilt):
              zero on a limit cycle
  effort      RMS servo torque (fraction of the 3.1 N m limit)
  energy      cost of transport: positive servo work / (m g distance)
              (weight 0 by default, then left out)
  saturation  servo torque beyond its limit, command beyond its range
  track       (with a reference) joint angles over the last cycle minus a
              reference stroke's (e.g. kinematic_ref.py's) at the same phase

There are no contacts (the patch disables them all) and the fluid forces are
smooth, so after a sampling stage (es) the residuals are polished with
Levenberg-Marquardt on theta, with the Jacobian from forward differences done
as one batched rollout (lm). Per-step controls (iLQR) are not used: they
would not stay periodic.
"""
import json
import multiprocessing as mp
import os
import pickle

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
S_TILT = 0.1          # rad steady tilt / tilt oscillation (wobble)
S_CYCLE_Q = 0.02      # rad joint / filter state mismatch over one cycle
S_CYCLE_V = 0.01      # m/s body velocity mismatch
S_CYCLE_W = 0.05      # rad/s body angular velocity mismatch
S_SAT = 0.05          # fraction of the force / command range
S_TRACK = 0.1         # rad joint angle off the reference
TRACK_SAMPLES = 40    # reference phases compared per cycle

DEFAULT_WEIGHTS = dict(speed=4.0, lateral=1.0, yaw=1.0, depth=0.3,
                       heading=0.3, tilt=0.3, wobble=0.3, cycle=0.3,
                       effort=1.0, energy=0.0,
                       saturation=1.0, track=1.0, attitude_rate=0.0)


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
               tau=C.TAU, direction=tuple(C.FORWARD), ctrl_limit=None,
               design=None, reference=None, fixed_period=None,
               yaw_rate=0.0, hold=None, robot=None, body_rate=None,
               attitude=None, heading_frame=False):
    self.dt, self.warm, self.measure = dt, warm, measure
    # Swim direction in the world frame; the body keeps its start heading,
    # so this picks which side of the robot leads. It may point up or down
    # (vertical primitives).
    self.direction = np.array(direction, dtype=float)
    self.direction /= np.linalg.norm(self.direction)
    self.lateral, self.vertical = drift_axes(self.direction)
    # Yaw rate target (rad/s, + is counterclockwise seen from above): turning
    # primitives. While turning the mean-heading residual is off and the tilt
    # is measured in the turning heading frame.
    self.yaw_rate = float(yaw_rate)
    # Body-frame angular-rate target (rad/s about body x, y, z): rotation
    # primitives about any axis (pitch, roll). While rotating, the heading,
    # tilt and wobble residuals are off, the yaw residual becomes the mean
    # body angular velocity against the target (all three axes), and the
    # cycle residual leaves out the world attitude.
    self.body_rate = None if body_rate is None else np.asarray(
        body_rate, dtype=float)
    # Attitude target (rad, rotation about world x, then world y): the body
    # z axis is held at that attitude's instead of level (attitude-held
    # swims; (pi, 0) is upside down).
    self.attitude = None if attitude is None else np.asarray(attitude,
                                                             dtype=float)
    # Measure speed and drift in the heading frame (turned by the mean
    # heading change over the scored cycles) instead of the world frame:
    # arcs and helices, whose heading turns while they swim.
    self.heading_frame = bool(heading_frame)
    # Actuators held still: their oscillation (a, b) is zero, their offset
    # (posture) stays free.
    self.hold = sorted(int(i) for i in (hold or []))
    self.speed, self.max_speed = speed, max_speed
    self.weights = dict(DEFAULT_WEIGHTS, **(weights or {}))
    self.layout = Layout(k)
    self.ctrl_limit = ctrl_limit
    self.design = dict(design or {})
    self.tau = tau
    self.robot = robot or C.robot()
    self.m = C.load_model(dt=dt, tau=tau, ground=False, ctrl_limit=ctrl_limit,
                          design=self.design, robot=self.robot)
    # v2: servo force comes from actuatorfrc sensors in the rollout.
    self.v2 = C.is_v2(self.m)
    C.check_layout(self.m)
    self.d = mujoco.MjData(self.m)
    self.state_layout = C.StateLayout(self.m)
    self.half = self.m.actuator_ctrlrange[:, 1].copy()
    self.mass = C.total_mass(self.m)
    self.x0 = C.initial_state(self.m) if x0 is None else np.asarray(x0)
    # A fixed stroke period overrides theta's (its log T is then ignored).
    self.fixed_period = fixed_period
    # A reference stroke: joint angles (nphase, nu) at uniform phases.
    self.reference = None if reference is None else np.asarray(reference,
                                                               float)

  def config(self):
    return dict(dt=self.dt, k=self.layout.k, warm=self.warm,
                measure=self.measure, speed=self.speed,
                max_speed=self.max_speed, weights=self.weights,
                x0=self.x0.tolist(), direction=self.direction.tolist(),
                ctrl_limit=self.ctrl_limit, design=self.design, tau=self.tau,
                reference=None if self.reference is None
                else self.reference.tolist(), fixed_period=self.fixed_period,
                yaw_rate=self.yaw_rate, hold=self.hold, robot=self.robot,
                body_rate=None if self.body_rate is None
                else self.body_rate.tolist(),
                attitude=None if self.attitude is None
                else self.attitude.tolist(), heading_frame=self.heading_frame)

  def fix(self, thetas):
    """thetas with the fixed period, if there is one, and the held
    actuators' oscillation zeroed."""
    thetas = np.array(thetas, dtype=float)
    if self.fixed_period:
      thetas[..., 0] = np.log(self.fixed_period)
    if self.hold:
      nu, k = self.layout.nu, self.layout.k
      for i in self.hold:
        for j in range(k):
          thetas[..., 1 + nu + i * k + j] = 0.0            # a_ij
          thetas[..., 1 + nu + nu * k + i * k + j] = 0.0   # b_ij
    return thetas

  def nstep(self, period):
    return int(np.ceil((self.warm + self.measure) * period / self.dt)) + 2

  def rollout(self, thetas):
    """States (n, nstep, nstate) and clip excess for a batch of thetas."""
    thetas = self.fix(np.atleast_2d(thetas))
    nstep = max(self.nstep(self.layout.period(t)) for t in thetas)
    ctrl = np.empty((len(thetas), nstep, C.NU))
    excess = np.empty((len(thetas), nstep, C.NU))
    for i, theta in enumerate(thetas):
      ctrl[i], excess[i] = controls(self.layout, theta, self.half, nstep,
                                    self.dt)
    # rollout's states are after each step; prepend the start state.
    states, sensordata = mujoco.rollout.rollout(self.m, self.d, self.x0, ctrl)
    x0 = np.broadcast_to(self.x0, (len(thetas), 1, self.x0.size))
    states = np.concatenate([x0, states], axis=1)
    if self.v2:
      # Actuator force over each step (sensor data is after the step; the
      # first step's stands in for the start state).
      self.last_force = np.concatenate([sensordata[:, :1], sensordata], axis=1)
    return states, ctrl, excess

  def evaluate(self, thetas):
    """(residuals (n, nres), metrics list) for a batch of thetas."""
    states, ctrl, excess = self.rollout(thetas)
    res, info = [], []
    for i, theta in enumerate(self.fix(np.atleast_2d(thetas))):
      force = self.last_force[i] if self.v2 else None
      r, metrics = self.residual_from(theta, states[i], excess[i], force)
      res.append(r)
      info.append(metrics)
    return np.array(res), info

  def residual_from(self, theta, states, excess, force_all=None):
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
    R = C.quat_to_mat(qpos[:, 3:7])
    yaw = np.unwrap(C.yaw_of(R))
    yaw_rate = float(at(yaw, t1) - at(yaw, t0)) / duration
    yaw0 = C.yaw_of(C.quat_to_mat(self.x0[L.qpos][3:7]))
    axes = (self.direction, self.lateral, self.vertical)
    if self.heading_frame:
      psi = float(np.mean(yaw[window]) - yaw0)
      c, s_ = np.cos(psi), np.sin(psi)
      Rz = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
      axes = tuple(Rz @ x for x in axes)
    speed = float(axes[0] @ disp) / duration
    lateral = float(axes[1] @ disp) / duration
    depth = float(axes[2] @ disp) / duration
    heading = float(np.mean(yaw[window]) - yaw0)
    tilt = R[window][:, :2, 2]                          # body z in world xy
    turning = self.yaw_rate != 0.0
    if turning:
      # Body z in the heading frame, which turns with the body.
      cy, sy = np.cos(yaw[window]), np.sin(yaw[window])
      tilt = np.stack([cy * tilt[:, 0] + sy * tilt[:, 1],
                       -sy * tilt[:, 0] + cy * tilt[:, 1]], axis=1)
      heading = 0.0
    if self.attitude is not None:
      # Body z in world (all three components) against the target's.
      tilt = R[window][:, :, 2] - attitude_z(self.attitude)
    tilt_mean = tilt.mean(axis=0)
    tilt_rms = float(np.sqrt(np.mean(np.sum((tilt - tilt_mean) ** 2, -1))))

    # Body-frame base velocities: free joint qvel is world linear velocity
    # and body-frame angular velocity.
    lin_body = np.einsum('nji,nj->ni', R, qvel[:, :3])
    ang_body = qvel[:, 3:6]
    rotating = self.body_rate is not None
    body_rate = (at(np.cumsum(ang_body, 0) * dt, t1)
                 - at(np.cumsum(ang_body, 0) * dt, t0)) / duration
    body_vel = (at(np.cumsum(lin_body, 0) * dt, t1)
                - at(np.cumsum(lin_body, 0) * dt, t0)) / duration

    def cycle_state(t):
      return np.concatenate([
          at(qpos[:, 7:], t) / S_CYCLE_Q,
          at(act, t) / S_CYCLE_Q,
          at(qvel[:, 6:], t) * period / (2 * np.pi) / S_CYCLE_Q,
          at(lin_body, t) / S_CYCLE_V,
          at(ang_body, t) / S_CYCLE_W,
          np.zeros(0) if rotating else at(R[:, :2, 2], t) / S_CYCLE_Q])
    cycle = cycle_state(t1) - cycle_state(tl)

    q, a, qd = qpos[window, 7:], act[window], qvel[window, 6:]
    limit = self.m.actuator_forcerange[:, 1]
    if force_all is None:
      force = C.actuator_force(self.m, q, a)
      servo = C.servo_command_force(self.m, q, a)
      force_excess = np.maximum(np.abs(servo) - limit, 0) / limit
      saturated = float(np.mean(np.abs(servo) > limit))
    else:
      # dcmotor: the motor's speed limit is voltage saturation, not torque.
      # Effort, saturation and work use the supply fraction u (C.motor_
      # fraction); the "force" for effort and work is the gross motor torque
      # u * stall, which is what v1's servo torque (with joint damping as the
      # back-EMF) measured.
      u = C.motor_fraction(force_all[window], qd)
      force = u * C.V2_STALL
      force_excess = np.maximum(np.abs(u) - 0.95, 0)
      saturated = float(np.mean(np.abs(u) > 0.99))
    effort = float(np.sqrt(np.mean((force / C.FORCE_LIMIT) ** 2)))
    power = force * qd
    work = float(np.sum(np.maximum(power, 0)) * dt)      # positive work
    distance = max(abs(speed) * duration, 1e-9)
    cot = work / (self.mass * 9.81 * distance)

    track = np.zeros(0)
    track_rms = None
    if self.reference is not None:
      phase = np.arange(TRACK_SAMPLES) / TRACK_SAMPLES
      sim = np.array([at(qpos[:, 7:], tl + ph * period) for ph in phase])
      ref = reference_at(self.reference, phase)
      track = (sim - ref).ravel()
      track_rms = float(np.sqrt(np.mean(track ** 2)))

    w = self.weights
    if self.max_speed:
      r_speed = (1.0 - speed) / S_SPEED * 0.05     # constant pull to faster
    else:
      r_speed = (speed - self.speed) / S_SPEED
    if rotating:
      r_rate = np.sqrt(w['yaw']) * (body_rate - self.body_rate) / S_YAW
      r_attitude = np.zeros(0)
      heading = 0.0
    else:
      r_rate = [np.sqrt(w['yaw']) * (yaw_rate - self.yaw_rate) / S_YAW]
      r_attitude = np.append(np.sqrt(w['tilt']) * tilt_mean / S_TILT,
                             np.sqrt(w['wobble']) * tilt_rms / S_TILT)
    r = np.concatenate([
        [np.sqrt(w['speed']) * r_speed,
         np.sqrt(w['lateral']) * lateral / S_LATERAL],
        r_rate,
        [np.sqrt(w['depth']) * depth / S_DEPTH,
         np.sqrt(w['heading']) * heading / S_HEADING],
        r_attitude,
        np.sqrt(w['cycle'] / cycle.size) * cycle,
        [np.sqrt(w['effort']) * effort / 0.1],
        np.sqrt(w['saturation']) * np.sqrt(
            np.mean(force_excess ** 2, axis=0)) / S_SAT,
        np.sqrt(w['saturation']) * np.sqrt(
            np.mean(excess[window] ** 2, axis=0)) / S_SAT,
        np.sqrt(w['track'] / max(track.size, 1)) * track / S_TRACK,
    ])
    if w['energy']:
      r = np.append(r, np.sqrt(w['energy']) * cot)
    if w['attitude_rate'] and not rotating:
      # Mean pitch and roll rate (body x, y) over the scored cycles: with the
      # walking legs held, the attitude creeps (or loops) over many strokes
      # while each scored window still looks straight.
      r = np.append(r, np.sqrt(w['attitude_rate']) * body_rate[:2] / S_YAW)
    metrics = dict(
        period=period, speed=speed, lateral_rate=lateral, yaw_rate=yaw_rate,
        yaw_rate_target=self.yaw_rate,
        depth_rate=depth, heading=heading,
        tilt_mean=float(np.linalg.norm(tilt_mean)),
        body_rate=body_rate.tolist(), body_velocity=body_vel.tolist(),
        velocity=(disp / duration).tolist(), heading_frame=self.heading_frame,
        body_rate_target=None if self.body_rate is None
        else self.body_rate.tolist(),
        attitude_target=None if self.attitude is None
        else self.attitude.tolist(),
        tilt_rms=tilt_rms, cycle_error=float(np.sqrt(np.mean(cycle ** 2))),
        effort=effort, saturated=saturated,
        command_clipped=float(np.mean(excess[window] > 0)),
        work_per_cycle=work / self.measure, cot=cot, track_rms=track_rms,
        cost=float(0.5 * r @ r))
    return r, metrics

  def limit_cycle(self, theta, nphase=200):
    """Samples of the last measured cycle at uniform phase.

    Returns dict with phase, time, qpos, qvel, act, ctrl arrays (nphase rows,
    phase 0 at the start of the last cycle) and the state there.
    """
    theta = self.fix(theta)
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


def step_design(z):
  """A step npz's design overrides ({} in files older than the field)."""
  return json.loads(str(z['design'])) if 'design' in z else {}


def step_tau(z):
  return float(z['tau']) if 'tau' in z else C.TAU


def step_half(z):
  """A step npz's command half-ranges (the task's in older files)."""
  if 'half' in z:
    return np.asarray(z['half'])
  return C.load_model(ground=False).actuator_ctrlrange[:, 1].copy()


def attitude_z(attitude):
  """World direction of the body z axis after rotating about world x by
  attitude[0], then about world y by attitude[1]."""
  ax, ay = attitude
  z = np.array([0.0, -np.sin(ax), np.cos(ax)])          # Rx(ax) e_z
  return np.array([np.cos(ay) * z[0] + np.sin(ay) * z[2], z[1],
                   -np.sin(ay) * z[0] + np.cos(ay) * z[2]])  # Ry(ay) z


def drift_axes(direction):
  """The two axes drift is measured along for a swim direction: lateral
  (horizontal, to the left of the direction) and vertical (completing the
  frame; world up for a horizontal direction). For a vertical direction the
  lateral axis is world +X and the "vertical" one world +-Y."""
  up = np.array([0.0, 0.0, 1.0])
  lateral = np.cross(up, direction)
  if np.linalg.norm(lateral) < 1e-9:
    lateral = np.array([1.0, 0.0, 0.0])
  lateral /= np.linalg.norm(lateral)
  if abs(direction[2]) < 1e-12:
    return lateral, up
  vertical = np.cross(direction, lateral)
  return lateral, vertical / np.linalg.norm(vertical)


def reference_at(reference, phase):
  """Rows of a reference stroke (uniform phases) at the given phases,
  periodic linear interpolation."""
  n = len(reference)
  ext = np.vstack([reference, reference[:1]])
  grid = np.arange(n + 1) / n
  phase = np.asarray(phase, float) % 1.0
  return np.stack([np.interp(phase, grid, ext[:, j])
                   for j in range(reference.shape[1])], axis=1)


def fit_reference(layout, reference, period, half, lead=0.0):
  """theta whose commands are the reference stroke (Fourier least squares
  with layout.k harmonics), shifted earlier by lead seconds to make up for
  the servo lag."""
  n = len(reference)
  phase = np.arange(n) / n
  target = reference_at(reference, phase + lead / period) / half
  ks = np.arange(1, layout.k + 1)
  basis = np.hstack([np.ones((n, 1)), np.cos(2 * np.pi * np.outer(phase, ks)),
                     np.sin(2 * np.pi * np.outer(phase, ks))])
  coef, *_ = np.linalg.lstsq(basis, target, rcond=None)
  c, a, b = coef[0], coef[1:1 + layout.k].T, coef[1 + layout.k:].T
  return clip_theta(layout, layout.join(period, c, a, b))


# ----- Parallel evaluation ----- #

_PROBLEM = None


def _init(config):
  global _PROBLEM
  _PROBLEM = Problem(**config)


def _evaluate(thetas):
  return _PROBLEM.evaluate(thetas)


class Evaluator:
  """A pool of processes, each holding its own Problem."""

  def __init__(self, problem, processes=None, max_batch=32):
    self.problem = problem
    self.processes = processes or os.cpu_count()
    self.max_batch = max_batch
    ctx = mp.get_context('fork')
    self.pool = ctx.Pool(self.processes, _init, (problem.config(),))

  def __call__(self, thetas):
    thetas = np.atleast_2d(thetas)
    # Each task keeps every state of its rollouts in memory (a long-period
    # gait is ~10^4 steps), so tasks stay small; a worker killed for memory
    # would hang Pool.map.
    n = max(1, min(self.max_batch,
                   int(np.ceil(len(thetas) / (self.processes * 2)))))
    chunks = [thetas[i:i + n] for i in range(0, len(thetas), n)]
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


def rescale_theta(layout, theta, half_from, half_to):
  """theta for command half-ranges half_to giving the same commands in
  radians as theta did for half_from."""
  log_t, c, a, b = layout.split(theta)
  r = np.asarray(half_from) / np.asarray(half_to)
  return layout.join(np.exp(log_t), c * r, a * r[:, None], b * r[:, None])


def clip_theta(layout, theta):
  theta = np.array(theta, dtype=float)
  theta[..., 0] = np.clip(theta[..., 0], np.log(T_MIN), np.log(T_MAX))
  theta[..., 1:] = np.clip(theta[..., 1:], -2.0, 2.0)
  return theta


# ----- Optimizers ----- #

def load_checkpoint(path):
  """An optimizer checkpoint (see es and lm), or None."""
  if not path or not os.path.exists(path):
    return None
  with open(path, 'rb') as f:
    return pickle.load(f)


def save_checkpoint(path, state):
  """Writes atomically, so a run killed mid-write keeps the last one."""
  if not path:
    return
  tmp = path + '.tmp'
  with open(tmp, 'wb') as f:
    pickle.dump(state, f)
  os.replace(tmp, path)


def es(evaluate, layout, base, iters=100, pop=48, sigma=0.05,
       sigma_period=0.05, sigma_min=0.005, seed=0, log=print,
       checkpoint=None):
  """Elitist evolution strategy around the best gait so far.

  Each generation samples around the current best and moves only when a
  sample beats it; one extra sample continues the last successful step.
  sigma follows the 1/5 success rule. (CEM-style averaging of an elite
  would cancel out strokes of different phase, as the MJPC README notes for
  the Cross-Entropy planner; the pterosaur's launch_track_refine.py uses the
  same elitist scheme.)

  checkpoint: a file the state is saved to after every generation; a rerun
  with the same file continues from it (a finished run returns at once).
  """
  rng = np.random.default_rng(seed)
  base = clip_theta(layout, base)
  scale = np.full(layout.size, 1.0)
  scale[0] = sigma_period / sigma
  saved = load_checkpoint(checkpoint)
  if saved:
    base, best_cost, best_info = saved['base'], saved['cost'], saved['info']
    momentum, sigma, start = saved['momentum'], saved['sigma'], saved['it']
    history = saved['history']
    rng.bit_generator.state = saved['rng']
    if log:
      log(f'  es resumed at generation {start} (cost {best_cost:.3f})')
  else:
    res, info = evaluate(base[None])
    best_cost, best_info = 0.5 * res[0] @ res[0], info[0]
    momentum = np.zeros(layout.size)
    history, start = [], 0
  for it in range(start, iters):
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
    save_checkpoint(checkpoint, dict(
        base=base, cost=best_cost, info=best_info, momentum=momentum,
        sigma=sigma, it=it + 1, history=history, rng=rng.bit_generator.state))
    if log and (it % 10 == 0 or it == iters - 1):
      log(f'  es {it:4d} cost {best_cost:10.3f} v {m["speed"]:+.4f} '
          f'T {m["period"]:.3f} lat {m["lateral_rate"]:+.4f} '
          f'yaw {m["yaw_rate"]:+.3f} cyc {m["cycle_error"]:.2f} '
          f'eff {m["effort"]:.2f} sigma {sigma:.3f}')
  return base, best_cost, history


def lm(evaluate, layout, theta, iters=30, eps=1e-4, mu=1e-2, max_step=0.3,
       log=print, checkpoint=None):
  """Levenberg-Marquardt on the residuals, forward-difference Jacobian.

  The n + 1 rollouts of one Jacobian go out as one batch. mu adapts like
  launch_ilqr.py: /3 on an accepted step, x10 on a rejected one.

  checkpoint: a file the state is saved to after every iteration; a rerun
  with the same file continues from it (a finished run returns at once).
  """
  theta = clip_theta(layout, theta)
  n = layout.size
  saved = load_checkpoint(checkpoint)
  if saved:
    theta, cost, info = saved['theta'], saved['cost'], saved['info']
    mu, start, history = saved['mu'], saved['it'], saved['history']
    if saved['done']:
      return theta, cost, history
    if log:
      log(f'  lm resumed at iteration {start} (cost {cost:.4f})')
  else:
    res, info = evaluate(theta[None])
    r, cost = res[0], 0.5 * res[0] @ res[0]
    history = [dict(info[0], iter=0, cost=float(cost), mu=mu)]
    start = 1
  accepted = True
  for it in range(start, iters + 1):
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
    save_checkpoint(checkpoint, dict(
        theta=theta, cost=cost, info=info, mu=mu, it=it + 1,
        history=history, done=not accepted or it == iters))
    if not accepted:
      break
  save_checkpoint(checkpoint, dict(theta=theta, cost=cost, info=info, mu=mu,
                                   it=iters + 1, history=history, done=True))
  return theta, cost, history


# ----- Seeds ----- #

def seeds(layout, n=256, period=1.0, amplitude=0.6, seed=0, offset=None):
  """Random first-harmonic strokes: half left/right mirrored, half free.

  Offsets (the posture the stroke oscillates about) are N(0, 0.2) of each
  half-range by default, i.e. near the zero pose; offset=x draws them
  uniformly within +-x of the half-range instead.

  A stroke per actuator is c + A cos(phase - phi); amplitudes and phases are
  random, so the batch covers rowing (proximal sweep + paddle feather in
  quadrature), heaving and mixed strokes.
  """
  rng = np.random.default_rng(seed)
  out = []
  for i in range(n):
    amp = amplitude * rng.uniform(0, 1, C.NU) * (rng.uniform(size=C.NU) < 0.6)
    phi = rng.uniform(-np.pi, np.pi, C.NU)
    c = (0.2 * rng.standard_normal(C.NU) if offset is None
         else rng.uniform(-offset, offset, C.NU))
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


def crab_seeds(layout, half, n=256, direction=(1, 0, 0), seed=0):
  """Strokes modelled on blue crab swimming (sideways, along the leg axis).

  The crab holds its walking legs still in a streamlined pose and drives
  itself with the rear paddles raised high, sculling: a fore-aft sweep with
  the blade feathered in quadrature, the two paddles offset in phase. Here
  the rear legs are the paddles (mid joint raises, proximal sweeps, distal
  feathers); the front and centre legs get constant offsets, the front ones
  swept toward the swim axis, with small oscillations. direction is +-X.
  Commands are in radians, then normalized by half (the command
  half-ranges).
  """
  rng = np.random.default_rng(seed)
  side = 1.0 if direction[0] >= 0 else -1.0
  out = []
  for _ in range(n):
    c = np.zeros(C.NU)
    a = np.zeros((C.NU, layout.k))
    b = np.zeros((C.NU, layout.k))

    def stroke(i, amp, phase):
      a[i, 0] += amp * np.cos(phase)
      b[i, 0] += amp * np.sin(phase)

    # Walking legs (front and centre, both sides): streamlined offsets. The
    # front legs (+-45 deg at rest) turn toward the X axis: right proximal
    # positive turns a right leg toward +Y, left proximal toward -Y.
    for leg, sweep in ((0, -0.79), (3, 0.0), (9, 0.79), (12, 0.0)):
      c[leg] = sweep * rng.uniform(0.3, 1.2) + 0.2 * rng.standard_normal()
      c[leg + 1] = rng.uniform(-0.8, 0.8)                 # mid: hold up/down
      c[leg + 2] = rng.uniform(-0.6, 0.6)                 # distal
      for j in range(3):
        stroke(leg + j, rng.uniform(0, 0.25), rng.uniform(-np.pi, np.pi))
    # Paddles: raised (beside the body up to over the back), sculling.
    lift = rng.uniform(1.0, 2.6)
    offset = rng.choice([0.0, 0.5 * np.pi, np.pi, rng.uniform(-np.pi, np.pi)])
    sweep_amp = rng.uniform(0.3, 1.0)
    feather_amp = rng.uniform(0.5, 1.5)
    feather_lead = rng.choice([-0.5 * np.pi, 0.5 * np.pi])
    heave_amp = rng.uniform(0.0, 0.5)
    for k, (prox, sign) in enumerate(((6, 1.0), (15, -1.0))):
      phase = offset * k
      c[prox] = 0.3 * rng.standard_normal()
      c[prox + 1] = lift + 0.15 * rng.standard_normal()
      c[prox + 2] = 0.3 * rng.standard_normal()
      stroke(prox, sweep_amp, phase)
      stroke(prox + 1, heave_amp, phase + rng.uniform(-np.pi, np.pi))
      stroke(prox + 2, feather_amp * sign, phase + feather_lead)
    if side < 0:   # lead with the other side: mirror left and right
      idx = C.MIRROR_INDEX
      c, a, b = (C.MIRROR_SIGN * c[idx], C.MIRROR_SIGN[:, None] * a[idx],
                 C.MIRROR_SIGN[:, None] * b[idx])
    period = np.exp(rng.uniform(np.log(0.4), np.log(1.2)))
    out.append(clip_theta(layout, layout.join(
        period, c / half, a / half[:, None], b / half[:, None])))
  return np.array(out)

