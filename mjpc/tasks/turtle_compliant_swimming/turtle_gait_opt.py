#!/usr/bin/env python3
"""Periodic gait trajectory optimization for the turtle design finalists.

  python turtle_gait_opt.py --turtle-sim ../../../../turtle_simulator run xw540_3s
  python turtle_gait_opt.py --turtle-sim ... run ak45_diff_red_thermal --thermal
  python turtle_gait_opt.py --turtle-sim ... export ak45_diff_red_thermal   # for turtle_trajopt

The turtle counterpart of the paddle crab's analysis/gait_opt.py (branch updated_paddle): same
method, the turtle's design variants (design_study.py) and motor envelopes.

A gait is a periodic open-loop joint-target trajectory: per channel a Fourier series in phase,
plus the period T,

  q_j(t) = c_j + sum_k a_jk cos(2 pi k t / T) + b_jk sin(2 pi k t / T)

for 5 channels on the left side (shoulder 1, 2, 3 and hip 1, 2); the right side is the mirror
image (shoulder 1-3 and hip 1 negated, hip 2 equal; from the joint axes), so the gait is
symmetric and the rear flippers can trim pitch. theta = [log T, c (5), a (5 x K), b (5 x K)].
Joint targets go through the design's transmission (ctrl = coef . q, e.g. a differential) and
every motor's torque-speed envelope is applied each step (design_study.Envelope), so the
search never asks a motor for more than it has.

Unlike design_study.py's search (fixed biological waveform, shell attitude locked), the gait is
scored on the free body's limit cycle: rolled out for `warm` strokes, then `measure` strokes
count. Residuals (least squares; 1 = "this much error", scales as in gait_opt.py):
  speed      constant pull towards faster (max speed)
  straight   sideways and vertical drift rate, yaw rate, mean heading
  level      mean shell tilt and its oscillation within the stroke
  cycle      state at the start of the last stroke minus at its end (a limit cycle)
  range      joint targets beyond the joint ranges
  thermal    (--thermal) RMS motor torque above the rated (continuous) torque
An elitist evolution strategy explores, then Levenberg-Marquardt polishes theta with a
forward-difference Jacobian (the fluid forces are smooth; contacts are off). Per-step controls
(iLQR) are not used: they would not stay periodic.

Seeds come from design_study.py's attitude-locked search (its best biological-waveform gait,
Fourier-fit), with the rear hips at zero.
"""
import argparse
import dataclasses
import json
import multiprocessing as mp
import os
import pickle
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import design_study as ds  # noqa: E402

OUT = HERE / "results" / "trajopt"
K_DEFAULT = 5
T_MIN, T_MAX = 0.3, 2.5
CHANNELS = ["shoulderL1", "shoulderL2", "shoulderL3", "hip_L1", "hip_L2"]
# full joint vector (design_study.JOINTS order) from the 5 left channels: (channel, sign)
MIRROR = [(0, 1), (1, 1), (2, 1), (0, -1), (1, -1), (2, -1), (3, 1), (4, 1), (3, -1), (4, 1)]
RANGE = np.array([1.2, 1.6, None, 1.05, 1.05], dtype=object)

# residual scales (as gait_opt.py)
S_SPEED = 0.005    # m/s
S_LATERAL = 0.003  # m/s
S_DEPTH = 0.003    # m/s
S_YAW = 0.02       # rad/s
S_HEADING = 0.1    # rad
S_TILT = 0.1       # rad
S_CYCLE_Q = 0.02   # rad
S_CYCLE_V = 0.01   # m/s
S_CYCLE_W = 0.05   # rad/s
S_RANGE = 0.05     # rad beyond the range
S_THERMAL = 0.05   # fraction of the rated torque
DEFAULT_WEIGHTS = dict(speed=4.0, lateral=1.0, depth=1.0, yaw=1.0, heading=0.3, tilt=0.3, wobble=0.3,
                       cycle=0.3, range=1.0, thermal=1.0)


class Layout:
  def __init__(self, k=K_DEFAULT, nc=len(CHANNELS)):
    self.k, self.nc = k, nc
    self.size = 1 + nc * (1 + 2 * k)

  def split(self, theta):
    nc, k = self.nc, self.k
    c = theta[1:1 + nc]
    a = theta[1 + nc:1 + nc + nc * k].reshape(nc, k)
    b = theta[1 + nc + nc * k:].reshape(nc, k)
    return float(np.exp(theta[0])), c, a, b

  def join(self, period, c, a, b):
    return np.concatenate([[np.log(period)], np.ravel(c), np.ravel(a), np.ravel(b)])


def channels_at(layout, theta, phase):
  """Left-side channel targets (len(phase), 5) at phases in strokes."""
  _, c, a, b = layout.split(theta)
  ks = np.arange(1, layout.k + 1)
  arg = 2 * np.pi * np.outer(phase, ks)  # (n, k)
  return c[None, :] + np.cos(arg) @ a.T + np.sin(arg) @ b.T


def full_targets(ch):
  """(n, 10) joint targets in design_study.JOINTS order from (n, 5) left channels."""
  return np.stack([s * ch[:, i] for i, s in MIRROR], axis=1)


def fit_channels(layout, table, period):
  """theta whose channels fit a joint-space table (N x 10, one stroke) by least squares."""
  n = len(table)
  phase = np.arange(n) / n
  ks = np.arange(1, layout.k + 1)
  basis = np.hstack([np.ones((n, 1)), np.cos(2 * np.pi * np.outer(phase, ks)),
                     np.sin(2 * np.pi * np.outer(phase, ks))])
  left = table[:, [0, 1, 2, 6, 7]]
  coef, *_ = np.linalg.lstsq(basis, left, rcond=None)
  k = layout.k
  return layout.join(period, coef[0], coef[1:1 + k].T, coef[1 + k:].T)


def clip_theta(layout, theta):
  theta = np.array(theta, float)
  theta[..., 0] = np.clip(theta[..., 0], np.log(T_MIN), np.log(T_MAX))
  theta[..., 1:] = np.clip(theta[..., 1:], -2.0, 2.0)
  return theta


class Problem:
  """Evaluates gaits on one design; one instance per process."""

  def __init__(self, design, k=K_DEFAULT, warm=3, measure=2, thermal=False, weights=None, v0=0.1,
               speed=None, energy=0.0):
    self.d = ds.Design(**{**design, "search": tuple(map(tuple, design.get("search", ())))})
    self.design = design
    self.layout = Layout(k)
    self.warm, self.measure = warm, measure
    self.thermal = thermal
    self.weights = dict(DEFAULT_WEIGHTS, **(weights or {}))
    self.v0 = v0
    self.speed = speed    # target speed (cruise); None = maximize speed
    self.energy = energy  # weight of the electrical cost of transport residual
    self.model, self.acts = ds.make_model(self.d, locked=False)
    self.env = ds.Envelope(self.model, self.acts)
    self.cmat = np.array([a["coef"] for a in self.acts])
    self.km = np.array([ds.MOTORS[a["motor"]]["km"] for a in self.acts])
    self.rated = np.array([ds.MOTORS[a["motor"]]["tau_rated"] or np.nan for a in self.acts])
    self.front = np.array([a["name"] not in ds.ACTUATORS[6:] for a in self.acts])
    m = self.model
    self.jq = np.array([m.joint(n).qposadr[0] for n in ds.JOINTS])
    self.jv = np.array([m.joint(n).dofadr[0] for n in ds.JOINTS])
    self.root = m.body(ds.ROOT).id
    self.mass = float(m.body_subtreemass[self.root])
    rng = [1.2, 1.6, self.d.twist_range, 1.05, 1.05]
    self.range = np.array(rng)
    self.active = np.ones(len(CHANNELS), bool)
    if self.d.twist_drive != "motor":
      self.active[2] = False  # passive / coupled twist: no command

  def config(self):
    return dict(design=self.design, k=self.layout.k, warm=self.warm, measure=self.measure,
                thermal=self.thermal, weights=self.weights, v0=self.v0, speed=self.speed, energy=self.energy)

  def fix(self, theta):
    theta = clip_theta(self.layout, theta)
    if not self.active[2]:
      nc, k = self.layout.nc, self.layout.k
      theta[1 + 2] = 0.0
      theta[1 + nc + 2 * k:1 + nc + 3 * k] = 0.0
      theta[1 + nc + nc * k + 2 * k:1 + nc + nc * k + 3 * k] = 0.0
    return theta

  def simulate(self, theta, log=False):
    import mujoco
    lay = self.layout
    theta = self.fix(theta)
    period = lay.split(theta)[0]
    m = self.model
    dt = m.opt.timestep
    n = int(np.ceil((self.warm + self.measure) * period / dt)) + 2
    phase = np.arange(n + 1) * dt / period
    ch = channels_at(lay, theta, phase)
    excess = np.maximum(np.abs(ch) - self.range[None, :], 0.0)[:, self.active]
    ch = np.clip(ch, -self.range, self.range)
    q = full_targets(ch)
    ctrl = q @ self.cmat.T
    data = mujoco.MjData(m)
    # start on the stroke, moving forward at v0, joints at their targets and rates
    data.qvel[1] = -self.v0  # free joint linear velocity, world -y is forward
    qd = np.gradient(q, dt, axis=0)
    for k, n_ in enumerate(ds.JOINTS):
      if not self.active[2] and n_ in ("shoulderL3", "shoulderR3"):
        continue
      data.qpos[self.jq[k]] = q[0, k]
      data.qvel[self.jv[k]] = qd[0, k]
    if self.d.twist_drive == "coupled":
      for j3, j2 in ((2, 1), (5, 4)):
        data.qpos[self.jq[j3]] = self.d.coupling * data.qpos[self.jq[j2]]
    mujoco.mj_forward(m, data)
    nq, nv, nu = m.nq, m.nv, m.nu
    qpos = np.empty((n + 1, nq))
    qvel = np.empty((n + 1, nv))
    force = np.empty((n, nu))
    vel = np.empty((n, nu))
    qpos[0], qvel[0] = data.qpos, data.qvel
    mujoco.set_mjcb_control(self.env)
    try:
      for i in range(n):
        data.ctrl[:] = ctrl[i]
        mujoco.mj_step(m, data)  # the callback clamps ctrl to the envelope at the step's state
        if not np.isfinite(data.qpos[0]):
          return None
        force[i] = data.actuator_force
        vel[i] = data.actuator_velocity
        qpos[i + 1], qvel[i + 1] = data.qpos, data.qvel
    finally:
      mujoco.set_mjcb_control(None)
    return dict(theta=theta, period=period, dt=dt, qpos=qpos, qvel=qvel, force=force, vel=vel,
                excess=excess, ctrl=ctrl, q=q)

  def evaluate(self, thetas):
    res, info = [], []
    for theta in np.atleast_2d(thetas):
      sim = self.simulate(theta)
      if sim is None:
        r = np.full(self.nres(), 1e3)
        res.append(r)
        info.append(dict(speed=np.nan, failed=True, cost=float(0.5 * r @ r)))
        continue
      r, mtr = self.residual(sim)
      res.append(r)
      info.append(mtr)
    return np.array(res), info

  def nres(self):
    if not hasattr(self, "_nres"):
      sim = self.simulate(fit_channels(self.layout, np.zeros((128, 10)), 1.0))
      self._nres = len(self.residual(sim)[0])
    return self._nres

  def residual(self, sim):
    import mujoco
    dt, period = sim["dt"], sim["period"]
    qpos, qvel = sim["qpos"], sim["qvel"]
    t0, t1 = self.warm * period, (self.warm + self.measure) * period
    tl = t1 - period

    def at(x, t):
      f = t / dt
      i = min(int(np.floor(f)), len(x) - 2)
      w = f - i
      return (1 - w) * x[i] + w * x[i + 1]

    i0, i1 = int(np.ceil(t0 / dt)), int(np.floor(t1 / dt))
    win = slice(i0, i1 + 1)
    T = t1 - t0
    # shell frame: rotation matrices of the root body (free joint)
    R = np.empty((len(qpos), 3, 3))
    tmp = np.empty(9)
    for i in range(len(qpos)):
      mujoco.mju_quat2Mat(tmp, qpos[i, 3:7])
      R[i] = tmp.reshape(3, 3)
    # COM is what we report; the free joint origin is a fine proxy for drift rates
    p0, p1 = at(qpos[:, :3], t0), at(qpos[:, :3], t1)
    disp = p1 - p0
    speed = -disp[1] / T
    lateral = disp[0] / T
    depth = disp[2] / T
    fwd = -R[:, :, 1]  # shell -y axis in world
    yaw = np.unwrap(np.arctan2(-fwd[:, 0], fwd[:, 1]) + np.pi)
    yaw -= yaw[0]
    yaw_rate = float(at(yaw, t1) - at(yaw, t0)) / T
    heading = float(np.mean(yaw[win]))
    up = R[win][:, :, 2]  # shell z axis in world
    tilt = up[:, :2]
    tilt_mean = tilt.mean(0)
    tilt_rms = float(np.sqrt(np.mean(np.sum((tilt - tilt_mean) ** 2, -1))))
    lin_body = np.einsum("nji,nj->ni", R, qvel[:, :3])
    ang_body = qvel[:, 3:6]

    def cyc(t):
      return np.concatenate([at(qpos[:, 7:], t) / S_CYCLE_Q,
                             at(qvel[:, 6:], t) * period / (2 * np.pi) / S_CYCLE_Q,
                             at(lin_body, t) / S_CYCLE_V, at(ang_body, t) / S_CYCLE_W,
                             at(R[:, :2, 2], t) / S_CYCLE_Q])
    cycle = cyc(t1) - cyc(tl)

    F = sim["force"][i0:i1]
    W = sim["vel"][i0:i1]
    eta = self.env.eta
    tau = F / eta
    pm = np.where(F * W > 0, F * W / eta, F * W * eta)
    cu = (tau / self.km) ** 2
    mech = float(np.clip(pm, 0, None).sum() * dt)
    elec = float(np.clip(pm + cu, 0, None).sum() * dt)
    rms = np.sqrt(np.mean(tau ** 2, axis=0))
    rr = rms / self.rated
    rr_front = rr[self.front]
    rms_over_rated = float(np.nanmax(rr_front)) if np.isfinite(rr_front).any() else None

    w = self.weights
    r_speed = (1.0 - speed) / S_SPEED * 0.05 if self.speed is None else (speed - self.speed) / S_SPEED
    r = [np.sqrt(w["speed"]) * r_speed,
         np.sqrt(w["lateral"]) * lateral / S_LATERAL,
         np.sqrt(w["depth"]) * depth / S_DEPTH,
         np.sqrt(w["yaw"]) * yaw_rate / S_YAW,
         np.sqrt(w["heading"]) * heading / S_HEADING]
    r += list(np.sqrt(w["tilt"]) * tilt_mean / S_TILT)
    r += [np.sqrt(w["wobble"]) * tilt_rms / S_TILT]
    r += list(np.sqrt(w["cycle"] / cycle.size) * cycle)
    ex = sim["excess"][win]
    r += list(np.sqrt(w["range"]) * np.sqrt(np.mean(ex ** 2, axis=0)) / S_RANGE)
    if self.energy:
      dist_ = max(speed * T, 1e-3)
      r += [np.sqrt(self.energy) * elec / (self.mass * 9.81 * dist_)]  # electrical CoT
    if self.thermal:
      over = np.where(np.isfinite(rr), np.maximum(rr - 1.0, 0.0), 0.0)[self.front]
      r += list(np.sqrt(w["thermal"]) * over / S_THERMAL)
    r = np.array(r)
    dist = max(speed * T, 1e-9)
    names = [a["name"] for a in self.acts]
    metrics = dict(
        period=period, speed=speed, bl_s=speed / ds.BODY_LENGTH, lateral_rate=lateral, depth_rate=depth,
        yaw_rate=yaw_rate, heading=heading, tilt_mean=float(np.degrees(np.arcsin(min(np.linalg.norm(tilt_mean), 1)))),
        tilt_rms_deg=float(np.degrees(tilt_rms)), max_tilt_deg=float(np.degrees(np.arccos(np.clip(up[:, 2], -1, 1)).max())),
        cycle_error=float(np.sqrt(np.mean(cycle ** 2))), mech_power=mech / T, elec_power=elec / T,
        mech_jpm=mech / dist if speed > 0 else float("nan"), elec_jpm=elec / dist if speed > 0 else float("nan"),
        cot_elec=elec / (self.mass * 9.81 * dist) if speed > 0 else float("nan"),
        actuators=names, tau_peak=np.abs(tau).max(0).tolist(), tau_rms=rms.tolist(),
        w_peak=np.abs(W).max(0).tolist(), rms_over_rated=rms_over_rated, mass=self.mass,
        range_excess=float(ex.max()) if ex.size else 0.0, cost=float(0.5 * r @ r))
    return r, metrics


# ----- parallel evaluation (as gait_opt.py) -----
_PROBLEM = None


def _init(config):
  global _PROBLEM
  _PROBLEM = Problem(**config)


def _evaluate(thetas):
  return _PROBLEM.evaluate(thetas)


class Evaluator:
  def __init__(self, problem, processes=None):
    self.processes = processes or os.cpu_count()
    self.pool = mp.get_context("fork").Pool(self.processes, _init, (problem.config(),))

  def __call__(self, thetas):
    thetas = np.atleast_2d(thetas)
    n = max(1, int(np.ceil(len(thetas) / (self.processes * 2))))
    chunks = [thetas[i:i + n] for i in range(0, len(thetas), n)]
    out = self.pool.map(_evaluate, chunks)
    return np.concatenate([o[0] for o in out]), [i for o in out for i in o[1]]

  def close(self):
    self.pool.close()
    self.pool.join()


# ----- optimizers (gait_opt.py's es and lm, with checkpoints) -----
def _load(path):
  if path and os.path.exists(path):
    with open(path, "rb") as f:
      return pickle.load(f)
  return None


def _save(path, state):
  if path:
    with open(path + ".tmp", "wb") as f:
      pickle.dump(state, f)
    os.replace(path + ".tmp", path)


def es(evaluate, layout, base, iters=60, pop=32, sigma=0.05, sigma_period=0.05, sigma_min=0.005, seed=0,
       checkpoint=None, log=print):
  """Elitist evolution strategy around the best gait so far, 1/5 success rule on sigma."""
  rng = np.random.default_rng(seed)
  base = clip_theta(layout, base)
  scale = np.ones(layout.size)
  scale[0] = sigma_period / sigma
  saved = _load(checkpoint)
  if saved:
    base, best, binfo, mom, sigma, start, hist = (saved[k] for k in ("base", "cost", "info", "mom", "sigma", "it", "hist"))
    rng.bit_generator.state = saved["rng"]
  else:
    res, info = evaluate(base[None])
    best, binfo = 0.5 * res[0] @ res[0], info[0]
    mom, start, hist = np.zeros(layout.size), 0, []
  for it in range(start, iters):
    samples = base + sigma * scale * rng.standard_normal((pop, layout.size))
    samples[0] = base + mom
    samples = clip_theta(layout, samples)
    res, info = evaluate(samples)
    cost = 0.5 * np.sum(res ** 2, axis=1)
    i = int(np.argmin(cost))
    success = np.mean(cost < best)
    if cost[i] < best:
      mom = samples[i] - base
      base, best, binfo = samples[i], cost[i], info[i]
    else:
      mom = np.zeros(layout.size)
    sigma = max(sigma_min, sigma * (1.2 if success > 0.2 else 0.85))
    hist.append(dict(iter=it, cost=float(best), sigma=sigma, speed=binfo.get("speed")))
    _save(checkpoint, dict(base=base, cost=best, info=binfo, mom=mom, sigma=sigma, it=it + 1, hist=hist,
                           rng=rng.bit_generator.state))
    if log and (it % 5 == 0 or it == iters - 1):
      log(f"  es {it:3d} cost {best:9.2f} v {binfo['speed']:+.4f} T {binfo['period']:.3f} "
          f"tilt {binfo['tilt_mean']:.1f}/{binfo['max_tilt_deg']:.0f} deg yaw {binfo['yaw_rate']:+.3f} "
          f"depth {binfo['depth_rate']:+.4f} cyc {binfo['cycle_error']:.2f} rms/rated {binfo['rms_over_rated']} "
          f"sigma {sigma:.3f}", flush=True)
  return base, best, binfo, hist


def lm(evaluate, layout, theta, iters=15, eps=1e-4, mu=1e-2, max_step=0.3, checkpoint=None, log=print):
  """Levenberg-Marquardt on the residuals; the n + 1 Jacobian rollouts go out as one batch."""
  theta = clip_theta(layout, theta)
  n = layout.size
  saved = _load(checkpoint)
  if saved:
    theta, cost, info, mu, start, hist = (saved[k] for k in ("theta", "cost", "info", "mu", "it", "hist"))
    if saved["done"]:
      return theta, cost, info, hist
  else:
    res, inf = evaluate(theta[None])
    cost, info = 0.5 * res[0] @ res[0], inf[0]
    hist, start = [dict(iter=0, cost=float(cost), mu=mu, speed=info.get("speed"))], 1
  accepted = True
  for it in range(start, iters + 1):
    res_p, inf_p = evaluate(np.vstack([theta[None], theta + eps * np.eye(n)]))
    r = res_p[0]
    cost, info = 0.5 * r @ r, inf_p[0]
    J = (res_p[1:] - r).T / eps
    g, H = J.T @ r, J.T @ J
    accepted = False
    while mu < 1e6:
      step = -np.linalg.solve(H + mu * np.diag(np.diag(H) + 1e-6), g)
      norm = np.max(np.abs(step))
      if norm > max_step:
        step *= max_step / norm
      trial = clip_theta(layout, theta + step)
      res_t, inf_t = evaluate(trial[None])
      ct = 0.5 * res_t[0] @ res_t[0]
      if ct < cost:
        theta, cost, info = trial, ct, inf_t[0]
        mu = max(mu / 3, 1e-6)
        accepted = True
        break
      mu *= 10
    hist.append(dict(iter=it, cost=float(cost), mu=mu, speed=info.get("speed")))
    if log:
      log(f"  lm {it:3d} cost {cost:9.3f} mu {mu:7.1e} v {info['speed']:+.4f} T {info['period']:.3f} "
          f"tilt {info['tilt_mean']:.1f}/{info['max_tilt_deg']:.0f} deg yaw {info['yaw_rate']:+.3f} "
          f"cyc {info['cycle_error']:.3f} rms/rated {info['rms_over_rated']}", flush=True)
    _save(checkpoint, dict(theta=theta, cost=cost, info=info, mu=mu, it=it + 1, hist=hist,
                           done=not accepted or it == iters))
    if not accepted:
      break
  return theta, cost, info, hist


# ----- driver -----
def seed_theta(turtle_sim, rec, layout):
  x = rec["best"]["x"]
  d = rec["design"]
  twist = x.get("twist", 0.0) if d["twist_drive"] == "motor" else 0.0
  tab = ds.gait(turtle_sim, x["yaw"], x["roll"], twist, x.get("shift", 0.0))
  period = ds.gs.BASE_PERIOD / x["cadence"]
  return fit_channels(layout, tab, period)


def design_of(rec):
  d = dict(rec["design"])
  for k, v in rec["best"]["x"].items():  # optimized design knobs (gear ratios, ...)
    if k in d:
      d[k] = v
  d["search"] = []
  return d


def run(args):
  rec = json.loads((ds.OUT / f"{args.name}.json").read_text())
  design = design_of(rec)
  OUT.mkdir(parents=True, exist_ok=True)
  tag = args.name + (args.tag or ("_thermal" if args.thermal and "_thermal" not in args.name else "")
                     + (f"_cruise{args.speed:g}" if args.speed else ""))
  prob = Problem(design, k=args.k, warm=args.warm, measure=args.measure, thermal=args.thermal,
                 v0=rec["best"]["speed"], speed=args.speed, energy=args.energy)
  lay = prob.layout
  theta0 = seed_theta(args.turtle_sim, rec, lay)
  ev = Evaluator(prob, args.processes)
  try:
    res, info = ev(theta0[None])
    print(f"{tag}: seed (locked-search gait, free body, open loop): v {info[0]['speed']:+.4f} m/s, "
          f"max tilt {info[0]['max_tilt_deg']:.0f} deg, cost {info[0]['cost']:.1f}", flush=True)
    theta, cost, binfo, h_es = es(ev, lay, theta0, iters=args.es_iters, pop=args.pop,
                                  checkpoint=str(OUT / f"{tag}_es.pkl"))
    theta, cost, binfo, h_lm = lm(ev, lay, theta, iters=args.lm_iters, checkpoint=str(OUT / f"{tag}_lm.pkl"))
  finally:
    ev.close()
  out = dict(name=args.name, design=design, thermal=args.thermal, speed_target=args.speed, energy=args.energy, k=args.k, warm=args.warm, measure=args.measure,
             theta=theta.tolist(), metrics=binfo, seed_metrics=info[0], es=h_es, lm=h_lm,
             locked_search=dict(speed=rec["best"]["speed"], x=rec["best"]["x"]))
  (OUT / f"{tag}.json").write_text(json.dumps(out, indent=1))
  m = binfo
  print(f"{tag}: {m['speed']:.4f} m/s ({m['bl_s']:.2f} BL/s), stroke {m['period']:.3f} s, max tilt "
        f"{m['max_tilt_deg']:.0f} deg, yaw rate {m['yaw_rate']:+.3f}, elec {m['elec_power']:.1f} W, "
        f"{m['elec_jpm']:.0f} J/m, rms/rated {m['rms_over_rated']}")


def export(args):
  rec = json.loads((OUT / f"{args.name}.json").read_text())
  lay = Layout(rec["k"])
  theta = np.array(rec["theta"])
  period = lay.split(theta)[0]
  n = ds.gs.N
  q = full_targets(channels_at(lay, theta, np.arange(n) / n))
  d = ds.Design(**{**rec["design"], "search": ()})
  cadence = ds.gs.BASE_PERIOD / period
  outdir = ds.DESIGNS_DIR / f"{args.name}_trajopt"
  ds.export_table(d, q, cadence, outdir, note=f"turtle_gait_opt.py result {args.name} (K={rec['k']})")
  print(f"wrote {outdir}/; run: turtle_trajopt --task_xml={outdir}/task.xml --param=Cadence={cadence:.4f}")


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--turtle-sim", required=True)
  sub = ap.add_subparsers(dest="cmd", required=True)
  r = sub.add_parser("run")
  r.add_argument("name", help="a design_study.py search result (results/design_study/<name>.json)")
  r.add_argument("--thermal", action="store_true")
  r.add_argument("--k", type=int, default=K_DEFAULT)
  r.add_argument("--warm", type=int, default=3)
  r.add_argument("--measure", type=int, default=2)
  r.add_argument("--es-iters", type=int, default=60)
  r.add_argument("--pop", type=int, default=32)
  r.add_argument("--lm-iters", type=int, default=15)
  r.add_argument("--processes", type=int, default=None)
  r.add_argument("--tag", default="")
  r.add_argument("--speed", type=float, default=None,
                 help="cruise: hold this speed (m/s) instead of maximizing speed; use with --energy")
  r.add_argument("--energy", type=float, default=0.0,
                 help="weight of the electrical cost-of-transport residual (e.g. 10)")
  e = sub.add_parser("export")
  e.add_argument("name", help="a result in results/trajopt/")
  args = ap.parse_args()
  {"run": run, "export": export}[args.cmd](args)


if __name__ == "__main__":
  main()
