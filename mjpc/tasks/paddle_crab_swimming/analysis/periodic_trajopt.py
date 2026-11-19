#!/usr/bin/env python3
"""Periodic multiple-shooting trajectory optimization of a swimming gait
(robot v2, MuJoCo >= 4, IPOPT through cyipopt).

  periodic_trajopt.py --init runs/v2_mY/step6_v0.1458_dt0.001.npz \
      --out runs/tj_mY [--nodes 54 --substeps 10 --dt 0.002]

One stroke is split into N shooting intervals of K simulation steps. The
decision variables are the state at the start of every interval and one
servo command per interval boundary (the command is interpolated linearly
inside an interval and wraps around, so it is periodic by construction):

  z = [x_0 .. x_{N-1}, p_0 .. p_{N-1}]

  max  speed = d . (end_{N-1} - x_0)[:3] / T  -  smooth * sum |p_{i+1} - p_i|^2
  s.t. x_{i+1} = Phi(x_i, p_i, p_{i+1})          (continuity, i < N-1)
       x_0 = Phi(x_{N-1}, p_{N-1}, p_0)          (periodicity, all but the
                                                  body position)
       lateral and vertical displacement = 0     (swims straight along d)
       command range                             (bounds on p)

Phi is K MuJoCo steps (mujoco.rollout). Its Jacobians come from forward
differences of whole intervals: one batched, threaded rollout of
N x (1 + nx + 2 nu) intervals per IPOPT iteration. IPOPT uses its
limited-memory (L-BFGS) Hessian.

The design follows Direct Simulation-based Multiple Shooting (Esteban et al.,
"Shooting for Contact", 2026): MuJoCo in the loop, defect constraints,
IPOPT. The code is written from scratch for this problem (that repository
carries no licence).

The free joint is replaced by three slide and three hinge joints (yaw about
z, then pitch about the body x, then roll about the body y), so the state is
a plain vector and the defects are plain differences. The dynamics are the
same (checked against the free-joint model in validate()).

The robot v2 dcmotor servos have no internal state, so x = (qpos, qvel),
nx = 2 nv = 48.
"""
import argparse
import json
import os
import tempfile
import time

import cyipopt
import mujoco
import mujoco.rollout
import numpy as np

import crab_model as C
import gait_opt as G

BASE_JOINTS = ('<joint name="base_x" type="slide" axis="1 0 0" damping="0" '
               'armature="0" limited="false"/>'
               '<joint name="base_y" type="slide" axis="0 1 0" damping="0" '
               'armature="0" limited="false"/>'
               '<joint name="base_z" type="slide" axis="0 0 1" damping="0" '
               'armature="0" limited="false"/>'
               '<joint name="base_yaw" type="hinge" axis="0 0 1" damping="0" '
               'armature="0" limited="false"/>'
               '<joint name="base_pitch" type="hinge" axis="1 0 0" '
               'damping="0" armature="0" limited="false"/>'
               '<joint name="base_roll" type="hinge" axis="0 1 0" damping="0" '
               'armature="0" limited="false"/>')
S_SPEED = 0.01      # m/s: objective units


# ----- Models ----- #

def load_euler_model(dt, design=None):
  """Robot v2 with the free joint replaced by x, y, z, yaw, pitch, roll."""
  with open(C.V2_XML) as f:
    xml = f.read()
  assert xml.count('<freejoint />') == 1
  xml = xml.replace('<freejoint />', BASE_JOINTS)
  fd, path = tempfile.mkstemp(suffix='.xml', prefix='.crab_euler_',
                              dir=C.V2_DIR)
  saved = C.V2_XML
  try:
    with os.fdopen(fd, 'w') as f:
      f.write(xml)
    C.V2_XML = path
    m = C.load_v2(dt=dt, ground=False, design=design)
  finally:
    C.V2_XML = saved
    os.remove(path)
  # load_v2 zeroes the damping of dofs 6: (the legs); the base has none.
  m.dof_damping[:6] = 0.0
  m.dof_armature[:6] = 0.0
  m.dof_frictionloss[:6] = 0.0
  # No command clamp in the optimizer's model: the command range is IPOPT's
  # bounds on p instead. MuJoCo's clamp makes the dynamics kinked at the
  # range (a forward difference at a bound reads zero slope), which stalled
  # the solves. Within the bounds the dynamics are the same.
  m.actuator_ctrllimited[:] = 0
  assert m.nq == m.nv == 6 + C.NU and m.na == 0
  return m


def smooth_servos(m):
  """Replace the dcmotor servos by their unsaturated law, in place.

  The v2 dcmotor (MuJoCo 4.0.1, checked numerically) applies
    tau = clip(kp (u - q) - kd qd, lo, hi)
    hi = min(tmax, vlim - c qd),  lo = max(-tmax, -vlim - c qd)
  with kp, kd its controller gains, tmax its force range, vlim = (K/R) Vmax
  the stall torque and c = K^2 / R the back-EMF slope. The clip is a kink
  that Newton steps oscillate across, so the optimizer's model drops it
  (an affine actuator, identical to the dcmotor inside its limits) and the
  limits become linear inequalities on each node's command and state
  (PeriodicShooting servo=...). Returns the servo parameters."""
  R, K = m.actuator_gainprm[:, 0].copy(), m.actuator_gainprm[:, 1].copy()
  kp, kd = m.actuator_gainprm[:, 4].copy(), m.actuator_gainprm[:, 6].copy()
  vmax = m.actuator_gainprm[:, 7].copy()
  servo = dict(kp=kp, kd=kd, tmax=m.actuator_forcerange[:, 1].copy(),
               vlim=K / R * vmax, c=K * K / R)
  m.actuator_gaintype[:] = mujoco.mjtGain.mjGAIN_FIXED
  m.actuator_biastype[:] = mujoco.mjtBias.mjBIAS_AFFINE
  m.actuator_dyntype[:] = mujoco.mjtDyn.mjDYN_NONE
  m.actuator_gainprm[:] = 0
  m.actuator_biasprm[:] = 0
  m.actuator_dynprm[:] = 0
  m.actuator_gainprm[:, 0] = kp
  m.actuator_biasprm[:, 1] = -kp
  m.actuator_biasprm[:, 2] = -kd
  m.actuator_forcelimited[:] = 0
  return servo


def load_free_model(dt, design=None):
  return C.load_v2(dt=dt, ground=False, design=design)


def euler_from_mat(R):
  """R = Rz(yaw) Rx(pitch) Ry(roll) -> (yaw, pitch, roll)."""
  pitch = np.arcsin(np.clip(R[2, 1], -1, 1))
  roll = np.arctan2(-R[2, 0], R[2, 2])
  yaw = np.arctan2(-R[0, 1], R[1, 1])
  return np.array([yaw, pitch, roll])


def _base_velocity_map(m_eul, d, qpos):
  """6x6 G with [ang_world; lin_world] of the plate = G @ base qvel."""
  plate = mujoco.mj_name2id(m_eul, mujoco.mjtObj.mjOBJ_BODY, 'plate')
  G6 = np.zeros((6, 6))
  for k in range(6):
    d.qpos[:] = qpos
    d.qvel[:] = 0
    d.qvel[k] = 1.0
    mujoco.mj_forward(m_eul, d)
    v = np.zeros(6)
    mujoco.mj_objectVelocity(m_eul, d, mujoco.mjtObj.mjOBJ_BODY, plate, v, 0)
    G6[:, k] = v
  return G6


def free_to_euler(m_eul, qpos_free, qvel_free):
  """Free-joint (qpos 25, qvel 24) -> Euler-base state x (48)."""
  d = mujoco.MjData(m_eul)
  R = C.quat_to_mat(np.asarray(qpos_free[3:7]))
  q = np.concatenate([qpos_free[:3], euler_from_mat(R), qpos_free[7:]])
  # free qvel: world linear velocity of the body origin, body-frame angular
  target = np.concatenate([R @ qvel_free[3:6], qvel_free[:3]])
  G6 = _base_velocity_map(m_eul, d, q)
  v = np.concatenate([np.linalg.solve(G6, target), qvel_free[6:]])
  return np.concatenate([q, v])


def euler_to_free(m_eul, x):
  """Euler-base state x (48) -> free-joint (qpos 25, qvel 24)."""
  nv = m_eul.nv
  q, v = x[:nv], x[nv:]
  d = mujoco.MjData(m_eul)
  d.qpos[:] = q
  mujoco.mj_kinematics(m_eul, d)
  plate = mujoco.mj_name2id(m_eul, mujoco.mjtObj.mjOBJ_BODY, 'plate')
  quat = d.xquat[plate].copy()
  R = d.xmat[plate].reshape(3, 3).copy()
  G6 = _base_velocity_map(m_eul, d, q)
  w = G6 @ v[:6]
  qpos = np.concatenate([q[:3], quat, q[6:]])
  qvel = np.concatenate([w[3:], R.T @ w[:3], v[6:]])
  return qpos, qvel


def full_state(m, qpos, qvel):
  d = mujoco.MjData(m)
  d.qpos[:], d.qvel[:] = qpos, qvel
  s = np.empty(mujoco.mj_stateSize(m, C.STATE_SPEC))
  mujoco.mj_getState(m, d, s, C.STATE_SPEC)
  return s


# ----- The NLP ----- #

class PeriodicShooting:

  def __init__(self, m, nodes, substeps, direction, smooth=1e-2,
               nthread=4, eps=1e-6, speed_target=None, prox=0.0,
               z_ref=None, servo=None, limit_margin=0.0):
    self.m = m
    self.N, self.K = nodes, substeps
    self.dt = m.opt.timestep
    self.T = self.N * self.K * self.dt
    self.nv, self.nu = m.nv, m.nu
    self.nx = 2 * m.nv
    self.direction = np.asarray(direction, float) / np.linalg.norm(direction)
    lat, vert = G.drift_axes(self.direction)
    self.drift = np.array([lat, vert])
    self.smooth = smooth
    self.eps = eps
    # With a speed target the speed is a constraint (speed >= target) and the
    # objective is smoothness plus a pull (prox) toward z_ref (the previous
    # solution): a ladder of targets instead of an unbounded maximization.
    self.speed_target = speed_target
    self.prox = prox
    self.z_ref = z_ref
    self.datas = [mujoco.MjData(m) for _ in range(nthread)]
    self.nstate = mujoco.mj_stateSize(m, C.STATE_SPEC)
    self.lo = m.actuator_ctrlrange[:, 0].copy()
    self.hi = m.actuator_ctrlrange[:, 1].copy()
    N, nx, nu = self.N, self.nx, self.nu
    self.nX = N * nx
    self.n = self.nX + N * nu
    # closure rows: every state component but the base position
    self.closed = np.arange(3, nx)
    self.m_rows = (N - 1) * nx + len(self.closed) + 2
    self.n_eq = self.m_rows
    self.row_speed = None
    if speed_target is not None:
      self.row_speed = self.m_rows
      self.m_rows += 1
    # Servo limits (smooth_servos) as constant linear rows at every node:
    #   tau      = kp (p - q) - kd qd    in [-tmax, tmax]
    #   tau + c qd                       in [-vlim, vlim]
    # (tightened by limit_margin, a fraction, for the steps between nodes).
    self.servo = servo
    self.n_lin = 0
    if servo is not None:
      self._build_limits(servo, limit_margin)
      self.row_lin = self.m_rows
      self.m_rows += self.n_lin
    self._build_structure()
    self._cache_z = None
    self._cache_fd = None
    self.evals = 0
    self.fd_time = 0.0
    self.feas_tol = 1e-4
    self._best_obj, self._best_z = np.inf, None

  # z layout
  def X(self, z):
    return z[:self.nX].reshape(self.N, self.nx)

  def P(self, z):
    return z[self.nX:].reshape(self.N, self.nu)

  def xcol(self, i):
    return i * self.nx

  def pcol(self, i):
    return self.nX + (i % self.N) * self.nu

  def interval_ctrl(self, p0, p1):
    """(..., K, nu) commands over an interval from boundary commands."""
    w = (np.arange(self.K) / self.K)[:, None]
    return (1 - w) * p0[..., None, :] + w * p1[..., None, :]

  def _states(self, x):
    """Full-physics initial states (n, nstate) from x (n, nx)."""
    s = np.zeros((len(x), self.nstate))
    s[:, 1:1 + self.nv] = x[:, :self.nv]
    s[:, 1 + self.nv:1 + 2 * self.nv] = x[:, self.nv:]
    return s

  def _ends(self, x, ctrl):
    """End states (n, nx) of intervals from x (n, nx), ctrl (n, K, nu)."""
    state, _ = mujoco.rollout.rollout(self.m, self.datas, self._states(x),
                                      ctrl, persistent_pool=True)
    last = state[:, -1]
    return np.concatenate([last[:, 1:1 + self.nv],
                           last[:, 1 + self.nv:1 + 2 * self.nv]], axis=1)

  def ends(self, z):
    X, P = self.X(z), self.P(z)
    ctrl = self.interval_ctrl(P, np.roll(P, -1, axis=0))
    return self._ends(X, ctrl)

  def fd(self, z):
    """Ends E (N, nx) and Jacobians dE/dx (N, nx, nx), dE/dp0, dE/dp1
    (N, nx, nu) by forward differences, cached on z."""
    if self._cache_z is not None and np.array_equal(z, self._cache_z):
      return self._cache_fd
    t0 = time.time()
    N, nx, nu, eps = self.N, self.nx, self.nu, self.eps
    X, P = self.X(z), self.P(z)
    P1 = np.roll(P, -1, axis=0)
    nb = 1 + nx + 2 * nu
    xs = np.repeat(X[:, None], nb, axis=1)           # (N, nb, nx)
    p0 = np.repeat(P[:, None], nb, axis=1)           # (N, nb, nu)
    p1 = np.repeat(P1[:, None], nb, axis=1)
    idx = np.arange(nx)
    xs[:, 1 + idx, idx] += eps
    idu = np.arange(nu)
    p0[:, 1 + nx + idu, idu] += eps
    p1[:, 1 + nx + nu + idu, idu] += eps
    ctrl = self.interval_ctrl(p0, p1)                # (N, nb, K, nu)
    E = self._ends(xs.reshape(-1, nx),
                   ctrl.reshape(-1, self.K, nu)).reshape(N, nb, nx)
    base = E[:, 0]
    D = (E[:, 1:] - base[:, None]) / eps             # (N, nx+2nu, nx)
    Ax = np.transpose(D[:, :nx], (0, 2, 1))
    B0 = np.transpose(D[:, nx:nx + nu], (0, 2, 1))
    B1 = np.transpose(D[:, nx + nu:], (0, 2, 1))
    self._cache_z = z.copy()
    self._cache_fd = (base, Ax, B0, B1)
    self.fd_time += time.time() - t0
    self.evals += 1
    return self._cache_fd

  def _ends_cached(self, z):
    if self._cache_z is not None and np.array_equal(z, self._cache_z):
      return self._cache_fd[0]
    return self.ends(z)

  # --- objective --- #

  def speed(self, z, E=None):
    E = self._ends_cached(z) if E is None else E
    return float(self.direction @ (E[-1, :3] - self.X(z)[0, :3])) / self.T

  def _prox_w(self):
    """Per-variable weights of the pull toward z_ref (state rows scaled to
    rad, rad/s; commands rad)."""
    w = np.ones(self.n)
    X = w[:self.nX].reshape(self.N, self.nx)
    X[:, self.nv:] = 0.1             # velocities: 1 rad/s counts as 0.1 rad
    return w

  def objective(self, z, E=None):
    P = self.P(z)
    dP = np.roll(P, -1, axis=0) - P
    f = self.smooth * float(np.sum(dP ** 2))
    if self.speed_target is None:
      f -= self.speed(z, E) / S_SPEED
    if self.prox:
      f += self.prox * float(np.sum(self._prox_w() * (z - self.z_ref) ** 2))
    return f

  def gradient(self, z):
    N = self.N
    g = np.zeros(self.n)
    if self.speed_target is None:
      E, Ax, B0, B1 = self.fd(z)
      c = -self.direction / (self.T * S_SPEED)       # d obj / d E_{N-1}[:3]
      g[self.xcol(N - 1):self.xcol(N - 1) + self.nx] += c @ Ax[-1, :3]
      g[self.pcol(N - 1):self.pcol(N - 1) + self.nu] += c @ B0[-1, :3]
      g[self.pcol(0):self.pcol(0) + self.nu] += c @ B1[-1, :3]
      g[0:3] -= c
    if self.prox:
      g += 2 * self.prox * self._prox_w() * (z - self.z_ref)
    P = self.P(z)
    dP = np.roll(P, -1, axis=0) - P
    gp = 2 * self.smooth * (np.roll(dP, 1, axis=0) - dP)
    g[self.nX:] += gp.ravel()
    return g

  # --- constraints --- #

  def constraints(self, z):
    X = self.X(z)
    E = self._ends_cached(z)
    defects = X[1:] - E[:-1]
    closure = X[0, self.closed] - E[-1, self.closed]
    straight = self.drift @ (E[-1, :3] - X[0, :3])
    out = [defects.ravel(), closure, straight]
    if self.speed_target is not None:
      out.append([self.speed(z, E) / S_SPEED])
    if self.n_lin:
      out.append(self.limit_values(z))
    out = np.concatenate(out)
    # keep-best (as in shooting-for-contact): the feasible point with the
    # lowest objective among those IPOPT evaluates, returned if it stops
    # without converging (IPOPT 3.11 cannot hand back its iterate).
    viol = float(np.abs(out[:self.n_eq]).max())
    if self.n_lin:
      lv = out[self.row_lin:]
      viol = max(viol, float(np.max(np.maximum(self.lin_lo - lv,
                                               lv - self.lin_hi))))
    if self.speed_target is not None:
      viol = max(viol, self.speed_target / S_SPEED - out[self.row_speed])
    if viol < self.feas_tol:
      f = self.objective(z, E)
      if f < self._best_obj:
        self._best_obj, self._best_z = f, z.copy()
    return out

  def _build_limits(self, servo, margin):
    N, nx, nu, nv = self.N, self.nx, self.nu, self.nv
    rows, cols, vals, lo, hi = [], [], [], [], []
    r = 0
    for i in range(N):
      for a in range(nu):
        q = self.xcol(i) + 6 + a
        qd = self.xcol(i) + nv + 6 + a
        p = self.pcol(i) + a
        for slope, lim in ((0.0, servo['tmax'][a]),
                           (servo['c'][a], servo['vlim'][a])):
          rows += [r, r, r]
          cols += [p, q, qd]
          vals += [servo['kp'][a], -servo['kp'][a],
                   -servo['kd'][a] + slope]
          lo.append(-lim * (1 - margin))
          hi.append(lim * (1 - margin))
          r += 1
    self.n_lin = r
    self.lin_rows = np.array(rows)
    self.lin_cols = np.array(cols)
    self.lin_vals = np.array(vals)
    self.lin_lo, self.lin_hi = np.array(lo), np.array(hi)

  def limit_values(self, z):
    out = np.zeros(self.n_lin)
    np.add.at(out, self.lin_rows, self.lin_vals * z[self.lin_cols])
    return out

  def _build_structure(self):
    N, nx, nu = self.N, self.nx, self.nu
    rows, cols, self._blocks = [], [], []

    def block(r0, nr, c0, nc):
      self._blocks.append((nr, nc))
      rows.append(np.repeat(np.arange(r0, r0 + nr), nc))
      cols.append(np.tile(np.arange(c0, c0 + nc), nr))

    for i in range(N - 1):
      r = i * nx
      block(r, nx, self.xcol(i), nx)          # -dE_i/dx_i
      block(r, nx, self.xcol(i + 1), nx)      # I (dense block, identity)
      block(r, nx, self.pcol(i), nu)          # -dE_i/dp_i
      block(r, nx, self.pcol(i + 1), nu)      # -dE_i/dp_{i+1}
    r = (N - 1) * nx
    nc = len(self.closed)
    block(r, nc, self.xcol(N - 1), nx)
    block(r, nc, self.xcol(0), nx)            # selection (dense, mostly 0)
    block(r, nc, self.pcol(N - 1), nu)
    block(r, nc, self.pcol(0), nu)
    r += nc
    block(r, 2, self.xcol(N - 1), nx)
    block(r, 2, self.xcol(0), nx)
    block(r, 2, self.pcol(N - 1), nu)
    block(r, 2, self.pcol(0), nu)
    if self.speed_target is not None:
      r += 2
      block(r, 1, self.xcol(N - 1), nx)
      block(r, 1, self.xcol(0), nx)
      block(r, 1, self.pcol(N - 1), nu)
      block(r, 1, self.pcol(0), nu)
    self._rows = np.concatenate(rows)
    self._cols = np.concatenate(cols)
    if self.n_lin:
      self._rows = np.concatenate([self._rows, self.row_lin + self.lin_rows])
      self._cols = np.concatenate([self._cols, self.lin_cols])
    # closure rows hit both p_{N-1} and p_0; with N > 2 these never overlap.
    assert N > 2

  def jacobianstructure(self):
    return self._rows, self._cols

  def jacobian(self, z):
    E, Ax, B0, B1 = self.fd(z)
    N, nx = self.N, self.nx
    I = np.eye(nx)
    vals = []
    for i in range(N - 1):
      vals += [-Ax[i], I, -B0[i], -B1[i]]
    sel = I[self.closed]
    vals += [-Ax[-1][self.closed], sel, -B0[-1][self.closed],
             -B1[-1][self.closed]]
    S = np.zeros((2, nx))
    S[:, :3] = self.drift
    vals += [self.drift @ Ax[-1, :3], -S, self.drift @ B0[-1, :3],
             self.drift @ B1[-1, :3]]
    if self.speed_target is not None:
      d = self.direction[None] / (self.T * S_SPEED)
      Sd = np.zeros((1, nx))
      Sd[0, :3] = d
      vals += [d @ Ax[-1, :3], -Sd, d @ B0[-1, :3], d @ B1[-1, :3]]
    if self.n_lin:
      vals.append(self.lin_vals)
    return np.concatenate([v.ravel() for v in vals])

  # --- Hessian (constrained Gauss-Newton: the objective's exact Hessian,
  # the constraints' curvature left out) --- #

  def hessianstructure(self):
    n, N, nu = self.n, self.N, self.nu
    r = [np.arange(n)]
    c = [np.arange(n)]
    for i in range(N):
      a = self.nX + i * nu + np.arange(nu)
      b = self.nX + ((i + 1) % N) * nu + np.arange(nu)
      r.append(np.maximum(a, b))
      c.append(np.minimum(a, b))
    return np.concatenate(r), np.concatenate(c)

  def hessian(self, z, lagrange, obj_factor):
    n, N, nu = self.n, self.N, self.nu
    diag = np.zeros(n)
    if self.prox:
      diag += 2 * self.prox * self._prox_w()
    diag[self.nX:] += 4 * self.smooth
    off = np.full(N * nu, -2 * self.smooth)
    return obj_factor * np.concatenate([diag, off])

  def intermediate(self, alg_mod, iter_count, obj_value, inf_pr, inf_du, mu,
                   d_norm, regularization_size, alpha_du, alpha_pr,
                   ls_trials):
    if iter_count % 10 == 0:
      print(f'  iter {iter_count:4d} obj {obj_value:9.4f} inf_pr {inf_pr:.2e} '
            f'inf_du {inf_du:.2e} | {self.evals} FD passes, '
            f'{self.fd_time:.0f} s', flush=True)
    return True

  # --- solve --- #

  def bounds(self, z0):
    lb = np.full(self.n, -np.inf)
    ub = np.full(self.n, np.inf)
    # gauge: the start position is fixed at the warm start's
    lb[0:3] = ub[0:3] = z0[0:3]
    lb[self.nX:] = np.tile(self.lo, self.N)
    ub[self.nX:] = np.tile(self.hi, self.N)
    return lb, ub

  def solve(self, z0, max_iter=300, tol=1e-6, options=None, hessian='lbfgs'):
    lb, ub = self.bounds(z0)
    z0 = np.clip(z0, lb, ub)
    cl = np.zeros(self.m_rows)
    cu = np.zeros(self.m_rows)
    if self.speed_target is not None:
      cl[self.row_speed] = self.speed_target / S_SPEED
      cu[self.row_speed] = 1e19
    if self.n_lin:
      cl[self.row_lin:] = self.lin_lo
      cu[self.row_lin:] = self.lin_hi
    nlp = cyipopt.Problem(n=self.n, m=self.m_rows, problem_obj=self,
                          lb=lb, ub=ub, cl=cl, cu=cu)
    opts = dict(hessian_approximation='limited-memory' if hessian == 'lbfgs'
                else 'exact', max_iter=max_iter,
                acceptable_constr_viol_tol=1e-4, acceptable_iter=5,
                acceptable_dual_inf_tol=1e-2,
                tol=tol, constr_viol_tol=1e-7, acceptable_tol=1e-4,
                print_level=3, mu_strategy='adaptive',
                limited_memory_max_history=20)
    opts.update(options or {})
    for k, v in opts.items():
      nlp.add_option(k, v)
    self._best_obj, self._best_z = np.inf, None
    z, info = nlp.solve(z0)
    if info['status'] not in (0, 1) and self._best_z is not None:
      print(f'  IPOPT did not converge; returning the best feasible iterate '
            f'(obj {self._best_obj:.4f})', flush=True)
      z = self._best_z
      info = dict(info, status=1, status_msg=b'kept best feasible iterate')
    return z, info


# ----- Warm start and validation ----- #

def gait_from_npz(path):
  z = np.load(path, allow_pickle=True)
  layout = G.Layout(int(z['k']))
  return dict(theta=z['theta'], layout=layout, half=z['half'],
              state=z['state'], period=layout.period(z['theta']),
              metrics=json.loads(str(z['metrics'])))


def fourier_ctrl(gait, n, dt, period, phase0=0.0):
  theta = gait['theta'].copy()
  theta[0] = np.log(period)
  u, _ = G.controls(gait['layout'], theta, gait['half'], n, dt, phase0)
  return u


def warm_start(prob, gait, cycles=4):
  """Nodes and commands of the gait's limit cycle with the stroke period
  stretched to prob.T (the Fourier commands at node times)."""
  m = prob.m
  L = C.StateLayout(load_free_model(prob.dt))
  s = gait['state']
  x = free_to_euler(m, s[L.qpos], s[L.qvel])
  nstep = cycles * prob.N * prob.K
  u = fourier_ctrl(gait, nstep, prob.dt, prob.T)
  state, _ = mujoco.rollout.rollout(m, mujoco.MjData(m),
                                    prob._states(x[None])[0], u[None])
  state = state[0]
  traj = np.concatenate([state[:, 1:1 + prob.nv],
                         state[:, 1 + prob.nv:1 + 2 * prob.nv]], axis=1)
  traj = np.concatenate([x[None], traj])
  start = (cycles - 1) * prob.N * prob.K
  X = traj[start:start + prob.N * prob.K:prob.K].copy()
  X[:, :3] -= X[0, :3]
  X[:, 3] -= 0.0
  P = fourier_ctrl(gait, prob.N * prob.K, prob.dt, prob.T)[::prob.K]
  return np.concatenate([X.ravel(), P.ravel()])


def periodic_ctrl(P, K, n, phase0=0):
  """Per-step commands (n, nu) of the periodic piecewise-linear command."""
  N = len(P)
  j = phase0 + np.arange(n)
  i = (j // K) % N
  w = ((j % K) / K)[:, None]
  return (1 - w) * P[i] + w * P[(i + 1) % N]


def simulate_free(x0_eul, P, K, dt_opt, dt, cycles, m_eul):
  """Roll the free-joint model with the periodic commands (resampled to dt)
  from the Euler state x0; returns qpos, qvel per step and the time."""
  m = load_free_model(dt)
  qpos, qvel = euler_to_free(m_eul, x0_eul)
  T = len(P) * K * dt_opt
  n = int(round(cycles * T / dt))
  t = np.arange(n) * dt
  # piecewise-linear command at time t (knots every K * dt_opt)
  f = (t / (K * dt_opt)) % len(P)
  i = np.floor(f).astype(int) % len(P)
  w = (f - np.floor(f))[:, None]
  u = (1 - w) * P[i] + w * P[(i + 1) % len(P)]
  s0 = full_state(m, qpos, qvel)
  state, _ = mujoco.rollout.rollout(m, mujoco.MjData(m), s0, u[None])
  state = np.concatenate([s0[None], state[0]])
  L = C.StateLayout(m)
  return state[:, L.qpos], state[:, L.qvel], T


def report(qpos, qvel, T, dt, direction, skip=1):
  """Speed and drift per cycle of a free-joint rollout."""
  steps = int(round(T / dt))
  ncyc = (len(qpos) - 1) // steps
  lat, vert = G.drift_axes(np.asarray(direction, float))
  rows = []
  for c in range(ncyc):
    a, b = c * steps, (c + 1) * steps
    disp = qpos[b, :3] - qpos[a, :3]
    yaw = C.yaw_of(C.quat_to_mat(qpos[[a, b], 3:7]))
    rows.append(dict(cycle=c, speed=float(direction @ disp) / T,
                     lateral=float(lat @ disp) / T,
                     depth=float(vert @ disp) / T,
                     yaw=float(np.angle(np.exp(1j * (yaw[1] - yaw[0]))))))
  return rows


def validate(z, prob, args, direction, m):
  X, P = prob.X(z), prob.P(z)
  out = {}
  for dt in sorted({args.dt, 0.001}):
    qpos, qvel, T = simulate_free(X[0], P, args.substeps, args.dt, dt,
                                  args.validate_cycles, m)
    rows = report(qpos, qvel, T, dt, direction)
    out[str(dt)] = rows
    print(f'free-joint model, dt {dt}: speed per cycle ' + ' '.join(
        f'{r["speed"]:.4f}' for r in rows))
    print(f'  lateral ' + ' '.join(f'{r["lateral"]:+.4f}' for r in rows))
    print(f'  yaw/cycle ' + ' '.join(f'{r["yaw"]:+.3f}' for r in rows),
          flush=True)
  return out


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawTextHelpFormatter)
  p.add_argument('--init', required=True)
  p.add_argument('--out', required=True)
  p.add_argument('--nodes', type=int, default=54)
  p.add_argument('--substeps', type=int, default=10)
  p.add_argument('--dt', type=float, default=0.002)
  p.add_argument('--smooth', type=float, default=1e-2)
  p.add_argument('--max_iter', type=int, default=300)
  p.add_argument('--threads', type=int, default=4)
  p.add_argument('--validate_cycles', type=int, default=12)
  p.add_argument('--maximize', action='store_true',
                 help='one solve maximizing speed (no ladder)')
  p.add_argument('--ladder', type=float, nargs='*', default=None,
                 help='speed targets, m/s (default: from the warm start '
                 'up in steps of --step until a target fails)')
  p.add_argument('--step', type=float, default=0.005)
  p.add_argument('--prox_max', type=int, default=0,
                 help='instead of a ladder: this many proximal stages, each '
                 'maximizing speed with the pull (--prox) to the previous '
                 'solution (a trust region), stopping when the gain is '
                 'below --min_gain')
  p.add_argument('--min_gain', type=float, default=2e-4)
  p.add_argument('--smooth_servo', action='store_true',
                 help='optimize on the unsaturated servo law with the '
                 'torque and voltage limits as constraints at the nodes '
                 '(smooth_servos); validation stays on the real dcmotor')
  p.add_argument('--limit_margin', type=float, default=0.0)
  p.add_argument('--prox0', type=float, default=None,
                 help='pull in stage 0 (default --prox). Keep it small when '
                 'the warm start is far from feasible: the Gauss-Newton '
                 'Hessian leaves out the constraint curvature, which is '
                 'scaled by multipliers that grow with the pull')
  p.add_argument('--prox', type=float, default=1.0)
  p.add_argument('--hessian', choices=('lbfgs', 'gn'), default='lbfgs',
                 help='IPOPT Hessian: limited-memory BFGS, or constrained '
                 'Gauss-Newton (objective Hessian only)')
  args = p.parse_args()
  os.makedirs(args.out, exist_ok=True)
  C.set_robot('v2')
  gait = gait_from_npz(args.init)
  z_init = np.load(args.init)
  direction = z_init['direction']
  m = load_euler_model(args.dt)
  servo = smooth_servos(m) if args.smooth_servo else None

  def make(**kw):
    return PeriodicShooting(m, args.nodes, args.substeps, direction,
                            smooth=args.smooth, nthread=args.threads,
                            servo=servo, limit_margin=args.limit_margin, **kw)

  prob = make()
  print(f'stroke {prob.T:.3f} s (warm-start gait {gait["period"]:.4f} s, '
        f'{gait["metrics"]["speed"]:.4f} m/s), {prob.n} variables, '
        f'{prob.m_rows} constraints', flush=True)
  z0 = warm_start(prob, gait)
  v0 = prob.speed(z0)
  print(f'warm start: speed {v0:.4f} m/s, max defect '
        f'{np.abs(prob.constraints(z0)).max():.3e}', flush=True)
  history = []
  t_start = time.time()
  if args.maximize:
    stages = [None]
  elif args.prox_max:
    stages = [round(v0 - args.step, 4)] + ['max'] * args.prox_max
  elif args.ladder:
    stages = list(args.ladder)
  else:
    # first a feasible periodic orbit near the warm start, then up
    stages = [round(v0 - args.step, 4)] + [
        round(v0 + k * args.step, 4) for k in range(1, 40)]
  z = z0
  best = None
  for stage, target in enumerate(stages):
    pull = args.prox if stage or args.prox0 is None else args.prox0
    prob = make(speed_target=None if target in (None, 'max') else target,
                prox=0.0 if target is None else pull, z_ref=z.copy())
    t0 = time.time()
    print(f'--- stage {stage}: speed target {target}', flush=True)
    zs, info = prob.solve(z, max_iter=args.max_iter, hessian=args.hessian)
    wall = time.time() - t0
    c = prob.constraints(zs)
    viol = float(np.abs(c[:prob.n_eq]).max())
    if prob.n_lin:
      lv = c[prob.row_lin:]
      viol = max(viol, float(np.max(np.maximum(prob.lin_lo - lv,
                                               lv - prob.lin_hi))))
    v = prob.speed(zs)
    ok = info['status'] in (0, 1) and viol < 1e-4
    row = dict(stage=stage, target=target, speed=v, max_defect=viol,
               status=info['status_msg'].decode(), wall_s=wall,
               fd_passes=prob.evals, ok=ok)
    history.append(row)
    print(f'stage {stage}: {row["status"]} in {wall:.0f} s, '
          f'{prob.evals} FD passes; speed {v:.4f} m/s, max defect '
          f'{viol:.2e}', flush=True)
    if not ok:
      print('stage failed; keeping the last good solution', flush=True)
      break
    gain = v - (best['speed'] if best else -np.inf)
    z = zs
    best = dict(z=zs, speed=v, stage=stage)
    np.savez(os.path.join(args.out, f'stage{stage}_v{v:.4f}.npz'),
             X=prob.X(zs), P=prob.P(zs), z=zs, nodes=args.nodes,
             substeps=args.substeps, dt=args.dt, period=prob.T,
             direction=direction, init=args.init, speed=v, robot='v2')
    with open(os.path.join(args.out, 'history.json'), 'w') as f:
      json.dump(history, f, indent=1)
    if target == 'max' and gain < args.min_gain:
      print(f'gain {gain:.5f} m/s below {args.min_gain}; done', flush=True)
      break
  out = dict(warm_speed=v0, history=history,
             total_wall_s=time.time() - t_start)
  if best is not None:
    print(f'best: stage {best["stage"]}, {best["speed"]:.4f} m/s '
          f'(warm start {v0:.4f}); validating', flush=True)
    out['best'] = dict(stage=best['stage'], speed=best['speed'])
    out['validate'] = validate(best['z'], prob, args, direction, m)
    np.savez(os.path.join(args.out, 'best.npz'), X=prob.X(best['z']),
             P=prob.P(best['z']), z=best['z'], nodes=args.nodes,
             substeps=args.substeps, dt=args.dt, period=prob.T,
             direction=direction, init=args.init, speed=best['speed'],
             robot='v2')
  with open(os.path.join(args.out, 'summary.json'), 'w') as f:
    json.dump(out, f, indent=1)


if __name__ == '__main__':
  main()
