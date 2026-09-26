#!/usr/bin/env python3
"""Gradient-based (iLQR) trajectory optimization of a pterosaur launch.

Full-horizon iLQR from the reference's deepest crouch through takeoff, on
the same physics as launch_trajopt.py (2x torque, hard joint limits,
default foot contact impedance, 2 ms timestep), with left/right symmetric
controls (6 channels mirrored to 12 actuators).

Contacts are shaped with smooth kinematic costs instead of contact counts:
  push (t < T):    hands on the ground and not sliding until T, feet until
                   FEET_LIFT_FRAC * T (the reference's contact order);
                   knee/shin/forearm clearance; body pitch within 45 deg
  takeoff (t = T): CoM velocity to the launch velocity; low angular momentum
  flight (t > T):  hands and feet off the ground; calm joints

Dynamics derivatives come from mujoco.mjd_transitionFD, cost derivatives
from finite differences of residuals (Gauss-Newton).

Usage:
  launch_ilqr.py --push_time 0.45 [--init controls.npy] [--iters 100]
      [--speed 6 --angle 30] [--out result.npz]
"""
import argparse
import os
import time

import mujoco
import numpy as np

import launch_trajopt as T

FEET_LIFT_FRAC = T.FEET_LIFT_FRAC
FLIGHT = 0.2          # seconds optimized after takeoff
PLANT_TOL = 0.001     # m, hand/foot height counted as planted

# residual scales (a residual of 1 = this much error) and weights
S_PLANT_H, W_PLANT_H = 0.005, 1.0     # planted limb height (m)
S_PLANT_V, W_PLANT_V = 0.1, 0.5       # planted limb horizontal speed (m/s)
S_CLEAR, W_CLEAR = 0.01, 5.0          # clearance violation (m)
S_PITCH, W_PITCH = 0.1, 1.0           # pitch beyond 45 deg (rad)
W_EFFORT = 0.01                       # control (unitless)
S_TAKEOFF_V, W_TAKEOFF_V = 0.5, 10.0  # takeoff CoM velocity error (m/s)
S_SPIN, W_SPIN = 0.05, 5.0            # takeoff angular momentum / mass
S_CALM, W_CALM = 5.0, 0.2             # joint speed in flight (rad/s)
S_LIFT, W_LIFT = 0.01, 1.0            # limb below 3 cm in flight (m)
LIFT_HEIGHT = 0.03

# 6 symmetric channels -> 12 actuators
E = np.zeros((12, 6))
E[T.LEFT, np.arange(6)] = 1
E[T.RIGHT, np.arange(6)] = T.MIRROR


class LaunchILQR:

  def __init__(self, push_time, torque=2.0, speed=6.0, angle=30.0):
    self.opt = T.LaunchOpt(torque, angle)  # physics model and start state
    self.m = self.opt.m
    self.d = mujoco.MjData(self.m)
    m = self.m
    self.nx = 2 * m.nv
    self.dt = m.opt.timestep
    self.n_push = int(round(push_time / self.dt))
    self.n_feet = int(round(FEET_LIFT_FRAC * push_time / self.dt))
    self.N = self.n_push + int(round(FLIGHT / self.dt))
    a = np.deg2rad(angle)
    self.v_target = speed * np.array([-np.cos(a), 0, np.sin(a)])
    self.mass = m.body_subtreemass[1]
    gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    self.hands = [gid('FL'), gid('FR')]
    self.feet = [gid('HL'), gid('HR')]
    self.clear = [(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b), np.array(p), c)
                  for b, p, c in T.CLEARANCE]
    self.angmom = T.sensor(m, 'torso_subtreeangmom')
    mujoco.mj_setState(m, self.d, self.opt.x0, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    mujoco.mj_forward(m, self.d)
    self.q0, self.v0 = self.d.qpos.copy(), self.d.qvel.copy()

  # ----- residuals ----- #
  def limb(self, g):
    """Height above floor and horizontal speed of a limb sphere."""
    m, d = self.m, self.d
    h = d.geom_xpos[g, 2] - m.geom_size[g, 0] - T.FLOOR_Z
    vel = np.zeros(6)
    mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_GEOM, g, vel, 0)
    return h, np.linalg.norm(vel[3:5])

  def residual(self, t):
    """Weighted residual vector at step t (costs = 0.5 |r|^2); mj_forward
    must have been called at the state. Control terms are separate."""
    d = self.d
    r = []
    if t < self.n_push:
      stance = self.hands + (self.feet if t < self.n_feet else [])
      for g in self.hands + self.feet:
        if g in stance:
          h, v = self.limb(g)
          r += [np.sqrt(W_PLANT_H) * max(h - PLANT_TOL, 0) / S_PLANT_H,
                np.sqrt(W_PLANT_V) * v / S_PLANT_V]
        else:
          r += [0, 0]
      for b, p, c in self.clear:
        height = (d.xpos[b] + d.xmat[b].reshape(3, 3) @ p)[2] - T.FLOOR_Z
        r.append(np.sqrt(W_CLEAR) * max(c - height, 0) / S_CLEAR)
      pitch = abs(np.arcsin(np.clip(d.xmat[1, 6], -1, 1)))
      r.append(np.sqrt(W_PITCH) * max(pitch - T.PITCH_MAX, 0) / S_PITCH)
    else:
      for g in self.hands + self.feet:
        h, _ = self.limb(g)
        r.append(np.sqrt(W_LIFT) * max(LIFT_HEIGHT - h, 0) / S_LIFT)
      r += list(np.sqrt(W_CALM) * d.qvel[6:] / S_CALM)
    return np.array(r) * np.sqrt(self.dt)

  def takeoff_residual(self):
    d = self.d
    v = d.subtree_linvel[1]
    L = d.sensordata[self.angmom] / self.mass
    return np.concatenate([np.sqrt(W_TAKEOFF_V) * (v - self.v_target) / S_TAKEOFF_V,
                           np.sqrt(W_SPIN) * L / S_SPIN])

  def residual_at(self, q, v, t):
    self.d.qpos[:], self.d.qvel[:] = q, v
    mujoco.mj_forward(self.m, self.d)
    r = self.residual(t)
    if t == self.n_push:
      r = np.concatenate([r, self.takeoff_residual()])
    return r

  # ----- rollout and cost ----- #
  def rollout(self, U):
    """U: (N, 6). Returns states (N+1, nq), (N+1, nv), cost."""
    m, d = self.m, self.d
    d.qpos[:], d.qvel[:] = self.q0, self.v0
    d.time = 0
    mujoco.mj_forward(m, d)
    Q = np.empty((self.N + 1, m.nq))
    V = np.empty((self.N + 1, m.nv))
    cost = 0.0
    for t in range(self.N + 1):
      Q[t], V[t] = d.qpos, d.qvel
      r = self.residual(t)
      if t == self.n_push:
        r = np.concatenate([r, self.takeoff_residual()])
      cost += 0.5 * r @ r
      if t == self.N:
        break
      u = np.clip(U[t], -1, 1)
      cost += 0.5 * W_EFFORT * self.dt * u @ u
      d.ctrl[:] = E @ u
      mujoco.mj_step(m, d)
      if not np.all(np.isfinite(d.qvel)) or np.abs(d.qvel).max() > 1e3:
        return Q, V, np.inf
    return Q, V, cost

  def derivatives(self, Q, V, U):
    """Dynamics Jacobians A (N, nx, nx), B (N, nx, 6) and residual
    Jacobians Jx (N+1) (lists), residuals R."""
    m, d = self.m, self.d
    nx, nv = self.nx, m.nv
    A = np.empty((self.N, nx, nx))
    B = np.empty((self.N, nx, 6))
    Bfull = np.empty((nx, m.nu))
    R, Jx = [], []
    eps = 1e-6
    dq = np.zeros(nv)
    for t in range(self.N + 1):
      # residual Jacobian w.r.t. state (tangent space), forward differences
      r0 = self.residual_at(Q[t], V[t], t)
      J = np.empty((len(r0), nx))
      for i in range(nv):
        q = Q[t].copy()
        dq[:] = 0
        dq[i] = 1
        mujoco.mj_integratePos(m, q, dq, eps)
        J[:, i] = (self.residual_at(q, V[t], t) - r0) / eps
      for i in range(nv):
        v = V[t].copy()
        v[i] += eps
        J[:, nv + i] = (self.residual_at(Q[t], v, t) - r0) / eps
      R.append(r0)
      Jx.append(J)
      if t == self.N:
        break
      d.qpos[:], d.qvel[:] = Q[t], V[t]
      d.ctrl[:] = E @ np.clip(U[t], -1, 1)
      mujoco.mj_forward(m, d)
      mujoco.mjd_transitionFD(m, d, 1e-6, False, A[t], Bfull, None, None)
      B[t] = Bfull @ E
    return A, B, R, Jx

  # ----- iLQR ----- #
  def solve(self, U, iters=100, verbose=True):
    Q, V, cost = self.rollout(U)
    mu = 1e-3
    history = [cost]
    for it in range(iters):
      t0 = time.time()
      A, B, R, Jx = self.derivatives(Q, V, U)
      # backward pass (Gauss-Newton cost Hessians)
      Vx = Jx[-1].T @ R[-1]
      Vxx = Jx[-1].T @ Jx[-1]
      k = np.zeros((self.N, 6))
      K = np.zeros((self.N, 6, self.nx))
      ok = True
      for t in reversed(range(self.N)):
        u = np.clip(U[t], -1, 1)
        lx = Jx[t].T @ R[t]
        lxx = Jx[t].T @ Jx[t]
        lu = W_EFFORT * self.dt * u
        luu = W_EFFORT * self.dt * np.eye(6)
        Qx = lx + A[t].T @ Vx
        Qu = lu + B[t].T @ Vx
        Qxx = lxx + A[t].T @ Vxx @ A[t]
        Quu = luu + B[t].T @ Vxx @ B[t] + mu * np.eye(6)
        Qux = B[t].T @ Vxx @ A[t]
        try:
          L = np.linalg.cholesky(Quu)
        except np.linalg.LinAlgError:
          ok = False
          break
        k[t] = -np.linalg.solve(Quu, Qu)
        K[t] = -np.linalg.solve(Quu, Qux)
        # saturated controls: no feedback, no push past the limit
        free = ~(((u >= 1) & (k[t] > 0)) | ((u <= -1) & (k[t] < 0)))
        k[t][~free] = 0
        K[t][~free] = 0
        Vx = Qx + K[t].T @ Quu @ k[t] + K[t].T @ Qu + Qux.T @ k[t]
        Vxx = Qxx + K[t].T @ Quu @ K[t] + K[t].T @ Qux + Qux.T @ K[t]
        Vxx = 0.5 * (Vxx + Vxx.T)
      if not ok:
        mu *= 10
        continue
      # forward pass with line search
      improved = False
      for alpha in [1.0, 0.5, 0.25, 0.1, 0.05, 0.02]:
        Un = self.forward(Q, V, U, k, K, alpha)
        Qn, Vn, cn = self.rollout(Un)
        if cn < cost:
          U, Q, V, cost = Un, Qn, Vn, cn
          improved = True
          break
      mu = max(mu / 3, 1e-6) if improved else mu * 10
      history.append(cost)
      if verbose:
        print(f'  iter {it:3d} cost {cost:.4f} alpha {alpha if improved else 0} '
              f'mu {mu:.1e} ({time.time() - t0:.1f}s)', flush=True)
      if mu > 1e4:
        break
    return np.clip(U, -1, 1), cost, history

  def forward(self, Q, V, U, k, K, alpha):
    """Closed-loop forward pass: u = U + alpha k + K (x - x_nominal)."""
    m, d = self.m, self.d
    d.qpos[:], d.qvel[:] = self.q0, self.v0
    mujoco.mj_forward(m, d)
    Un = np.empty_like(U)
    dx = np.zeros(self.nx)
    for t in range(self.N):
      mujoco.mj_differentiatePos(m, dx[:m.nv], 1.0, Q[t], d.qpos)
      dx[m.nv:] = d.qvel - V[t]
      Un[t] = np.clip(U[t] + alpha * k[t] + K[t] @ dx, -1, 1)
      d.ctrl[:] = E @ Un[t]
      mujoco.mj_step(m, d)
    return Un

  def report(self, U):
    """Launch metrics with the evaluation used by launch_trajopt.py."""
    half = np.zeros((self.opt.nstep, 6))
    half[:self.N] = U
    half[self.N:] = 0
    _, det = self.opt.evaluate(half[None], detail=True)
    return half, det[0]


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--push_time', type=float, default=0.45)
  parser.add_argument('--torque', type=float, default=2.0)
  parser.add_argument('--speed', type=float, default=6.0)
  parser.add_argument('--angle', type=float, default=30.0)
  parser.add_argument('--init', default=None, help='controls (nstep, 6) .npy')
  parser.add_argument('--iters', type=int, default=100)
  parser.add_argument('--out', default='launch_ilqr.npz')
  args = parser.parse_args()

  solver = LaunchILQR(args.push_time, args.torque, args.speed, args.angle)
  if args.init:
    U = np.load(args.init)[:solver.N]
    print(f'init from {args.init}')
  else:
    U = solver.opt.pd_warm_start(args.push_time)[:solver.N]
    print(f'init from PD warm start, push {args.push_time}s')
  _, det = solver.report(U)
  print('initial:', det, flush=True)

  U, cost, _ = solver.solve(U, iters=args.iters)
  half, det = solver.report(U)
  print('result:', det, flush=True)
  solver.opt.record(half, args.out)
  np.save(os.path.splitext(args.out)[0] + '_controls.npy', half)
  print('wrote', args.out)


if __name__ == '__main__':
  main()
