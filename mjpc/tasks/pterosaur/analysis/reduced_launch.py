#!/usr/bin/env python3
"""Reduced-order launch model for actuator and spring sizing.

A planar point-mass body pushed off the ground by two massless 2-link limbs
(hindlimbs: hip2 + knee; forelimbs: shoulder2 + elbow; left and right
lumped), with geometry taken from the MuJoCo pterosaur model at the launch
crouch. Each joint has a DC-motor torque-speed limit (stall torque falling
linearly to zero at the no-load speed, so power is bounded) and an optional
latched parallel spring, preloaded in the crouch and released at push start.

Contact is unilateral with Coulomb friction (the tangential force is clamped
to the friction cone, so slipping cannot add thrust). A limb whose force
would pull unloads while its foot/hand stays down; it lifts off, for good,
when the body moves out of its reach or a joint reaches its limit. Takeoff
is when both limbs are off.
Body pitch is ignored (translation only).

For a design, the joint torque profiles (piecewise linear, symmetric
left/right) are optimized with the cross-entropy method to maximize takeoff
speed along the launch direction; rollouts are simulated as a vectorized
batch.

Complements habib_launch.py (energetics: constant-acceleration push-off
power vs. available power) with dynamics: limb geometry, actuator
torque-speed limits and springs.

Usage:
  reduced_launch.py [--torque 2] [--no_load_speed 0] [--spring_energy 0]
      [--mass 50.13] [--angle 30] [--limb_scale 1]
"""
import argparse
import os
from dataclasses import dataclass, field

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TASK_DIR = os.path.dirname(HERE)
G = 9.81


@dataclass
class Limb:
  name: str
  proximal_offset: np.ndarray   # proximal joint relative to CoM (x, z), m
  contact: np.ndarray           # foot/hand on the ground relative to CoM
  l1: float                     # proximal segment, m
  l2: float                     # distal segment, m
  branch: float                 # +1/-1: which way the middle joint bends
  stall_torque: np.ndarray      # (2,) N m, both sides combined
  q_crouch: np.ndarray = None   # joint angles at the crouch
  q_lo: np.ndarray = None
  q_hi: np.ndarray = None
  spring_k: np.ndarray = field(default_factory=lambda: np.zeros(2))
  spring_rest: np.ndarray = field(default_factory=lambda: np.zeros(2))


def robot_limbs(torque_scale=1.0, limb_scale=1.0, joint_margin=0.3):
  """Sagittal limb geometry of the MuJoCo model at the launch crouch."""
  import mujoco
  m = mujoco.MjModel.from_xml_path(os.path.join(TASK_DIR, 'task.xml'))
  d = mujoco.MjData(m)
  ref = np.load(os.path.join(TASK_DIR, 'reference', 'launch.npz'))
  airborne = np.flatnonzero(~ref['hands_contact'] & ~ref['feet_contact'])
  i0 = int(np.argmin(ref['qpos'][:airborne[0], 2]))
  d.qpos[:] = ref['qpos'][i0]
  mujoco.mj_forward(m, d)
  jnt = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
  geom = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
  act = lambda n: m.actuator_gainprm[mujoco.mj_name2id(
      m, mujoco.mjtObj.mjOBJ_ACTUATOR, n), 0]
  com = d.subtree_com[1][[0, 2]]
  q = ref['qpos'][:, 7:]
  limbs = []
  for name, j1, j2, tip, sides, idx in [
      ('hind', 'leftleg_hip2', 'leftleg_knee', 'HL',
       ['leftleg_hip2', 'leftleg_knee', 'rightleg_hip2', 'rightleg_knee'], [7, 8]),
      ('fore', 'leftarm_shoulder2', 'leftarm_elbow', 'FL',
       ['leftarm_shoulder2', 'leftarm_elbow', 'rightarm_shoulder2',
        'rightarm_elbow'], [1, 2])]:
    p = d.xanchor[jnt(j1)][[0, 2]]
    k = d.xanchor[jnt(j2)][[0, 2]]
    g = geom(tip)
    t = d.geom_xpos[g][[0, 2]] - np.array([0, m.geom_size[g, 0]])
    l1 = np.linalg.norm(k - p) * limb_scale
    l2 = np.linalg.norm(t - k) * limb_scale
    a, b = k - p, t - k
    branch = np.sign(a[0] * b[1] - a[1] * b[0])
    stall = torque_scale * np.array([act(sides[0]) + act(sides[2]),
                                     act(sides[1]) + act(sides[3])])
    limb = Limb(name, (p - com) * limb_scale, (t - com) * limb_scale, l1, l2,
                branch, stall)
    limb.q_crouch = joint_angles(limb, np.zeros((1, 2)))[0]
    # sign between model joint angles and these planar angles, from how both
    # change along the reference while the limb is planted
    i1 = i0 + 40
    d.qpos[:] = ref['qpos'][i1]
    mujoco.mj_forward(m, d)
    p1 = d.subtree_com[1][[0, 2]] - com
    q1 = joint_angles(limb, p1[None])[0]
    sign = np.sign((q1 - limb.q_crouch) * (q[i1, idx] - q[i0, idx]))
    sign[sign == 0] = 1
    d.qpos[:] = ref['qpos'][i0]
    mujoco.mj_forward(m, d)
    # joint range: the model joint excursion over the reference, +- margin,
    # expressed as a change from the crouch
    exc = (q[:, idx] - q[i0, idx]) * sign
    limb.q_lo = limb.q_crouch + exc.min(axis=0) - joint_margin
    limb.q_hi = limb.q_crouch + exc.max(axis=0) + joint_margin
    limbs.append(limb)
  mass = m.body_subtreemass[1]
  return limbs, mass


def joint_angles(limb, p):
  """(B, 2) body positions -> (B, 2) joint angles [proximal abs, middle rel]."""
  v = limb.contact[None] - (p + limb.proximal_offset[None])
  r = np.linalg.norm(v, axis=1)
  cos_mid = np.clip((r ** 2 - limb.l1 ** 2 - limb.l2 ** 2) /
                    (2 * limb.l1 * limb.l2), -1, 1)
  mid = limb.branch * np.arccos(cos_mid)
  a1 = (np.arctan2(v[:, 1], v[:, 0]) -
        np.arctan2(limb.l2 * np.sin(mid), limb.l1 + limb.l2 * np.cos(mid)))
  return np.stack([a1, mid], axis=1)


def jacobian(limb, q):
  """(B, 2, 2) d(contact - proximal)/dq."""
  a1, a12 = q[:, 0], q[:, 0] + q[:, 1]
  J = np.empty((len(q), 2, 2))
  J[:, 0, 0] = -limb.l1 * np.sin(a1) - limb.l2 * np.sin(a12)
  J[:, 0, 1] = -limb.l2 * np.sin(a12)
  J[:, 1, 0] = limb.l1 * np.cos(a1) + limb.l2 * np.cos(a12)
  J[:, 1, 1] = limb.l2 * np.cos(a12)
  return J


@dataclass
class Design:
  torque_scale: float = 2.0
  no_load_speed: float = 0.0    # rad/s; 0 = no speed limit (ideal motor)
  spring_energy: float = 0.0    # J, split over the four middle/proximal joints
  mass: float = None            # kg; None = model mass
  limb_scale: float = 1.0
  joint_margin: float = 0.3
  friction: float = 1.0


class ReducedLaunch:

  def __init__(self, design, angle=30.0, horizon=0.6, dt=0.001, knots=16):
    self.design = design
    self.limbs, model_mass = robot_limbs(design.torque_scale, design.limb_scale,
                                         design.joint_margin)
    self.mass = design.mass or model_mass
    self.dt, self.n = dt, int(round(horizon / dt))
    self.knots = knots
    a = np.deg2rad(angle)
    self.direction = np.array([-np.cos(a), np.sin(a)])  # launch toward -x
    self.com_height = None
    if design.spring_energy > 0:
      self.add_springs(design.spring_energy)

  def add_springs(self, energy):
    """Parallel springs on all four joints, energy/4 each at the crouch,
    resting at the extended end of the joint's range."""
    for limb in self.limbs:
      rest = np.where(np.abs(limb.q_hi - limb.q_crouch) >
                      np.abs(limb.q_lo - limb.q_crouch), limb.q_hi, limb.q_lo)
      limb.spring_rest = rest
      limb.spring_k = 2 * (energy / 4) / (limb.q_crouch - rest) ** 2

  @property
  def seeds(self):
    """Heuristic starting commands: the middle joints (knee, elbow) drive
    limb extension at full effort, with and without the proximal joints."""
    out = []
    for prox in (0.0, 1.0, -1.0):
      c = np.zeros((4, self.knots))
      for i, limb in enumerate(self.limbs):
        extend = -np.sign(self.reach_gradient(limb)[1])
        c[2 * i + 1] = extend
        c[2 * i] = prox
      out.append(c)
    return np.array(out)

  @staticmethod
  def reach_gradient(limb):
    """d(reach)/dq at the crouch."""
    q = limb.q_crouch
    a1, a12 = q[0], q[0] + q[1]
    fk = np.array([limb.l1 * np.cos(a1) + limb.l2 * np.cos(a12),
                   limb.l1 * np.sin(a1) + limb.l2 * np.sin(a12)])
    J = jacobian(limb, q[None])[0]
    return fk @ J / np.linalg.norm(fk)

  def controls(self, params):
    """(B, 4, knots) in [-1, 1] -> (B, n, 4) piecewise-linear commands."""
    t_knot = np.linspace(0, 1, params.shape[2])
    t = np.linspace(0, 1, self.n)
    B = params.shape[0]
    out = np.empty((B, self.n, 4))
    for c in range(4):
      for b in range(B):
        out[b, :, c] = np.interp(t, t_knot, params[b, c])
    return np.clip(out, -1, 1)

  def simulate(self, params, record=False):
    """Batched rollout. Returns takeoff velocity (B, 2), takeoff time,
    penalties and optionally the trajectory."""
    u = self.controls(params)
    B = params.shape[0]
    p = np.zeros((B, 2))
    v = np.zeros((B, 2))
    stance = np.ones((B, 2), bool)
    q_prev = [joint_angles(l, p) for l in self.limbs]
    takeoff_v = np.full((B, 2), np.nan)
    takeoff_t = np.full(B, np.nan)
    penalty = np.zeros(B)
    work = np.zeros(B)
    spring0 = self.spring_energy(q_prev)
    traj = []
    w0 = self.design.no_load_speed
    for k in range(self.n):
      force = np.zeros((B, 2))
      power = np.zeros(B)
      for i, limb in enumerate(self.limbs):
        q = joint_angles(limb, p)
        J = jacobian(limb, q)
        # joint velocities: contact fixed, body moves: dq = -J^-1 v
        det = J[:, 0, 0] * J[:, 1, 1] - J[:, 0, 1] * J[:, 1, 0]
        det = np.where(np.abs(det) < 1e-6, 1e-6, det)
        Jinv = np.stack([np.stack([J[:, 1, 1], -J[:, 0, 1]], 1),
                         np.stack([-J[:, 1, 0], J[:, 0, 0]], 1)], 1) / det[:, None, None]
        dq = -np.einsum('bij,bj->bi', Jinv, v)
        cmd = u[:, k, 2 * i:2 * i + 2]
        stall = limb.stall_torque[None]
        if w0 > 0:
          hi = np.minimum(stall, stall * (1 - dq / w0))
          lo = np.maximum(-stall, stall * (-1 - dq / w0))
          tau = lo + (cmd + 1) / 2 * (hi - lo)
        else:
          tau = cmd * stall
        tau_spring = -limb.spring_k[None] * (q - limb.spring_rest[None])
        # force on the body from the limb: virtual work tau . dq = F . v
        F = -np.einsum('bji,bj->bi', Jinv, tau + tau_spring)
        # unilateral contact and liftoff conditions
        reach = np.linalg.norm(limb.contact[None] - (p + limb.proximal_offset[None]),
                               axis=1) / (limb.l1 + limb.l2)
        limit = np.any((q < limb.q_lo[None]) | (q > limb.q_hi[None]), axis=1)
        # the ground cannot pull: a limb whose force would pull just unloads
        # (foot/hand stays down); it lifts off, for good, when the body moves
        # out of its reach or a joint reaches its limit. Feet never leave and
        # return, so there are no impacts.
        F[F[:, 1] < 0] = 0
        lift = stance[:, i] & ((reach > 0.99) | limit)
        stance[:, i] &= ~lift
        F *= stance[:, i:i + 1]
        # friction: tangential force limited to the cone (conservative: the
        # joint torque that would need more grip is lost)
        limit_x = self.design.friction * np.maximum(F[:, 1], 0)
        slip = np.maximum(np.abs(F[:, 0]) - limit_x, 0)
        penalty += slip / (self.mass * G) * self.dt * 0  # recorded only
        F[:, 0] = np.clip(F[:, 0], -limit_x, limit_x)
        force += F
        power += np.einsum('bi,bi->b', tau, dq) * stance[:, i]
      airborne = ~stance.any(axis=1) & np.isnan(takeoff_t)
      takeoff_v[airborne] = v[airborne]
      takeoff_t[airborne] = k * self.dt
      work += np.maximum(power, 0) * self.dt
      acc = force / self.mass + np.array([0, -G])
      acc[~stance.any(axis=1)] = [0, -G]
      v = v + acc * self.dt
      p = p + v * self.dt
      if record:
        traj.append((p.copy(), v.copy(), stance.copy()))
    # never took off: use the final state, heavily penalized
    never = np.isnan(takeoff_t)
    takeoff_v[never] = v[never]
    takeoff_t[never] = self.n * self.dt
    penalty += never * 0.5
    result = {'v': takeoff_v, 't': takeoff_t, 'penalty': penalty,
              'work': work}
    if record:
      result['traj'] = traj
    return result

  def spring_energy(self, q):
    e = 0.0
    for limb, qi in zip(self.limbs, q):
      e = e + 0.5 * np.sum(limb.spring_k[None] * (qi - limb.spring_rest[None]) ** 2, axis=1)
    return e

  def score(self, res):
    v = res['v']
    along = v @ self.direction
    perp = np.abs(v[:, 0] * self.direction[1] - v[:, 1] * self.direction[0])
    return along - perp - 10 * res['penalty']

  def optimize(self, iters=80, pop=256, elite=24, seed=0, verbose=False,
               init=None):
    """Cross-entropy optimization of the joint commands; init: warm start
    (4, knots) commands."""
    rng = np.random.default_rng(seed)
    if init is None:
      mean = np.zeros((4, self.knots))
      std = np.full((4, self.knots), 0.8)
    else:
      mean = init.copy()
      std = np.full((4, self.knots), 0.3)
    best, best_score = None, -np.inf
    for it in range(iters):
      params = np.clip(mean + std * rng.standard_normal((pop, 4, self.knots)), -1, 1)
      if best is not None:
        params[0] = best
      elif init is not None:
        params[0] = init
      else:
        params[:len(self.seeds)] = self.seeds
      res = self.simulate(params)
      s = self.score(res)
      order = np.argsort(s)[::-1]
      if s[order[0]] > best_score:
        best_score, best = s[order[0]], params[order[0]].copy()
      e = params[order[:elite]]
      mean = 0.3 * mean + 0.7 * e.mean(axis=0)
      std = np.maximum(0.3 * std + 0.7 * e.std(axis=0), 0.02)
      if verbose and it % 10 == 0:
        print(f'  iter {it:3d} best {best_score:.3f}', flush=True)
    res = self.simulate(best[None])
    v = res['v'][0]
    return {
        'params': best,
        'speed': float(np.linalg.norm(v)),
        'angle_deg': float(np.degrees(np.arctan2(v[1], -v[0]))),
        'speed_along': float(v @ self.direction),
        'takeoff_time': float(res['t'][0]),
        'work_J': float(res['work'][0]),
        'kinetic_energy_J': float(0.5 * self.mass * v @ v),
        'friction_violation': float(res['penalty'][0]),
    }

  def rescale(self, params, from_design):
    """Commands for this design giving the same joint torques as `params`
    on `from_design` (ideal motors)."""
    ratio = from_design.torque_scale / self.design.torque_scale
    return np.clip(params * ratio, -1, 1)


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--torque', type=float, default=2.0)
  p.add_argument('--no_load_speed', type=float, default=0.0)
  p.add_argument('--spring_energy', type=float, default=0.0)
  p.add_argument('--mass', type=float, default=None)
  p.add_argument('--limb_scale', type=float, default=1.0)
  p.add_argument('--angle', type=float, default=30.0)
  p.add_argument('--iters', type=int, default=80)
  args = p.parse_args()
  design = Design(args.torque, args.no_load_speed, args.spring_energy,
                  args.mass, args.limb_scale)
  model = ReducedLaunch(design, args.angle)
  for l in model.limbs:
    print(f'{l.name}: l1 {l.l1:.3f} l2 {l.l2:.3f} m, stall {l.stall_torque} N m, '
          f'contact {np.round(l.contact, 3)}, proximal {np.round(l.proximal_offset, 3)}')
  print(model.optimize(args.iters, verbose=True))


if __name__ == '__main__':
  main()
