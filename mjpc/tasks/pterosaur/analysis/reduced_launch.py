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


def joint_angles(limb, p, geom=None):
  """(B, 2) body positions -> (B, 2) joint angles [proximal abs, middle rel].
  geom: per-rollout contact and proximal offset (B, 2), default the limb's."""
  contact = limb.contact[None] if geom is None else geom['contact']
  prox = limb.proximal_offset[None] if geom is None else geom['prox']
  v = contact - (p + prox)
  r = np.linalg.norm(v, axis=1)
  cos_mid = np.clip((r ** 2 - limb.l1 ** 2 - limb.l2 ** 2) /
                    (2 * limb.l1 * limb.l2), -1, 1)
  mid = limb.branch * np.arccos(cos_mid)
  a1 = (np.arctan2(v[:, 1], v[:, 0]) -
        np.arctan2(limb.l2 * np.sin(mid), limb.l1 + limb.l2 * np.cos(mid)))
  if limb.q_crouch is not None:
    # unwrap next to the reference crouch (the forelimb sits near -pi)
    a1 = limb.q_crouch[0] + (a1 - limb.q_crouch[0] + np.pi) % (2 * np.pi) - np.pi
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
  force_cap: float = 0.0        # N, peak total ground force; 0 = no cap


# The launch crouch: CoM height above the ground, hand and foot x relative
# to the CoM (launch toward -x) and body pitch (rad; positive raises the
# hindquarters), held fixed during the push. The reference's deepest crouch
# is the default; optimize(search_crouch=True) searches within these bounds.
CROUCH_NAMES = ('height', 'hand_x', 'foot_x', 'pitch')
CROUCH_BOUNDS = np.array([[0.25, 0.75], [-1.1, 0.0], [-0.1, 0.7], [-0.5, 0.5]])
PROXIMAL_CLEARANCE = 0.10   # m, shoulder/hip above the ground in the crouch
MIDDLE_CLEARANCE = 0.02     # m, knee/elbow above the ground in stance

# Designable latched springs, one per joint (hind proximal, hind middle,
# fore proximal, fore middle). Each stores its share of the energy budget
# and, once its limb pair's latch releases, pushes the joint through
# `travel` rad (signed), with torque tau0 (1 - s / |travel|)^exponent over
# the travel s (exponent 0: constant torque, 1: linear spring, 2: progressive
# release) and nothing beyond (a stop/one-way clutch, so a spring never
# pulls back). Optimized as unit parameters in [-1, 1]:
#   share (4), travel (4), exponent (4), release time (2: hind, fore)
SPRING_JOINTS = ('hind_prox', 'hind_mid', 'fore_prox', 'fore_mid')
MAX_TRAVEL = 3.0     # rad
MAX_RELEASE = 0.5    # s (0.3 for reduced_springs_{main,robust}.json)


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
    hind, fore = self.limbs
    self.reference_crouch = np.array([-0.5 * (hind.contact[1] + fore.contact[1]),
                                      fore.contact[0], hind.contact[0], 0.0])

  def geometry(self, crouch):
    """Per-rollout limb geometry for crouches (B, 4): contact, proximal
    offset, joint limits, crouch angles and springs (B, 2) per limb, and
    whether the crouch is infeasible (B,)."""
    B = len(crouch)
    height, pitch = crouch[:, 0], crouch[:, 3]
    c, s = np.cos(pitch), np.sin(pitch)
    zero = np.zeros(B)
    bad = np.zeros(B, bool)
    out = []
    for limb, x_contact in zip(self.limbs, (crouch[:, 2], crouch[:, 1])):
      x, z = limb.proximal_offset
      g = {'prox': np.stack([c * x - s * z, s * x + c * z], 1),
           'contact': np.stack([x_contact, -height], 1)}
      # proximal joint limits are relative to the body, so they turn with it
      shift = np.stack([pitch, zero], 1)
      g['lo'], g['hi'] = limb.q_lo[None] + shift, limb.q_hi[None] + shift
      g['q0'] = joint_angles(limb, np.zeros((B, 2)), g)
      reach = (np.linalg.norm(g['contact'] - g['prox'], axis=1) /
               (limb.l1 + limb.l2))
      bad |= (reach > 0.97) | (reach < 0.2)
      bad |= np.any((g['q0'] < g['lo']) | (g['q0'] > g['hi']), axis=1)
      bad |= g['prox'][:, 1] < -height + PROXIMAL_CLEARANCE
      # parallel springs on both joints, energy/4 each in the crouch, resting
      # at the extended end of the joint's range
      e = self.design.spring_energy
      g['rest'] = np.where(np.abs(g['hi'] - g['q0']) > np.abs(g['lo'] - g['q0']),
                           g['hi'], g['lo'])
      g['k'] = 2 * (e / 4) / np.maximum((g['q0'] - g['rest']) ** 2, 1e-6)
      out.append(g)
    return out, bad

  def crouch_from_unit(self, z):
    lo, hi = CROUCH_BOUNDS[:, 0], CROUCH_BOUNDS[:, 1]
    return lo + (np.clip(z, -1, 1) + 1) / 2 * (hi - lo)

  def unit_from_crouch(self, crouch):
    lo, hi = CROUCH_BOUNDS[:, 0], CROUCH_BOUNDS[:, 1]
    return 2 * (crouch - lo) / (hi - lo) - 1

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

  def simulate(self, params, record=False, crouch=None, springs=None):
    """Batched rollout from crouches (B, 4) (default: the reference's),
    with designable springs (unit parameters (B, 14), see SPRING_JOINTS;
    default: the design's legacy linear springs). Returns takeoff velocity (B, 2), takeoff time, per-limb liftoff times,
    penalties and optionally the trajectory."""
    u = self.controls(params)
    B = params.shape[0]
    if crouch is None:
      crouch = np.repeat(self.reference_crouch[None], B, axis=0)
    geom, infeasible = self.geometry(crouch)
    height = crouch[:, 0]
    p = np.zeros((B, 2))
    v = np.zeros((B, 2))
    stance = np.ones((B, 2), bool)
    liftoff_t = np.full((B, 2), np.nan)
    clash = infeasible.copy()
    takeoff_v = np.full((B, 2), np.nan)
    takeoff_t = np.full(B, np.nan)
    penalty = np.zeros(B)
    work = np.zeros(B)
    q0 = [g['q0'] for g in geom]
    spring0 = self.spring_energy(q0, geom)
    sp = None
    if springs is not None:
      sp = self.spring_physical(springs)
      spring0 = sp['energy'].sum(axis=1)
      for g in geom:
        g['k'] = np.zeros_like(g['k'])
      latch = {'q_rel': [np.zeros((B, 2)), np.zeros((B, 2))],
               'released': np.zeros((B, 2), bool)}
    spring_work = np.zeros(B)
    net_work = np.zeros(B)
    peak_force = np.zeros(B)
    peak_torque = np.zeros((B, 4))
    takeoff_z = np.full(B, np.nan)
    traj = []
    w0 = self.design.no_load_speed
    for k in range(self.n):
      force = np.zeros((B, 2))
      power = np.zeros(B)
      for i, (limb, g) in enumerate(zip(self.limbs, geom)):
        q = joint_angles(limb, p, g)
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
        tau_spring = -g['k'] * (q - g['rest'])
        if sp is not None:
          tau_spring = tau_spring + self.spring_torque(sp, latch, i, q, k * self.dt)
        # force on the body from the limb: virtual work tau . dq = F . v
        F = -np.einsum('bji,bj->bi', Jinv, tau + tau_spring)
        # unilateral contact and liftoff conditions
        reach = np.linalg.norm(g['contact'] - (p + g['prox']), axis=1) / (limb.l1 + limb.l2)
        limit = np.any((q < g['lo']) | (q > g['hi']), axis=1)
        # the ground cannot pull: a limb whose force would pull just unloads
        # (foot/hand stays down); it lifts off, for good, when the body moves
        # out of its reach or a joint reaches its limit. Feet never leave and
        # return, so there are no impacts.
        F[F[:, 1] < 0] = 0
        lift = stance[:, i] & ((reach > 0.99) | limit)
        liftoff_t[lift, i] = k * self.dt
        stance[:, i] &= ~lift
        F *= stance[:, i:i + 1]
        # a planted limb may not fold flat or put its knee/elbow in the ground
        middle_z = g['prox'][:, 1] + p[:, 1] + limb.l1 * np.sin(q[:, 0])
        clash |= stance[:, i] & ((reach < 0.15) |
                                 (middle_z < -height + MIDDLE_CLEARANCE))
        # friction: tangential force limited to the cone (conservative: the
        # joint torque that would need more grip is lost)
        limit_x = self.design.friction * np.maximum(F[:, 1], 0)
        F[:, 0] = np.clip(F[:, 0], -limit_x, limit_x)
        force += F
        power += np.einsum('bi,bi->b', tau, dq) * stance[:, i]
        spring_work += np.einsum('bi,bi->b', tau_spring, dq) * stance[:, i] * self.dt
        peak_torque[:, 2 * i:2 * i + 2] = np.maximum(
            peak_torque[:, 2 * i:2 * i + 2],
            np.abs(tau + tau_spring) * stance[:, i:i + 1])
      airborne = ~stance.any(axis=1) & np.isnan(takeoff_t)
      takeoff_v[airborne] = v[airborne]
      takeoff_t[airborne] = k * self.dt
      takeoff_z[airborne] = p[airborne, 1]
      work += np.maximum(power, 0) * self.dt
      net_work += power * self.dt
      peak_force = np.maximum(peak_force, np.linalg.norm(force, axis=1))
      acc = force / self.mass + np.array([0, -G])
      acc[~stance.any(axis=1)] = [0, -G]
      v = v + acc * self.dt
      p = p + v * self.dt
      if record:
        traj.append((p.copy(), v.copy(), stance.copy(), power.copy()))
    # never took off: use the final state, heavily penalized
    never = np.isnan(takeoff_t)
    takeoff_v[never] = v[never]
    takeoff_t[never] = self.n * self.dt
    takeoff_z[never] = p[never, 1]
    penalty += never * 0.5 + clash * 1.0
    # energy audit: the body cannot gain more than the motors and springs
    # put in (stiff springs integrated with too large a step create energy)
    gained = 0.5 * self.mass * np.sum(takeoff_v ** 2, axis=1) + self.mass * G * takeoff_z
    excess = gained - (net_work + spring_work)
    energy_error = excess / np.maximum(gained, 1.0)
    penalty += np.maximum(energy_error - 0.02, 0) * 5
    if self.design.force_cap > 0:
      penalty += np.maximum(peak_force / self.design.force_cap - 1, 0)
    result = {'v': takeoff_v, 't': takeoff_t, 'liftoff_t': liftoff_t,
              'penalty': penalty, 'clash': clash, 'work': work,
              'spring_J': spring0, 'spring_work': spring_work,
              'energy_error': energy_error, 'peak_force': peak_force,
              'peak_torque': peak_torque}
    if record:
      result['traj'] = traj
    return result

  def spring_physical(self, z):
    """Unit spring parameters (B, 14) -> energy, travel, exponent (B, 4)
    and release time (B, 2)."""
    z = np.clip(z, -1, 1)
    share = (z[:, 0:4] + 1) / 2 * self.spring_mask[None]
    energy = (self.spring_budget * share /
              np.maximum(share.sum(axis=1, keepdims=True), 1e-9))
    travel = MAX_TRAVEL * z[:, 4:8]
    travel = np.where(travel >= 0, 1, -1) * np.maximum(np.abs(travel), 0.1)
    return {'energy': energy, 'travel': travel, 'exponent': z[:, 8:12] + 1,
            'release': (z[:, 12:14] + 1) / 2 * MAX_RELEASE}

  @staticmethod
  def spring_torque(sp, latch, i, q, t):
    """Torque (B, 2) of limb i's springs at joint angles q, time t."""
    j = slice(2 * i, 2 * i + 2)
    newly = (t >= sp['release'][:, i]) & ~latch['released'][:, i]
    latch['q_rel'][i][newly] = q[newly]
    latch['released'][newly, i] = True
    d, n = sp['travel'][:, j], sp['exponent'][:, j]
    a = np.abs(d)
    tau0 = sp['energy'][:, j] * (n + 1) / a
    progress = np.sign(d) * (q - latch['q_rel'][i]) / a
    left = np.clip(1 - progress, 0, 2)
    tau = np.sign(d) * tau0 * left ** n * (progress < 1)
    return tau * latch['released'][:, i:i + 1]

  def spring_unit_default(self):
    """Equal shares, 1.5 rad travel toward the far end of each joint's
    range (the legacy springs' direction), linear, released at once."""
    z = np.zeros(14)
    for i, limb in enumerate(self.limbs):
      far = np.where(np.abs(limb.q_hi - limb.q_crouch) >
                     np.abs(limb.q_lo - limb.q_crouch), 1, -1)
      z[4 + 2 * i:6 + 2 * i] = 0.5 * far
    z[12:14] = -1
    return z

  def spring_energy(self, q, geom):
    e = 0.0
    for g, qi in zip(geom, q):
      e = e + 0.5 * np.sum(g['k'] * (qi - g['rest']) ** 2, axis=1)
    return e

  def score(self, res):
    v = res['v']
    along = v @ self.direction
    perp = np.abs(v[:, 0] * self.direction[1] - v[:, 1] * self.direction[0])
    return along - perp - 10 * res['penalty']

  def optimize(self, iters=80, pop=256, elite=24, seed=0, verbose=False,
               init=None, crouch=None, search_crouch=False, spring_search=None):
    """Cross-entropy optimization of the joint commands; init: warm start
    (4, knots) commands. crouch: fixed crouch (4,), default the reference's.
    search_crouch: also optimize the crouch, starting from `crouch`.
    spring_search: designable springs: dict with 'energy' (J budget) and
    optionally 'mask' (4 joints), 'shape' (free exponent), 'staged' (free
    release times), 'init' (14 unit parameters), 'freeze' (keep init)."""
    rng = np.random.default_rng(seed)
    crouch = self.reference_crouch if crouch is None else np.asarray(crouch)
    nk = 4 * self.knots
    nc = 4 if search_crouch else 0
    search = spring_search is not None
    if search:
      self.spring_budget = spring_search['energy']
      self.spring_mask = np.asarray(spring_search.get('mask', np.ones(4)), float)
    ns = 14 if search else 0
    size = nk + nc + ns
    mean = np.zeros(size)
    std = np.full(size, 0.8)
    if init is not None:
      mean[:nk] = init.ravel()
      std[:nk] = 0.3
    if nc:
      mean[nk:nk + nc] = self.unit_from_crouch(crouch)
      std[nk:nk + nc] = 0.4
    if ns:
      z = np.asarray(spring_search.get('init', self.spring_unit_default()), float)
      mean[nk + nc:] = z
      std[nk + nc:] = 0.5
      std[nk + nc:nk + nc + 4] *= self.spring_mask   # masked shares stay put
      if not spring_search.get('shape'):
        mean[nk + nc + 8:nk + nc + 12], std[nk + nc + 8:nk + nc + 12] = 0, 0
      if not spring_search.get('staged'):
        mean[nk + nc + 12:], std[nk + nc + 12:] = -1, 0
      if spring_search.get('freeze'):
        std[nk + nc:] = 0
    seeds = [np.r_[s.ravel(), mean[nk:]] for s in self.seeds]
    fixed_std = std == 0
    best, best_score = None, -np.inf

    def split(x):
      ctrl = x[:, :nk].reshape(-1, 4, self.knots)
      cr = (self.crouch_from_unit(x[:, nk:nk + nc]) if nc
            else np.repeat(crouch[None], len(x), axis=0))
      sp = x[:, nk + nc:] if ns else None
      return ctrl, cr, sp

    for it in range(iters):
      x = np.clip(mean + std * rng.standard_normal((pop, size)), -1, 1)
      if best is not None:
        x[0] = best
      elif init is not None:
        x[0] = mean
      else:
        x[:len(seeds)] = seeds
      ctrl, crouches, sp = split(x)
      res = self.simulate(ctrl, crouch=crouches, springs=sp)
      s = self.score(res)
      order = np.argsort(s)[::-1]
      if s[order[0]] > best_score:
        best_score, best = s[order[0]], x[order[0]].copy()
      e = x[order[:elite]]
      mean = 0.3 * mean + 0.7 * e.mean(axis=0)
      std = np.maximum(0.3 * std + 0.7 * e.std(axis=0), 0.02)
      std[fixed_std] = 0
      if verbose and it % 10 == 0:
        print(f'  iter {it:3d} best {best_score:.3f}', flush=True)
    ctrl, best_crouch, sp = split(best[None])
    res = self.simulate(ctrl, crouch=best_crouch, springs=sp)
    v = res['v'][0]
    out = {
        'params': ctrl[0],
        'crouch': dict(zip(CROUCH_NAMES, best_crouch[0].round(4).tolist())),
        'speed': float(np.linalg.norm(v)),
        'angle_deg': float(np.degrees(np.arctan2(v[1], -v[0]))),
        'speed_along': float(v @ self.direction),
        'takeoff_time': float(res['t'][0]),
        'liftoff_hind_fore': res['liftoff_t'][0].round(3).tolist(),
        'work_J': float(res['work'][0]),
        'spring_J': float(res['spring_J'][0]),
        'spring_work_J': float(res['spring_work'][0]),
        'kinetic_energy_J': float(0.5 * self.mass * v @ v),
        'energy_error': float(res['energy_error'][0]),
        'peak_force_N': float(res['peak_force'][0]),
        'peak_torque_Nm': res['peak_torque'][0].round(0).tolist(),
        'infeasible': bool(res['clash'][0]),
    }
    if sp is not None:
      out['spring_unit'] = sp[0].tolist()
      phys = self.spring_physical(sp)
      out['springs'] = {k: v[0].round(3).tolist() for k, v in phys.items()}
    return out

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
