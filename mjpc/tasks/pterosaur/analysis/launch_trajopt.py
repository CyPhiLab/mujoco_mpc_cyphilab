#!/usr/bin/env python3
"""Offline trajectory optimization of a ground-powered pterosaur launch.

Optimizes an open-loop, left/right symmetric control sequence from the
reference's deepest crouch through takeoff to maximize the CoM takeoff
velocity along a launch direction, subject to (as penalties):
  - takeoff spin (whole-body angular momentum) and body pitch,
  - knees, shins and forearms staying clear of the ground (push on feet),
  - calm limbs after takeoff (no flailing).

The optimizer is sampling-based (smooth control perturbations, elite
averaging) around a warm start: the reference joint motion, time-compressed
to several push durations and tracked with a joint PD controller, recorded
as open-loop controls.

Usage:
  launch_trajopt.py [--torque 2] [--angle 30] [--iters 150] [--out result.npz]
"""
import argparse
import copy
import os
import time

import mujoco
import numpy as np
from mujoco import rollout

HERE = os.path.dirname(os.path.abspath(__file__))
TASK_DIR = os.path.dirname(HERE)

SIM_DT = 0.002
HORIZON = 1.0           # seconds from the deepest crouch
JOINT_MARGIN = 0.3      # hard joint limits: reference range +- margin
MIN_FLIGHT = 0.15       # airborne this long counts as takeoff
KNOT_DT = 0.04          # control perturbation knot spacing
# MuJoCo default contact impedance for the feet: the model's soft feet
# (solimp width 3.1 cm) sink onto the shins; much stiffer (1 mm) bounces
FOOT_SOLIMP = [0.9, 0.95, 0.001]
FOOT_SOLREF = [0.02, 1]

# left/right mirror sign per limb joint (shoulder1 axes are mirrored)
MIRROR = np.array([-1, 1, 1, 1, 1, 1])
LEFT = np.array([0, 1, 2, 6, 7, 8])
RIGHT = np.array([3, 4, 5, 9, 10, 11])

# clearance points (body, point in body frame, minimum height above floor);
# reference minima: knee 18, mid-shin 10, shin tip 5.0, mid-forearm 21,
# forearm tip 9.2 cm
CLEARANCE = [
    ('tibia', [0, 0, 0], 0.12), ('tibia_2', [0, 0, 0], 0.12),
    ('tibia', [0.184, 0, 0], 0.06), ('tibia_2', [0.184, 0, 0], 0.06),
    ('tibia', [0.30, 0, 0], 0.035), ('tibia_2', [0.30, 0, 0], 0.035),
    ('radius_and_ulna', [-0.34, 0, -0.01], 0.15),
    ('radius_and_ulna_2', [-0.34, 0, -0.01], 0.15),
    ('radius_and_ulna', [-0.55, 0, -0.02], 0.06),
    ('radius_and_ulna_2', [-0.55, 0, -0.02], 0.06),
]
FLOOR_Z = -1.0

# objective weights
W_PERP = 1.0        # off-direction takeoff velocity (per m/s)
W_SPIN = 5.0        # takeoff angular momentum per unit mass (per m^2/s)
W_PITCH = 5.0       # body pitch beyond PITCH_MAX (per rad)
PITCH_MAX = np.deg2rad(45)
W_CLEAR = 500.0     # clearance violation integrated over time (per m s)
W_CALM = 0.5        # mean joint speed after takeoff (per rad/s)
W_EFFORT = 0.1      # mean squared control
W_SMOOTH = 20.0     # mean control change per step (per unit per 2 ms)
W_TAP = 0.5         # each hand/foot liftoff before takeoff (tapping)


def reference():
  return np.load(os.path.join(TASK_DIR, 'reference', 'launch.npz'))


def load_model(torque):
  bodies = sorted({b for b, _, _ in CLEARANCE})
  frames = ''.join(
      f'<framepos name="to_pos_{b}" objtype="xbody" objname="{b}"/>'
      f'<framexaxis name="to_x_{b}" objtype="xbody" objname="{b}"/>'
      f'<framezaxis name="to_z_{b}" objtype="xbody" objname="{b}"/>'
      for b in bodies)
  extra = f"""<mujoco>
  <include file="task.xml"/>
  <sensor>
    <contact name="to_floor" subtree1="body" geom2="floor" num="1" data="found"/>
    <contact name="to_hand_l" body1="radius_and_ulna" geom2="floor" num="1" data="found"/>
    <contact name="to_hand_r" body1="radius_and_ulna_2" geom2="floor" num="1" data="found"/>
    <contact name="to_foot_l" body1="tibia" geom2="floor" num="1" data="found"/>
    <contact name="to_foot_r" body1="tibia_2" geom2="floor" num="1" data="found"/>
    <subtreeangmom name="to_angmom" body="body"/>
    <framexaxis name="to_body_x" objtype="xbody" objname="body"/>
    {frames}
  </sensor>
</mujoco>"""
  path = os.path.join(TASK_DIR, '_trajopt_tmp.xml')
  with open(path, 'w') as f:
    f.write(extra)
  try:
    m = mujoco.MjModel.from_xml_path(path)
  finally:
    os.remove(path)
  ref = reference()
  q = ref['qpos'][:, 7:]
  lo, hi = q.min(axis=0) - JOINT_MARGIN, q.max(axis=0) + JOINT_MARGIN
  m.opt.timestep = SIM_DT
  m.actuator_gainprm[:, 0] *= torque
  for j in range(12):
    m.jnt_limited[j + 1] = 1
    m.jnt_range[j + 1] = [lo[j], hi[j]]
  for name in ['FL', 'FR', 'HL', 'HR']:
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
    m.geom_solimp[g, :3] = FOOT_SOLIMP
    m.geom_solref[g, :2] = FOOT_SOLREF
  return m


def sensor(m, name):
  i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
  return slice(m.sensor_adr[i], m.sensor_adr[i] + m.sensor_dim[i])


def push_start_index(ref):
  airborne = np.flatnonzero(~ref['hands_contact'] & ~ref['feet_contact'])
  return int(np.argmin(ref['qpos'][:airborne[0], 2]))


def symmetrize(ctrl):
  """Full (.., 12) controls -> (.., 6) symmetric left-side channels."""
  return 0.5 * (ctrl[..., LEFT] + MIRROR * ctrl[..., RIGHT])


def expand(half):
  """(.., 6) symmetric channels -> (.., 12) controls."""
  ctrl = np.empty(half.shape[:-1] + (12,))
  ctrl[..., LEFT] = half
  ctrl[..., RIGHT] = MIRROR * half
  return np.clip(ctrl, -1, 1)


class LaunchOpt:

  def __init__(self, torque=2.0, angle=30.0, nthread=4):
    self.m = load_model(torque)
    self.ref = reference()
    self.i0 = push_start_index(self.ref)
    self.nstep = int(round(HORIZON / SIM_DT))
    self.datas = [mujoco.MjData(self.m) for _ in range(nthread)]
    a = np.deg2rad(angle)
    self.direction = np.array([-np.cos(a), 0, np.sin(a)])
    self.mass = self.m.body_subtreemass[1]
    m = self.m
    self.s = {n: sensor(m, n) for n in
              ['to_floor', 'to_angmom', 'to_body_x', 'torso_subtreelinvel',
               'torso_subtreecom', 'to_hand_l', 'to_hand_r', 'to_foot_l',
               'to_foot_r']}
    for b in {b for b, _, _ in CLEARANCE}:
      for k in ('pos', 'x', 'z'):
        self.s[f'{k}_{b}'] = sensor(m, f'to_{k}_{b}')
    self.x0 = self.initial_state()

  def initial_state(self):
    m, d = self.m, self.datas[0]
    mujoco.mj_resetData(m, d)
    d.qpos[:] = self.ref['qpos'][self.i0]
    d.qvel[:] = self.ref['qvel'][self.i0]
    mujoco.mj_forward(m, d)
    spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
    x0 = np.empty(mujoco.mj_stateSize(m, spec))
    mujoco.mj_getState(m, d, x0, spec)
    return x0

  # ----- warm starts ----- #
  def pd_warm_start(self, push_time, kp=600.0, kd=20.0):
    """Track the reference joints with the push compressed to push_time;
    return the recorded symmetric controls (nstep, 6)."""
    m = self.m
    d = mujoco.MjData(m)
    mujoco.mj_setState(m, d, self.x0, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    mujoco.mj_forward(m, d)
    ref_q, ref_dt = self.ref['qpos'][:, 7:], self.ref['time'][1]
    t_push = self.ref['time'][self.i0]
    airborne = np.flatnonzero(~self.ref['hands_contact'] & ~self.ref['feet_contact'])
    t_takeoff = self.ref['time'][airborne[0]]
    rate = (t_takeoff - t_push) / push_time
    gain = m.actuator_gainprm[:, 0]
    half = np.empty((self.nstep, 6))
    for k in range(self.nstep):
      t = k * SIM_DT
      tr = t_push + (t * rate if t < push_time else (t_takeoff - t_push) + t - push_time)
      i = min(int(tr / ref_dt), len(ref_q) - 2)
      w = tr / ref_dt - i
      q_ref = (1 - w) * ref_q[i] + w * ref_q[i + 1]
      v_ref = (ref_q[i + 1] - ref_q[i]) / ref_dt * (rate if t < push_time else 1)
      u = (kp * (q_ref - d.qpos[7:]) + kd * (v_ref - d.qvel[6:])) / gain
      half[k] = symmetrize(np.clip(u, -1, 1))
      d.ctrl[:] = expand(half[k])
      mujoco.mj_step(m, d)
    return half

  # ----- evaluation ----- #
  def simulate(self, halves):
    ctrl = expand(halves)
    n = ctrl.shape[0]
    state, sens = rollout.rollout(self.m, self.datas, np.tile(self.x0, (n, 1)), ctrl)
    return ctrl, state, sens

  def score(self, ctrl, state, sens, detail=False):
    m = self.m
    n = ctrl.shape[0]
    nq, nv = m.nq, m.nv
    qvel = state[:, :, 1 + nq:1 + nq + nv]
    found = sens[:, :, self.s['to_floor']][:, :, 0]
    comvel = sens[:, :, self.s['torso_subtreelinvel']]
    angmom = sens[:, :, self.s['to_angmom']]
    body_x = sens[:, :, self.s['to_body_x']]
    need = int(round(MIN_FLIGHT / SIM_DT))
    calm_steps = int(round(0.3 / SIM_DT))

    # clearance heights (n, nstep, points)
    heights = []
    for b, p, _ in CLEARANCE:
      pos = sens[:, :, self.s[f'pos_{b}']]
      xa = sens[:, :, self.s[f'x_{b}']]
      za = sens[:, :, self.s[f'z_{b}']]
      heights.append(pos[..., 2] + p[0] * xa[..., 2] + p[2] * za[..., 2] - FLOOR_Z)
    heights = np.stack(heights, axis=-1)
    thresholds = np.array([c for _, _, c in CLEARANCE])
    violation = np.maximum(thresholds - heights, 0).sum(axis=-1)  # (n, nstep)
    pitch = np.abs(np.arcsin(np.clip(body_x[..., 2], -1, 1)))    # (n, nstep)
    limbs = np.stack([sens[:, :, self.s[k]][:, :, 0] > 0 for k in
                      ('to_hand_l', 'to_hand_r', 'to_foot_l', 'to_foot_r')],
                     axis=-1)                                     # (n, nstep, 4)
    liftoffs = limbs[:, :-1] & ~limbs[:, 1:]                       # (n, nstep-1, 4)

    scores = np.empty(n)
    details = []
    for i in range(n):
      if not np.all(np.isfinite(comvel[i])) or np.abs(comvel[i]).max() > 30:
        scores[i] = -100
        details.append({'blowup': True})
        continue
      # takeoff: first step of an airborne run lasting MIN_FLIGHT
      air = found[i] == 0
      k = None
      run = 0
      for j in range(self.nstep):
        run = run + 1 if air[j] else 0
        if run >= need:
          k = j - need + 1
          break
      took_off = k is not None
      if not took_off:
        k = self.nstep - 1
      v = comvel[i, k]
      along = v @ self.direction
      perp = np.linalg.norm(v - along * self.direction)
      spin = np.linalg.norm(angmom[i, k]) / self.mass
      window = slice(0, min(k + calm_steps, self.nstep))
      pitch_excess = np.maximum(pitch[i, window] - PITCH_MAX, 0).max()
      clear = violation[i, :k + 1].sum() * SIM_DT
      calm = np.abs(qvel[i, k:k + calm_steps, 6:]).mean() if took_off else 0
      effort = np.mean(ctrl[i] ** 2)
      jitter = np.abs(np.diff(ctrl[i], axis=0)).mean()
      # each limb should lift off once: extra liftoffs are taps/bounces
      taps = max(int(liftoffs[i, :k].sum()) - 4, 0)
      s = (along - W_PERP * perp - W_SPIN * spin - W_PITCH * pitch_excess
           - W_CLEAR * clear - W_CALM * calm - W_EFFORT * effort
           - W_SMOOTH * jitter - W_TAP * taps)
      if not took_off:
        s -= 5
      scores[i] = s
      if detail:
        details.append({
            'took_off': took_off, 'takeoff_time': k * SIM_DT,
            'speed': float(np.linalg.norm(v)),
            'angle_deg': float(np.degrees(np.arctan2(v[2], np.hypot(v[0], v[1])))),
            'spin_per_mass': float(spin), 'max_pitch_deg': float(np.degrees(pitch[i, window].max())),
            'clearance_violation_ms': float(clear), 'calm_rad_s': float(calm),
            'effort': float(effort), 'jitter': float(jitter), 'taps': taps,
            'score': float(s)})
    return (scores, details) if detail else scores

  def evaluate(self, halves, detail=False):
    ctrl, state, sens = self.simulate(halves)
    return self.score(ctrl, state, sens, detail)

  # ----- optimization ----- #
  def optimize(self, init, iters=150, pop=192, elite=16, sigma0=0.25,
               sigma_min=0.03, seed=0, verbose=True):
    rng = np.random.default_rng(seed)
    nknot = int(round(HORIZON / KNOT_DT)) + 1
    t_knot = np.linspace(0, 1, nknot)
    t = np.linspace(0, 1, self.nstep)
    base = init.copy()
    best, best_score = base.copy(), self.evaluate(base[None])[0]
    sigma = sigma0
    history = []
    for it in range(iters):
      knots = sigma * rng.standard_normal((pop, nknot, 6))
      noise = np.empty((pop, self.nstep, 6))
      for c in range(6):
        noise[:, :, c] = np.array([np.interp(t, t_knot, knots[p, :, c]) for p in range(pop)])
      noise[0] = 0  # keep the current base in the population
      samples = np.clip(base + noise, -1, 1)
      scores = self.evaluate(samples)
      order = np.argsort(scores)[::-1]
      if scores[order[0]] > best_score:
        best_score, best = scores[order[0]], samples[order[0]].copy()
      step = (samples[order[:elite]] - base).mean(axis=0)
      base = np.clip(base + step, -1, 1)
      spread = scores[order[:elite]].std()
      sigma = max(sigma_min, sigma * (0.97 if spread < 0.05 else 0.99))
      history.append(float(best_score))
      if verbose and (it % 10 == 0 or it == iters - 1):
        print(f'  iter {it:3d} best {best_score:.3f} (pop best {scores[order[0]]:.3f}, sigma {sigma:.3f})',
              flush=True)
    return best, best_score, history

  def record(self, halves, path):
    """Simulate a control sequence and save the trajectory (npz)."""
    m = self.m
    d = mujoco.MjData(m)
    mujoco.mj_setState(m, d, self.x0, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    mujoco.mj_forward(m, d)
    ctrl = expand(halves)
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
    hand_geoms = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in ['FL', 'FR']}
    foot_geoms = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in ['HL', 'HR']}
    out = {k: [] for k in ['time', 'qpos', 'qvel', 'ctrl', 'hands_contact',
                           'feet_contact', 'any_contact', 'comvel']}
    for k in range(self.nstep):
      d.ctrl[:] = ctrl[k]
      mujoco.mj_step(m, d)
      geoms = {c.geom2 if c.geom1 == floor else c.geom1
               for c in d.contact[:d.ncon] if floor in (c.geom1, c.geom2) and c.dist <= 0}
      out['time'].append(d.time)
      out['qpos'].append(d.qpos.copy())
      out['qvel'].append(d.qvel.copy())
      out['ctrl'].append(ctrl[k].copy())
      out['hands_contact'].append(bool(geoms & hand_geoms))
      out['feet_contact'].append(bool(geoms & foot_geoms))
      out['any_contact'].append(bool(geoms))
      out['comvel'].append(d.subtree_linvel[1].copy())
    out = {k: np.array(v) for k, v in out.items()}
    np.savez(path, **out, start_qpos=self.ref['qpos'][self.i0],
             start_time=self.ref['time'][self.i0])
    return out


def smooth(half, window=0.03):
  """Moving-average low-pass of a control sequence (nstep, 6)."""
  n = max(1, int(round(window / SIM_DT)))
  kernel = np.ones(n) / n
  padded = np.pad(half, ((n // 2, n - 1 - n // 2), (0, 0)), mode='edge')
  return np.stack([np.convolve(padded[:, c], kernel, mode='valid')
                   for c in range(half.shape[1])], axis=1)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--torque', type=float, default=2.0)
  parser.add_argument('--angle', type=float, default=30.0)
  parser.add_argument('--iters', type=int, default=150)
  parser.add_argument('--out', default='launch_trajopt.npz')
  parser.add_argument('--init', default=None,
                      help='refine from saved controls (*_controls.npy)')
  args = parser.parse_args()

  opt = LaunchOpt(args.torque, args.angle)
  if args.init:
    init = smooth(np.load(args.init))
    _, det = opt.evaluate(init[None], detail=True)
    print(f'refining from {args.init} (smoothed): {det[0]}', flush=True)
    best, _, _ = opt.optimize(init, iters=args.iters, sigma0=0.1)
    _, det = opt.evaluate(best[None], detail=True)
    print(f'done: {det[0]}', flush=True)
    opt.record(best, args.out)
    np.save(os.path.splitext(args.out)[0] + '_controls.npy', best)
    print('wrote', args.out)
    return

  print(f'start at reference t={opt.ref["time"][opt.i0]:.2f}s, horizon {HORIZON}s')

  # warm starts: reference push compressed to several durations
  candidates = {}
  for push_time in [0.3, 0.4, 0.5, 0.7, 1.06]:
    half = opt.pd_warm_start(push_time)
    _, det = opt.evaluate(half[None], detail=True)
    candidates[push_time] = (half, det[0])
    print(f'warm start push {push_time:.2f}s: {det[0]}')
  ranked = sorted(candidates, key=lambda p: candidates[p][1].get('score', -1e9),
                  reverse=True)
  best, best_score = None, -np.inf
  for init_time in ranked[:2]:
    print(f'optimizing from push {init_time}s warm start', flush=True)
    t0 = time.time()
    half, score, _ = opt.optimize(candidates[init_time][0], iters=args.iters)
    _, det = opt.evaluate(half[None], detail=True)
    print(f'done in {time.time() - t0:.0f}s: {det[0]}', flush=True)
    if score > best_score:
      best, best_score = half, score
  opt.record(best, args.out)
  np.save(os.path.splitext(args.out)[0] + '_controls.npy', best)
  print('wrote', args.out)


if __name__ == '__main__':
  main()
