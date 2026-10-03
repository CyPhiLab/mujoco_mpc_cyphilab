#!/usr/bin/env python3
"""Fluid launch by tracking a retimed reference, then speeding it up.

The reference push (deepest crouch to takeoff) is retimed per limb: the arm
joints run the reference's crouch-to-hand-liftoff motion over the push time
T, the leg joints run its crouch-to-foot-liftoff motion over T - FOOT_LEAD,
so the feet leave FOOT_LEAD before the hands instead of 0.35 s (reference).
launch_ilqr.py's iLQR then optimizes with that contact schedule (limbs
planted until their liftoff, nothing touching after), a joint and body
pitch tracking cost on the retimed reference, and a takeoff velocity
target. A continuation chain shortens the push and raises the target speed
step by step, each step from the previous solution.

Usage:
  launch_track.py [--torque 6 --spring_energy 500] [--out tracked]
"""
import argparse
import json
import os
import time

import mujoco
import numpy as np

import launch_ilqr as L
import launch_trajopt as T

HERE = os.path.dirname(os.path.abspath(__file__))
FOOT_LEAD = 0.025          # s, reference feet liftoff before the hands
FOOT_FREE = 0.1            # s, feet must stay planted until this long before
                           # the end of the push, and may leave any time after
S_TRACK, W_TRACK = 0.15, 1.0       # joint tracking (rad)
S_TRACK_PITCH, W_TRACK_PITCH = 0.1, 1.0   # body pitch tracking (rad)
ARM = np.array([0, 1, 2, 3, 4, 5])
LEG = np.array([6, 7, 8, 9, 10, 11])

# planted limbs and no re-contact must outweigh the takeoff speed target:
# with launch_ilqr.py's weights a 1 cm hand lift held for 0.3 s costs ~0.6,
# a 1 m/s takeoff speed error 20, so the optimizer lifted the hands early
L.W_PLANT_H, L.W_PLANT_V = 50.0, 10.0
L.W_LIFT = 20.0

# continuation: (push time s, takeoff speed m/s at 30 deg)
CHAIN = [(0.6, 3.0), (0.55, 4.0), (0.5, 5.0), (0.47, 6.0), (0.45, 7.0),
         (0.45, 8.0)]
# second chain (--chain relax, from the first chain's last step): the
# reference tracking is loosened step by step (push time, speed, tracking
# weight scale); planted limbs, no re-contact and pitch limits stay
RELAX = [(0.45, 8.0, 0.3), (0.45, 8.0, 0.1), (0.45, 9.0, 0.03),
         (0.45, 10.0, 0.03)]
# --chain push: same push time, higher targets (with --track_scale and
# --min_hand_force to get the arms to bear load)
PUSH = [(0.45, 8.0, 1.0), (0.45, 9.0, 1.0)]
# --chain bodypath (with --reference bodypath): the forelimb-push reference
# at increasing speeds, from a PD warm start on it
BODYPATH = [(0.45, v, 1.0) for v in (6.0, 7.0, 8.0, 9.0, 10.0)]
# --chain bodypath_real: realistic motors (--torque 2 --no_load_speed 20)
# with designed springs, at increasing speeds from a PD warm start
BODYPATH_REAL = [(0.45, v, 1.0) for v in (3.0, 4.0, 5.0, 6.0, 7.0)]
# --chain relax_real: from a bodypath_real solution (--init), the tracking
# loosened step by step at increasing speeds
RELAX_REAL = [(0.45, 4.0, 0.3), (0.45, 4.0, 0.1), (0.45, 5.0, 0.1),
              (0.45, 6.0, 0.1)]

# Designed latched springs (per side), from the joint work in the best clean
# launch with ideal 6x motors (tracked_C_bodypath step 1: shoulder2 336 J,
# elbow 207 J, hip2 252 J, knee 369 J of positive work per side): each joint's
# share of that work, released when the joint starts its 5-95% work phase,
# travelling its excursion over that phase (joint angle at its start and end).
#   joint (left/right qpos[7:] index), work share, release s, q start, q end
SPRING_DESIGN = [
    ('shoulder2', (1, 4), 336 / 1164, 0.29, -1.162, 0.693),
    ('elbow', (2, 5), 207 / 1164, 0.40, 0.903, -0.112),
    ('hip2', (7, 10), 252 / 1164, 0.21, -0.880, 0.612),
    ('knee', (8, 11), 369 / 1164, 0.29, -1.000, 0.387),
]
SPRING_RAMP = 0.3   # last fraction of the travel over which the torque ramps
                    # to zero (near-constant torque, then a stop)
JOINT_NAMES = ['leftarm_shoulder1', 'leftarm_shoulder2', 'leftarm_elbow',
               'rightarm_shoulder1', 'rightarm_shoulder2', 'rightarm_elbow',
               'leftleg_hip1', 'leftleg_hip2', 'leftleg_knee',
               'rightleg_hip1', 'rightleg_hip2', 'rightleg_knee']


# spring hardware mount (point mass) per spring joint: the proximal end of
# the segment the spring rides on (shoulder/hip housing, top of the
# humerus/femur with a cable to the elbow/knee), keeping limb inertia low
SPRING_MOUNT = {'leftarm_shoulder2': 'smaller_housing',
                'rightarm_shoulder2': 'smaller_housing_2',
                'leftarm_elbow': 'bevel_out', 'rightarm_elbow': 'bevel_out_2',
                'leftleg_hip2': 'leg_motor_1', 'rightleg_hip2': 'hip',
                'leftleg_knee': 'leg_motor_2', 'rightleg_knee': 'femur'}


def designed_springs(budget, design=SPRING_DESIGN, ramp=SPRING_RAMP,
                     efficiency=1.0, mass=0.0):
  """Latched spring actuators (launch_trajopt.add_latched_springs) storing
  budget J over both sides, and their release times. ramp: fraction of the
  travel over which the torque ramps to zero (1 = linear spring);
  efficiency: fraction of the stored energy returned (hysteresis); mass:
  spring hardware kg over both sides, split by energy, as point masses
  (launch_trajopt.add_point_masses)."""
  springs, times, masses = [], [], []
  for name, sides, share, release, q0, q1 in design:
    energy = budget * share / 2
    a = abs(q1 - q0)
    tau0 = efficiency * energy / (a * (1 - ramp / 2))
    k = tau0 / (ramp * a)
    for side in sides:
      springs.append({'joint': JOINT_NAMES[side], 'k': k, 'rest': q1,
                      'tau0': float(np.sign(q1 - q0) * tau0)})
      times.append(release)
      if mass > 0:
        masses.append((SPRING_MOUNT[JOINT_NAMES[side]], mass * share / 2))
  return springs, times, masses


# minimum normal force per hand while planted (residual scale and weight)
S_FMIN, W_FMIN = 50.0, 20.0


def pitch_of(quat):
  w, x, y, z = quat
  return np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))


def retimed_reference(push_time, dt, n):
  """Joint angles (n, 12) and body pitch (n,) of the retimed reference push
  (and its flight motion afterward, at real time)."""
  ref = T.reference()
  i0 = T.push_start_index(ref)
  t_ref, q_ref = ref['time'], ref['qpos']
  t0 = t_ref[i0]
  t_hands = t_ref[i0 + int(np.argmin(ref['hands_contact'][i0:]))]
  t_feet = t_ref[i0 + int(np.argmin(ref['feet_contact'][i0:]))]
  t = np.arange(n) * dt

  def warp(t_end_ref, t_end):
    # crouch -> liftoff over [0, t_end], then the reference at real time
    return np.where(t < t_end, t0 + (t_end_ref - t0) * t / t_end,
                    t_end_ref + (t - t_end))

  joints = np.empty((n, 12))
  for idx, (t_end_ref, t_end) in ((ARM, (t_hands, push_time)),
                                  (LEG, (t_feet, push_time - FOOT_LEAD))):
    tau = warp(t_end_ref, t_end)
    for j in idx:
      joints[:, j] = np.interp(tau, t_ref, q_ref[:, 7 + j])
  tau = warp(t_hands, push_time)
  pitch = np.interp(tau, t_ref, [pitch_of(q[3:7]) for q in q_ref])
  return joints, pitch


# limbs for the body-path reference: contact geom and its three hinges
# (abduction, proximal, middle; qpos[7:] indices). Abduction is needed: the
# limbs do not move in the sagittal plane, so a planted hand drifts sideways
# 1.5-2 cm without it
PATH_LIMBS = [('FL', (0, 1, 2)), ('FR', (3, 4, 5)), ('HL', (6, 7, 8)),
              ('HR', (9, 10, 11))]


def limb_ik(m, d, geom, hinges, target, iters=20, tol=0.003):
  """Move the hinges so the geom center reaches target (from the current
  qpos, damped least squares); returns True if reached within tol."""
  jac = np.zeros((3, m.nv))
  qadr = [7 + h for h in hinges]
  dadr = [6 + h for h in hinges]
  jnt = [h + 1 for h in hinges]
  for _ in range(iters):
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d)
    err = d.geom_xpos[geom] - target
    if np.linalg.norm(err) < 0.001:
      break
    mujoco.mj_jacGeom(m, d, jac, None, geom)
    J = jac[:, dadr]
    dq = -np.linalg.solve(J.T @ J + 1e-4 * np.eye(len(hinges)), J.T @ err)
    for k, (qa, j) in enumerate(zip(qadr, jnt)):
      lo, hi = m.jnt_range[j]
      d.qpos[qa] = np.clip(d.qpos[qa] + dq[k], lo, hi)
  mujoco.mj_kinematics(m, d)
  return np.linalg.norm(d.geom_xpos[geom] - target) < tol


def body_path_reference(m, q0, push_time, dt, n, angle_deg=30.0,
                        ref_speed=7.0, stroke_frac=0.95, follow_pitch=False):
  """Kinematic push: the body translates along the launch direction by
  d(t) = s (t/T)^p (at the crouch pitch, or with follow_pitch the retimed
  reference's pitch change, which noses up ~17 deg and pulls the hands out
  of reach), the sagittal limb
  joints keep each hand/foot on its starting spot by IK until the limb is
  out of reach (then that limb freezes and lifts off); abduction joints
  follow the retimed reference. s: stroke_frac of the hands' reach along
  the launch direction; p sets the reference's final speed to ref_speed.
  Returns joints (n, 12), pitch (n,), per-limb liftoff times, stroke, p."""
  joints_ret, pitch_ret = retimed_reference(push_time, dt, n)
  if not follow_pitch:
    pitch_ret = np.full_like(pitch_ret, pitch_of(q0[3:7]))
  d = mujoco.MjData(m)
  d.qpos[:] = q0
  mujoco.mj_kinematics(m, d)
  gid = {g: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, g) for g, _ in PATH_LIMBS}
  targets = {g: d.geom_xpos[gid[g]].copy() for g, _ in PATH_LIMBS}
  a = np.deg2rad(angle_deg)
  direction = np.array([-np.cos(a), 0.0, np.sin(a)])
  quat0 = q0[3:7].copy()

  def pose(dist, dpitch, q_prev):
    q = q_prev.copy()
    q[:3] = q0[:3] + dist * direction
    half = 0.5 * dpitch
    mujoco.mju_mulQuat(q[3:7], np.array([np.cos(half), 0, np.sin(half), 0]), quat0)
    return q

  def sweep(stroke, p):
    """Joints along the path and per-limb liftoff times."""
    n_push = int(round(push_time / dt))
    joints = joints_ret.copy()
    q = q0.copy()
    frozen = {g: False for g, _ in PATH_LIMBS}
    liftoff = {g: push_time for g, _ in PATH_LIMBS}
    for k in range(n_push + 1):
      t = k * dt
      q = pose(stroke * (t / push_time) ** p, pitch_ret[k] - pitch_ret[0], q)
      d.qpos[:] = q
      for g, h in PATH_LIMBS:
        if frozen[g]:
          continue
        if not limb_ik(m, d, gid[g], h, targets[g]):
          frozen[g] = True
          liftoff[g] = t
          for hh in h:                          # keep the last reached pose
            d.qpos[7 + hh] = joints[k - 1, hh] if k else q0[7 + hh]
      q = d.qpos.copy()
      joints[k] = q[7:]
    joints[n_push + 1:] = joints[n_push]
    return joints, liftoff

  # stroke: the longest path along which both hands stay on their spots
  # until the end of the push (pitch included)
  lo_s, hi_s = 0.0, 1.5
  for _ in range(12):
    mid = 0.5 * (lo_s + hi_s)
    _, lift = sweep(mid, max(1.0, ref_speed * push_time / mid))
    ok = min(lift['FL'], lift['FR']) >= push_time - 1e-9
    lo_s, hi_s = (mid, hi_s) if ok else (lo_s, mid)
  stroke = stroke_frac * lo_s
  p = max(1.0, ref_speed * push_time / stroke)
  joints, liftoff = sweep(stroke, p)
  pitch = pitch_ret.copy()
  return joints, pitch, liftoff, stroke, p


class TrackILQR(L.LaunchILQR):
  """LaunchILQR plus tracking of the retimed reference during the push."""

  def __init__(self, push_time, torque, speed, track_scale=1.0,
               min_hand_force=0.0, reference='retimed', window=None,
               free_hands=False, **kw):
    """push_time: the reference push; window: the optimized push (>=
    push_time; the reference holds its last pose after push_time and the
    takeoff velocity is scored ballistically, see LaunchILQR.z_ref);
    free_hands: hands planted only while the feet are (then free to push
    or leave whenever), not until the end of the push."""
    self.track_scale = track_scale
    self.min_hand_force = min_hand_force
    window = max(window or push_time, push_time)
    super().__init__(window, torque, speed,
                     feet_lift_frac=(push_time - FOOT_FREE) / window,
                     hands_lift_frac=1.0, **kw)
    if reference == 'bodypath':
      # forelimb push: feet planted until shortly before the legs straighten
      # in the reference (then free), hands until the end of the push
      self.q_track, self.pitch_track, lift, stroke, _ = body_path_reference(
          self.m, self.q0, push_time, self.dt, self.N + 1,
          kw.get('angle', 30.0), ref_speed=speed)
      self.n_feet = int(round((min(lift['HL'], lift['HR']) - 0.02) / self.dt))
      if window > push_time:
        self.z_ref = self.reference_com_height(push_time, stroke, kw.get('angle', 30.0))
    else:
      self.q_track, self.pitch_track = retimed_reference(push_time, self.dt,
                                                         self.N + 1)
      assert window == push_time, 'window needs the bodypath reference'
    if free_hands:
      self.n_hands = self.n_feet

  def reference_com_height(self, push_time, stroke, angle):
    """CoM height of the body-path reference at the end of its push."""
    d = mujoco.MjData(self.m)
    a = np.deg2rad(angle)
    d.qpos[:] = self.q0
    d.qpos[:3] += stroke * np.array([-np.cos(a), 0.0, np.sin(a)])
    d.qpos[7:] = self.q_track[int(round(push_time / self.dt))]
    mujoco.mj_kinematics(self.m, d)
    mujoco.mj_comPos(self.m, d)
    return float(d.subtree_com[1][2])

  def residual(self, t):
    r = super().residual(t)
    if t < self.n_push:
      d = self.d
      track = np.sqrt(W_TRACK) * (d.qpos[7:] - self.q_track[t]) / S_TRACK
      pitch = (np.sqrt(W_TRACK_PITCH) * (pitch_of(d.qpos[3:7]) - self.pitch_track[t])
               / S_TRACK_PITCH)
      r = np.concatenate([r, np.sqrt(self.dt * self.track_scale) * np.r_[track, pitch]])
      if self.min_hand_force > 0 and t < self.n_hands:
        short = np.maximum(self.min_hand_force - self.limb_forces()[:2], 0)
        r = np.concatenate([r, np.sqrt(self.dt * W_FMIN) * short / S_FMIN])
    return r

  def pd_warm_start(self, kp=600.0, kd=20.0):
    """Controls (N, 6) tracking the retimed reference with PD."""
    m, d = self.m, self.d
    d.qpos[:], d.qvel[:] = self.q0, self.v0
    mujoco.mj_forward(m, d)
    gain = m.actuator_gainprm[:12, 0]
    # desired joint torques -> actuator forces (differential transmissions)
    act_map = T.actuator_joint_map(m)
    U = np.empty((self.N, 6))
    v_track = np.gradient(self.q_track, self.dt, axis=0)
    for t in range(self.N):
      tau = kp * (self.q_track[t] - d.qpos[7:]) + kd * (v_track[t] - d.qvel[6:])
      u = np.linalg.solve(act_map.T, tau) / gain
      U[t] = T.symmetrize(np.clip(u, -1, 1))
      self.set_ctrl(U[t], t)
      mujoco.mj_step(m, d)
    return U


def sensitivity(solver, U, eps=1e-5, ndir=4, seed=1):
  """Largest cost change over ndir random smooth control perturbations of
  size eps, relative to the cost: ~eps-sized for a smooth rollout, O(0.1-1)
  when a contact switches on or off (chaotic rollout)."""
  import launch_refine as R
  c0 = solver.rollout(U)[2]
  dirs = R.perturbation(np.random.default_rng(seed), ndir, solver.N, 1.0, solver.dt)
  return float(max(abs(solver.rollout(np.clip(U + eps * d, -1, 1))[2] - c0)
                   for d in dirs) / c0)


def contact_strip(opt, half, steps=300):
  """Per-limb contact over the first steps (2 ms each), '#' = touching."""
  m, d = opt.m, mujoco.MjData(opt.m)
  mujoco.mj_setState(m, d, opt.x0, mujoco.mjtState.mjSTATE_FULLPHYSICS)
  ctrl = opt.with_latch(T.expand(half))
  names = ['to_hand_l', 'to_hand_r', 'to_foot_l', 'to_foot_r']
  rows = []
  for k in range(steps):
    d.ctrl[:] = ctrl[k]
    mujoco.mj_step(m, d)
    rows.append([d.sensordata[opt.s[n]][0] > 0 for n in names])
  rows = np.array(rows)
  return {n: ''.join('#' if x else '.' for x in rows[:, i]) for i, n in enumerate(names)}


def render(opt, traj, det, title, path_base):
  """Real-time 60 fps video of the whole recorded launch and a 16-frame
  strip to 0.1 s after takeoff (needs MUJOCO_GL for offscreen rendering)."""
  import render_launch as RL
  from PIL import Image
  t_off = det['takeoff_time'] if det.get('took_off') else 0.6
  RL.render_trajectory(traj['time'], traj['qpos'], title, path_base + '.mp4',
                       traj['comvel'], slow=1, m=opt.m, fps=60)
  m, d = opt.m, mujoco.MjData(opt.m)
  rec = RL.Recorder(m)
  rec.cam.distance = 3.5
  for t in np.linspace(0, t_off + 0.1, 16):
    k = min(int(np.searchsorted(traj['time'], t)), len(traj['time']) - 1)
    d.qpos[:] = traj['qpos'][k]
    mujoco.mj_forward(m, d)
    rec.capture(d, f"t={t:.3f} v={np.linalg.norm(traj['comvel'][k]):.1f} "
                   f"hands {int(traj['hands_contact'][k])} feet {int(traj['feet_contact'][k])}")
  W, H = RL.WIDTH // 2, RL.HEIGHT // 2
  sheet = Image.new('RGB', (4 * W, 4 * H))
  for i, f in enumerate(rec.frames):
    sheet.paste(Image.fromarray(f).resize((W, H)), ((i % 4) * W, (i // 4) * H))
  sheet.save(path_base + '_frames.png')


def parser():
  p = argparse.ArgumentParser()
  p.add_argument('--torque', type=float, default=6.0)
  p.add_argument('--spring_energy', type=float, default=500.0)
  p.add_argument('--angle', type=float, default=30.0)
  p.add_argument('--iters', type=int, default=60)
  p.add_argument('--steps', type=int, default=len(CHAIN))
  p.add_argument('--out', default=os.path.join(HERE, 'tracked'))
  p.add_argument('--chain', default='speed',
                 choices=['speed', 'relax', 'push', 'bodypath', 'bodypath_real',
                          'relax_real'])
  p.add_argument('--no_load_speed', type=float, default=0.0,
                 help='rad/s, DC-motor torque-speed line (0: ideal motors)')
  p.add_argument('--springs', default='legacy', choices=['legacy', 'designed'],
                 help='legacy: --spring_energy joint springs; designed: '
                      'SPRING_DESIGN latched springs storing --spring_budget')
  p.add_argument('--spring_budget', type=float, default=2500.0)
  p.add_argument('--spring_design', default=None,
                 help='spring_design.py JSON (default: SPRING_DESIGN)')
  p.add_argument('--spring_profile', default='flat', choices=['flat', 'linear'],
                 help='flat: constant torque then a ramp over SPRING_RAMP; '
                      'linear: a plain linear spring')
  p.add_argument('--spring_efficiency', type=float, default=1.0,
                 help='fraction of the stored energy returned')
  p.add_argument('--spring_mass', type=float, default=0.0,
                 help='spring hardware kg (both sides); if > 0 the budget is '
                      'spring_mass x spring_density and the mass is added')
  p.add_argument('--spring_density', type=float, default=500.0,
                 help='J of stored energy per kg of spring hardware')
  p.add_argument('--arm_motors_on_legs', action='store_true',
                 help='hip motors = shoulder motors, knee motors = elbow motors')
  p.add_argument('--shoulder_differential', action='store_true',
                 help='shoulder abduction and swing motors drive both joints '
                      'through a differential')
  p.add_argument('--reference', default='retimed', choices=['retimed', 'bodypath'])
  p.add_argument('--track_scale', type=float, default=1.0,
                 help='multiplies the chain tracking weights')
  p.add_argument('--min_hand_force', type=float, default=0.0,
                 help='N per hand while planted (0: off)')
  p.add_argument('--plant_tol', type=float, default=None,
                 help='m, height counted as planted (launch_ilqr default 1 mm; '
                      '0: a planted limb must touch)')
  p.add_argument('--speeds', type=float, nargs='*', default=None,
                 help='override the chain target speeds (push time and '
                      'tracking of its first step)')
  p.add_argument('--window', type=float, default=0.0,
                 help='optimized push (s, > the reference push): the hands may '
                      'keep pushing past it; takeoff scored ballistically')
  p.add_argument('--free_hands', action='store_true',
                 help='hands planted only while the feet are, then free')
  p.add_argument('--init', default=None, help='controls (.npy) to start from')
  p.add_argument('--init_push', type=float, default=0.45)
  p.add_argument('--first_step', type=int, default=0, help='numbering offset')
  return p


def make_design(args, verbose=True):
  """TrackILQR design kwargs from the command line (also sets the
  process-wide differential control mirroring and plant tolerance)."""
  if args.plant_tol is not None:
    L.PLANT_TOL = args.plant_tol
  design = dict(no_load_speed=args.no_load_speed)
  if args.arm_motors_on_legs:
    design['arm_motors_on_legs'] = True
  if args.shoulder_differential:
    L.use_shoulder_differential()
    design['shoulder_differential'] = True
  if args.springs == 'designed':
    spring_design = SPRING_DESIGN
    if args.spring_design:
      spring_design = [(r['name'], tuple(r['sides']), r['share'], r['release'],
                        r['q0'], r['q1']) for r in json.load(open(args.spring_design))]
    budget = (args.spring_mass * args.spring_density if args.spring_mass > 0
              else args.spring_budget)
    springs, times, masses = designed_springs(
        budget, spring_design, ramp=1.0 if args.spring_profile == 'linear' else SPRING_RAMP,
        efficiency=args.spring_efficiency, mass=args.spring_mass)
    design.update(latched_springs=springs, latch_times=times)
    if masses:
      design['point_masses'] = masses
    if verbose:
      print(f'springs: {budget:.0f} J stored, {args.spring_mass:g} kg')
    for sp, t in zip(springs, times) if verbose else ():
      print(f"  {sp['joint']:20s} release {t:.3f} s, k {sp['k']:6.0f} N m/rad, "
            f"rest {sp['rest']:+.3f}, peak torque {sp['tau0']:+5.0f} N m")
  else:
    design.update(spring_energy=args.spring_energy)
  return design


def main():
  args = parser().parse_args()
  design = make_design(args)
  chain = {'speed': [(T_, v, 1.0) for T_, v in CHAIN], 'relax': RELAX,
           'push': PUSH, 'bodypath': BODYPATH,
           'bodypath_real': BODYPATH_REAL, 'relax_real': RELAX_REAL}[args.chain]
  if args.speeds:
    chain = [(chain[0][0], v, chain[0][2]) for v in args.speeds]
  chain = [(T_, v, w * args.track_scale) for T_, v, w in chain]
  os.makedirs(args.out, exist_ok=True)
  json.dump(vars(args), open(os.path.join(args.out, 'args.json'), 'w'), indent=1)
  summary_path = os.path.join(args.out, 'summary.json')
  summary = json.load(open(summary_path)) if os.path.exists(summary_path) else []
  U_prev, prev_push = None, None
  if args.init:
    U_prev, prev_push = np.load(args.init), args.init_push
  for i, (push_time, speed, track_scale) in enumerate(chain[:args.steps]):
    i += args.first_step
    t0 = time.time()
    solver = TrackILQR(push_time, args.torque, speed, track_scale=track_scale,
                       min_hand_force=args.min_hand_force,
                       reference=args.reference, angle=args.angle,
                       window=args.window or None, free_hands=args.free_hands,
                       **design)
    if U_prev is None:
      U = solver.pd_warm_start()
    else:
      # previous solution, time-scaled to the new push
      s_old = np.arange(len(U_prev)) * solver.dt / prev_push
      s_new = np.arange(solver.N) * solver.dt / push_time
      U = np.stack([np.interp(s_new, s_old, U_prev[:, c]) for c in range(6)], axis=1)
    U, cost, _ = solver.solve(U, iters=args.iters, verbose=False)
    half, det = solver.report(U)
    opt = solver.opt
    opt.debounce_steps = 5
    _, det = opt.evaluate(half[None], detail=True)
    det = det[0]
    strip = contact_strip(opt, half)
    sens = sensitivity(solver, U)
    name = f'step{i}_T{push_time:g}_v{speed:g}' + (
        f'_track{track_scale:g}' if track_scale != 1 else '')
    np.save(os.path.join(args.out, name + '_controls.npy'), half)
    traj = opt.record(half, os.path.join(args.out, name + '.npz'))
    if os.environ.get('MUJOCO_GL'):
      render(opt, traj, det, f"{name}: {det['speed']:.1f} m/s @ {det['angle_deg']:.0f} deg",
             os.path.join(args.out, name))
    row = {'name': name, 'push_time': push_time, 'target_speed': speed,
           'track_scale': track_scale,
           'cost': float(cost), 'seconds': round(time.time() - t0),
           **{k: det[k] for k in ('took_off', 'speed', 'angle_deg', 'takeoff_time',
                                  'spin_per_mass', 'max_pitch_deg', 'taps',
                                  'calm_rad_s')},
           'sensitivity_1e-5': sens, 'contacts': strip}
    summary.append(row)
    json.dump(summary, open(summary_path, 'w'), indent=1)
    print(f"{name}: {det['speed']:.2f} m/s @ {det['angle_deg']:.0f} deg, takeoff "
          f"{det['takeoff_time']:.3f} s, taps {det['taps']}, spin "
          f"{det['spin_per_mass']:.2f}, pitch {det['max_pitch_deg']:.0f}, "
          f"sensitivity {sens:.3g} "
          f"[{row['seconds']} s]", flush=True)
    for n, s in strip.items():
      print(f'   {n:10s} {s[:260]}', flush=True)
    U_prev, prev_push = U, push_time


if __name__ == '__main__':
  main()
