#!/usr/bin/env python3
"""Launch design study: actuator torque scaling vs. parallel springs.

For each design variant, optimizes open-loop push-off controls (cross-entropy
method over left/right symmetric control splines) to maximize the CoM velocity
at takeoff along a target launch angle, starting from the deepest crouch of the
reference trajectory (reference/launch.npz). Springs are modeled as latched
parallel torsion springs: preloaded in the crouch and released at the start of
the push, with rest angle at the limb's liftoff angle in the reference.

Usage:
  launch_design_study.py [--quick] [--out results.json]
"""
import argparse
import copy
import json
import os
import time

import mujoco
import numpy as np
from mujoco import rollout

HERE = os.path.dirname(os.path.abspath(__file__))
TASK_DIR = os.path.dirname(HERE)

REF_DT = 0.01            # reference sample time
SIM_DT = 0.002           # physics timestep for the study (0.01 is unstable)
MAX_SPEED = 30.0         # m/s, faster CoM motion is a simulation blow-up
T_START = 1.02           # deepest crouch in the reference, push starts here
HORIZON = 1.2            # seconds of push to optimize
LAUNCH_ANGLE = 30.0      # degrees above horizontal, toward -x like reference
KNOTS = 8                # control spline knots per actuator
JOINT_MARGIN = 0.3       # hard joint limits: reference range +- margin

# hinge indices (qpos[7:] / actuator order)
L_ARM = [0, 1, 2]
R_ARM = [3, 4, 5]
L_LEG = [6, 7, 8]
R_LEG = [9, 10, 11]
SHOULDER2 = [1, 4]
HIP2 = [7, 10]


def load_model():
  """Task model plus floor-contact and angular-momentum sensors."""
  extra = """<mujoco>
  <include file="task.xml"/>
  <sensor>
    <contact name="study_floor" subtree1="body" geom2="floor" num="1"
             data="found"/>
    <subtreeangmom name="study_angmom" body="body"/>
  </sensor>
</mujoco>"""
  path = os.path.join(TASK_DIR, '_launch_study_tmp.xml')
  with open(path, 'w') as f:
    f.write(extra)
  try:
    return mujoco.MjModel.from_xml_path(path)
  finally:
    os.remove(path)


def sensor_slice(m, name):
  i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
  return slice(m.sensor_adr[i], m.sensor_adr[i] + m.sensor_dim[i])


def mirror_signs(ref_q):
  """Sign mapping left -> right joints, from the reference trajectory."""
  signs = np.ones(6)
  for k in range(6):
    left, right = ref_q[:, k if k < 3 else k + 3], ref_q[:, k + 3 if k < 3 else k + 6]
    signs[k] = 1.0 if np.dot(left - left.mean(), right - right.mean()) >= 0 else -1.0
  return signs


def expand_controls(params, signs, nstep):
  """(N, 6, KNOTS) symmetric spline params -> (N, nstep, 12) controls."""
  n = params.shape[0]
  t_knot = np.linspace(0, 1, KNOTS)
  t = np.linspace(0, 1, nstep)
  half = np.empty((n, nstep, 6))
  for k in range(6):
    for i in range(n):
      half[i, :, k] = np.interp(t, t_knot, params[i, k])
  ctrl = np.empty((n, nstep, 12))
  ctrl[:, :, L_ARM] = half[:, :, 0:3]
  ctrl[:, :, R_ARM] = half[:, :, 0:3] * signs[0:3]
  ctrl[:, :, L_LEG] = half[:, :, 3:6]
  ctrl[:, :, R_LEG] = half[:, :, 3:6] * signs[3:6]
  return np.clip(ctrl, -1, 1)


class Study:

  def __init__(self, nthread=4):
    self.base = load_model()
    ref = np.load(os.path.join(TASK_DIR, 'reference', 'launch.npz'))
    self.ref_qpos, self.ref_qvel = ref['qpos'], ref['qvel']
    q = self.ref_qpos[:, 7:]
    self.q_lo = q.min(axis=0) - JOINT_MARGIN
    self.q_hi = q.max(axis=0) + JOINT_MARGIN
    self.signs = mirror_signs(q)
    self.i_start = int(round(T_START / REF_DT))
    self.nstep = int(round(HORIZON / SIM_DT))
    self.nthread = nthread
    self.datas = [mujoco.MjData(self.base) for _ in range(nthread)]

    # spring rest angles: limb liftoff angle in the reference
    feet_off = int(np.argmin(ref['feet_contact']))
    hands_off = int(np.argmin(ref['hands_contact']))
    self.spring_rest = {j: q[hands_off, j] for j in SHOULDER2}
    self.spring_rest.update({j: q[feet_off, j] for j in HIP2})
    self.crouch = q[self.i_start]
    self.stand = q[0]

    m = self.base
    self.s_contact = sensor_slice(m, 'study_floor')
    self.s_comvel = sensor_slice(m, 'torso_subtreelinvel')
    self.s_com = sensor_slice(m, 'torso_subtreecom')
    self.s_angmom = sensor_slice(m, 'study_angmom')
    self.s_joint = slice(sensor_slice(m, 'pos_FR_hip_joint').start,
                         sensor_slice(m, 'pos_RL_calf_joint').stop)
    # jointpos sensors are in (R arm, L arm, R leg, L leg) order
    self.sensor_to_hinge = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
    a = np.deg2rad(LAUNCH_ANGLE)
    self.direction = np.array([-np.cos(a), 0, np.sin(a)])
    self.mass = m.body_subtreemass[1]

  def spring_stiffness(self, energy):
    """Per-joint stiffness storing energy/4 in each of 4 springs at crouch."""
    return {j: 2 * (energy / 4) / (self.crouch[j] - rest) ** 2
            for j, rest in self.spring_rest.items()}

  def variant(self, torque_scale=1.0, spring_energy=0.0, no_load_speed=None):
    m = copy.copy(self.base)
    m.opt.timestep = SIM_DT
    # the model has no joint limits; add hard stops at the reference range
    for j in range(12):
      m.jnt_limited[j + 1] = 1
      m.jnt_range[j + 1] = [self.q_lo[j], self.q_hi[j]]
    gain = self.base.actuator_gainprm[:, 0] * torque_scale
    m.actuator_gainprm[:, 0] = gain
    if no_load_speed:
      # DC motor line: tau = gain * (u - qdot / no_load_speed)
      m.actuator_biastype[:] = mujoco.mjtBias.mjBIAS_AFFINE
      m.actuator_biasprm[:, :] = 0
      m.actuator_biasprm[:, 2] = -gain / no_load_speed
    info = {'springs': {}}
    if spring_energy > 0:
      for j, k in self.spring_stiffness(spring_energy).items():
        jid = j + 1  # skip freejoint
        m.jnt_stiffness[jid] = k
        m.qpos_spring[m.jnt_qposadr[jid]] = self.spring_rest[j]
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, jid)
        info['springs'][name] = {
            'k': float(k),
            'rest': float(self.spring_rest[j]),
            'torque_at_crouch': float(k * abs(self.crouch[j] - self.spring_rest[j])),
            'torque_to_hold_stand': float(k * abs(self.stand[j] - self.spring_rest[j])),
            'motor_torque': float(gain[j]),
        }
    return m, info

  def initial_state(self, m):
    d = self.datas[0]
    mujoco.mj_resetData(m, d)
    d.qpos[:] = self.ref_qpos[self.i_start]
    d.qvel[:] = self.ref_qvel[self.i_start]
    mujoco.mj_forward(m, d)
    spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
    state = np.empty(mujoco.mj_stateSize(m, spec))
    mujoco.mj_getState(m, d, state, spec)
    return state

  def evaluate(self, m, x0, params):
    ctrl = expand_controls(params, self.signs, self.nstep)
    n = params.shape[0]
    _, sens = rollout.rollout(m, self.datas, np.tile(x0, (n, 1)), ctrl)
    return self.score(sens)

  def score(self, sens, detail=False):
    n = sens.shape[0]
    found = sens[:, :, self.s_contact][:, :, 0]
    comvel = sens[:, :, self.s_comvel]
    angmom = sens[:, :, self.s_angmom]
    joints = sens[:, :, self.s_joint][:, :, np.argsort(self.sensor_to_hinge)]
    scores = np.empty(n)
    details = []
    min_flight = int(0.1 / SIM_DT)
    blowup = np.abs(comvel).max(axis=(1, 2)) > MAX_SPEED
    for i in range(n):
      # final liftoff: start of the airborne segment lasting to the end
      contact = np.flatnonzero(found[i] > 0)
      k = contact[-1] + 1 if len(contact) else 0
      took_off = k <= self.nstep - min_flight
      k = min(k, self.nstep - 1)
      v = comvel[i, k]
      along = v @ self.direction
      perp = np.linalg.norm(v - along * self.direction)
      over = (np.maximum(joints[i, :k + 1] - self.q_hi, 0) +
              np.maximum(self.q_lo - joints[i, :k + 1], 0)).sum() * SIM_DT
      pitch_rate = angmom[i, k, 1] / self.mass
      s = along - perp - 0.5 * abs(pitch_rate) - 10 * over
      if not took_off:
        s -= 3
      if blowup[i] or not np.isfinite(s):
        s = -100.0
      scores[i] = s
      if detail:
        speed = np.linalg.norm(v)
        details.append({
            'took_off': bool(took_off),
            'takeoff_time': float(T_START + (k + 1) * SIM_DT),
            'speed': float(speed),
            'angle_deg': float(np.degrees(np.arctan2(v[2], -v[0]))),
            'speed_along_target': float(along),
            'angmom_y': float(angmom[i, k, 1]),
            'joint_range_violation': float(over),
            'score': float(s),
        })
    return (scores, details) if detail else scores

  def optimize(self, m, iters=45, pop=256, elite=24, seed=0, init=None,
               verbose=False):
    rng = np.random.default_rng(seed)
    x0 = self.initial_state(m)
    mean = np.zeros((6, KNOTS)) if init is None else init.copy()
    std = np.full((6, KNOTS), 0.7 if init is None else 0.4)
    best, best_score = mean.copy(), -np.inf
    for it in range(iters):
      samples = mean + std * rng.standard_normal((pop, 6, KNOTS))
      samples[0] = best
      samples = np.clip(samples, -1.2, 1.2)
      scores = self.evaluate(m, x0, samples)
      order = np.argsort(scores)[::-1]
      if scores[order[0]] > best_score:
        best_score, best = scores[order[0]], samples[order[0]].copy()
      elites = samples[order[:elite]]
      mean = 0.3 * mean + 0.7 * elites.mean(axis=0)
      std = np.maximum(0.3 * std + 0.7 * elites.std(axis=0), 0.03)
      if verbose and it % 10 == 0:
        print(f'    iter {it:3d} best {best_score:.3f}')
    ctrl = expand_controls(best[None], self.signs, self.nstep)
    _, sens = rollout.rollout(m, self.datas[0], x0[None], ctrl)
    _, detail = self.score(sens, detail=True)
    detail = detail[0]
    detail.update(self.energy(m, x0, ctrl[0], detail['takeoff_time']))
    return best, detail

  def energy(self, m, x0, ctrl, takeoff_time):
    """Motor work and spring energy released up to takeoff."""
    m = copy.copy(m)
    m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_ENERGY
    d = mujoco.MjData(m)
    mujoco.mj_setState(m, d, x0, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    mujoco.mj_forward(m, d)
    motor_work = np.zeros(m.nu)
    peak_torque = np.zeros(m.nu)
    def spring_energy():
      e = 0.0
      for j in range(1, m.njnt):
        dq = d.qpos[m.jnt_qposadr[j]] - m.qpos_spring[m.jnt_qposadr[j]]
        e += 0.5 * m.jnt_stiffness[j] * dq * dq
      return e
    spring_0 = spring_energy()
    com_0 = d.subtree_com[1][2]
    steps = int(round((takeoff_time - T_START) / SIM_DT))
    for t in range(steps):
      d.ctrl[:] = ctrl[t]
      mujoco.mj_step(m, d)
      dof = m.actuator_trnid[:, 0]
      power = d.actuator_force * d.qvel[m.jnt_dofadr[dof]]
      motor_work += power * SIM_DT
      peak_torque = np.maximum(peak_torque, np.abs(d.actuator_force))
    ke = 0.5 * m.body_subtreemass[1] * np.sum(d.subtree_linvel[1] ** 2)
    gain = m.actuator_gainprm[:, 0]
    return {
        'motor_work_J': float(motor_work.sum()),
        'motor_positive_work_J': float(np.maximum(motor_work, 0).sum()),
        'spring_energy_released_J': float(spring_0 - spring_energy()),
        'com_kinetic_energy_J': float(ke),
        'total_kinetic_energy_J': float(d.energy[1]),
        'com_height_gain_m': float(d.subtree_com[1][2] - com_0),
        'peak_torque_fraction': [float(x) for x in peak_torque / gain],
    }


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--quick', action='store_true')
  parser.add_argument('--out', default=os.path.join(HERE, 'launch_design_results.json'))
  parser.add_argument('--seeds', type=int, default=1)
  args = parser.parse_args()

  study = Study()
  iters = 15 if args.quick else 45
  if args.quick:
    grid = [(None, [1.0, 2.0], [0.0, 500.0])]
  else:
    grid = [(None, [1.0, 1.5, 2.0, 3.0], [0.0, 250.0, 500.0, 1000.0]),
            (15.0, [1.0, 2.0], [0.0, 500.0, 1000.0])]

  results = []
  for no_load, torque_scales, spring_energies in grid:
    for ts in torque_scales:
      warm = None  # best controls of the previous spring level
      for es in spring_energies:
        m, info = study.variant(ts, es, no_load)
        t0 = time.time()
        runs = [study.optimize(m, iters=iters, seed=s) for s in range(args.seeds)]
        if warm is not None:
          runs.append(study.optimize(m, iters=iters, seed=99, init=warm))
        warm, best = max(runs, key=lambda r: r[1]['score'])
        runs = [r[1] for r in runs]
        row = {'torque_scale': ts, 'spring_energy_J': es,
               'motor': 'ideal' if not no_load else f'dc_{no_load:g}rad/s',
               **best, 'run_scores': [r['score'] for r in runs],
               'controls': warm.tolist(), **info}
        results.append(row)
        print(f"{row['motor']:>12s} torque x{ts:<3g} spring {es:6.0f} J -> "
              f"takeoff {best['speed']:5.2f} m/s @ {best['angle_deg']:5.1f} deg "
              f"(t={best['takeoff_time']:.2f}s, off={best['took_off']}) "
              f"motor {best['motor_positive_work_J']:4.0f} J "
              f"spring {best['spring_energy_released_J']:4.0f} J "
              f"[{time.time() - t0:.0f}s]", flush=True)
  with open(args.out, 'w') as f:
    json.dump(results, f, indent=1)
  print('wrote', args.out)


if __name__ == '__main__':
  main()
