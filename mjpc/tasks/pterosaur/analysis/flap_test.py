#!/usr/bin/env python3
"""Flapping test of the winged robot (wing_model.py) in a wind tunnel.

The trunk is held at a fixed attitude in a head wind (its free joint reset
every step, so it acts as an infinitely heavy body; gravity off). Joint PD
control holds the flight pose and beats the wings by sinusoids on the
shoulder abduction joints (the flapping axis in the flight pose), through the
motors with their torque-speed limits. Reports the achieved stroke, cycle
averaged lift and thrust (whole robot), motor saturation and power.

Usage:
  flap_test.py [--render DIR]
"""
import argparse
import os

import mujoco
import numpy as np

import wing_model as W

ARM = ['shoulder1', 'shoulder2', 'elbow', 'finger']
KP, KD = 3000.0, 60.0          # joint PD (N m/rad, N m s/rad), torque-limited
FLAP_RANGE = 2.9               # rad, test range of the abduction joints
# strokes: name, stroke angle (deg, peak to peak), period (s); Habib's normal
# flap (60 deg in 0.379 s) and climb-out (150 deg downstroke in 0.285 s)
STROKES = [('normal', 60.0, 2 * 0.379), ('intermediate', 100.0, 2 * 0.33),
           ('climb-out', 150.0, 2 * 0.285)]


def actuator_joint_map(m, joints):
  """(nu, len(joints)) joint torque per unit actuator force, by name."""
  jid = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n): k for k, n in enumerate(joints)}
  out = np.zeros((m.nu, len(joints)))
  for i in range(m.nu):
    t = m.actuator_trnid[i, 0]
    if m.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT:
      if t in jid:
        out[i, jid[t]] = m.actuator_gear[i, 0]
    else:
      for w in range(m.tendon_adr[t], m.tendon_adr[t] + m.tendon_num[t]):
        if m.wrap_objid[w] in jid:
          out[i, jid[m.wrap_objid[w]]] = m.wrap_prm[w] * m.actuator_gear[i, 0]
  return out


def flap_sign(m, d):
  """+1 if raising the left abduction angle lifts the left wingtip."""
  tip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'left_wingtip')
  j = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'leftarm_shoulder1')]
  mujoco.mj_kinematics(m, d)
  z0 = d.site_xpos[tip, 2]
  d.qpos[j] += 0.1
  mujoco.mj_kinematics(m, d)
  up = d.site_xpos[tip, 2] > z0
  d.qpos[j] -= 0.1
  return 1.0 if up else -1.0


def run(m, stroke_deg, period, speed, alpha_deg=10.0, cycles=4, record=None):
  d = mujoco.MjData(m)
  joints = W.LIMB_JOINTS + W.FINGER_JOINTS
  qadr = np.array([m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in joints])
  vadr = np.array([m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in joints])
  amap = actuator_joint_map(m, joints)
  gain = m.actuator_gainprm[:, 0]
  th = np.radians(alpha_deg)
  base_q = np.array([0, 0, 1.0, np.cos(th / 2), 0, np.sin(th / 2), 0])   # nose (-x) up
  pose = W.flight_pose()
  q_hold = np.array([pose.get(n, 0.0) for n in joints])
  # legs tucked: the X2 start crouch angles
  z = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'export_x2', 'trajectory.npz'))
  for k, n in enumerate(W.LIMB_JOINTS):
    if 'leg' in n:
      q_hold[k] = z['start_qpos'][7 + k]
  d.qpos[:7] = base_q
  d.qpos[qadr] = q_hold
  sign = flap_sign(m, d)
  ia = [joints.index('leftarm_shoulder1'), joints.index('rightarm_shoulder1')]
  amp = np.radians(stroke_deg) / 2
  w = 2 * np.pi / period
  m.opt.wind[:] = [speed, 0, 0]
  gravity = m.opt.gravity.copy()
  m.opt.gravity[:] = 0
  n = int(round(cycles * period / m.opt.timestep))
  F, Q, U, P = [], [], [], []
  for t in range(n):
    s = t * m.opt.timestep
    q_des, v_des = q_hold.copy(), np.zeros(len(joints))
    # downstroke first: start at the top of the stroke
    q_des[ia[0]] += sign * amp * np.cos(w * s)
    q_des[ia[1]] -= sign * amp * np.cos(w * s)       # mirrored axis on the right
    v_des[ia[0]] = -sign * amp * w * np.sin(w * s)
    v_des[ia[1]] = sign * amp * w * np.sin(w * s)
    tau = KP * (q_des - d.qpos[qadr]) + KD * (v_des - d.qvel[vadr])
    u = np.linalg.lstsq(amap.T, tau, rcond=None)[0] / gain
    d.ctrl[:] = np.clip(u, -1, 1)
    mujoco.mj_step(m, d)
    d.qpos[:7] = base_q
    d.qvel[:6] = 0
    F.append(d.qfrc_fluid[:3].copy())
    Q.append(d.qpos[qadr[ia]].copy())
    U.append(np.abs(d.ctrl) > 0.99)
    P.append(d.actuator_force * d.actuator_velocity)
    if record is not None:
      record.append(d.qpos.copy())
  m.opt.wind[:] = 0
  m.opt.gravity[:] = gravity
  last = slice(n - int(round(2 * period / m.opt.timestep)), n)   # last 2 cycles
  F, Q, U, P = map(np.array, (F, Q, U, P))
  names = [m.actuator(i).name for i in range(m.nu)]
  sh = [names.index('leftarm_shoulder1'), names.index('leftarm_shoulder2')]
  ts = np.arange(n) * m.opt.timestep
  down = np.sin(w * ts) > 0                        # q_des = amp cos(w t): top at t = 0
  down_last = down[last]
  return {'lift_N': F[last, 2].mean(), 'thrust_N': -F[last, 0].mean(),
          'down_lift_N': F[last, 2][down_last].mean(), 'up_lift_N': F[last, 2][~down_last].mean(),
          'down_thrust_N': -F[last, 0][down_last].mean(), 'up_thrust_N': -F[last, 0][~down_last].mean(),
          'peak_lift_N': F[last, 2].max(),
          'stroke_deg': np.degrees(Q[last, 0].max() - Q[last, 0].min()),
          'shoulder_saturated': U[last][:, sh].mean(),
          'shoulder_power_W': P[last][:, sh].sum(axis=1).max(),
          'shoulder_mean_power_W': P[last][:, sh].sum(axis=1).mean()}


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--render', default=None, help='directory for climb-out videos')
  args = p.parse_args()
  m = W.build()
  for n in ('leftarm_shoulder1', 'rightarm_shoulder1'):
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    m.jnt_range[j] = [-FLAP_RANGE, FLAP_RANGE]
  weight = m.body_subtreemass[1] * 9.81
  print(f'weight {weight:.0f} N; trunk at 10 deg nose-up; averages over the last 2 cycles')
  print('stroke         target   speed | achieved  mean lift (x weight)  thrust  peak lift | '
        'shoulder sat.  peak / mean power (one side)')
  for name, stroke, period in STROKES:
    for speed in (0.0, 8.0, 15.0):
      r = run(m, stroke, period, speed)
      print(f'{name:12s} {stroke:4.0f}deg/{period:.2f}s {speed:4.0f} | {r["stroke_deg"]:5.0f} deg '
            f'{r["lift_N"]:7.0f} N ({r["lift_N"] / weight:4.2f}) {r["thrust_N"]:7.0f} N '
            f'{r["peak_lift_N"]:7.0f} N | {r["shoulder_saturated"]:5.0%} '
            f'{r["shoulder_power_W"]:7.0f} / {r["shoulder_mean_power_W"]:5.0f} W')
      print(f'{"":33s}   downstroke: lift {r["down_lift_N"]:5.0f} N, thrust {r["down_thrust_N"]:5.0f} N; '
            f'upstroke: lift {r["up_lift_N"]:5.0f} N, thrust {r["up_thrust_N"]:5.0f} N')
  if args.render:
    import render_launch as RL
    os.makedirs(args.render, exist_ok=True)
    for speed in (0.0, 15.0):
      rec = []
      run(m, 150.0, 2 * 0.285, speed, cycles=3, record=rec)
      t = np.arange(len(rec)) * m.opt.timestep
      RL.render_trajectory(t, np.array(rec), f'climb-out stroke, {speed:g} m/s',
                           os.path.join(args.render, f'flap_climbout_v{speed:g}.mp4'),
                           m=m, fps=60)


if __name__ == '__main__':
  main()
