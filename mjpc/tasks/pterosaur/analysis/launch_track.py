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
FOOT_LEAD = 0.025          # s, feet leave this long before the hands
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


class TrackILQR(L.LaunchILQR):
  """LaunchILQR plus tracking of the retimed reference during the push."""

  def __init__(self, push_time, torque, speed, **kw):
    super().__init__(push_time, torque, speed,
                     feet_lift_frac=(push_time - FOOT_LEAD) / push_time,
                     hands_lift_frac=1.0, **kw)
    self.q_track, self.pitch_track = retimed_reference(push_time, self.dt,
                                                       self.N + 1)

  def residual(self, t):
    r = super().residual(t)
    if t < self.n_push:
      d = self.d
      track = np.sqrt(W_TRACK) * (d.qpos[7:] - self.q_track[t]) / S_TRACK
      pitch = (np.sqrt(W_TRACK_PITCH) * (pitch_of(d.qpos[3:7]) - self.pitch_track[t])
               / S_TRACK_PITCH)
      r = np.concatenate([r, np.sqrt(self.dt) * np.r_[track, pitch]])
    return r

  def pd_warm_start(self, kp=600.0, kd=20.0):
    """Controls (N, 6) tracking the retimed reference with PD."""
    m, d = self.m, self.d
    d.qpos[:], d.qvel[:] = self.q0, self.v0
    mujoco.mj_forward(m, d)
    gain = m.actuator_gainprm[:12, 0]
    U = np.empty((self.N, 6))
    v_track = np.gradient(self.q_track, self.dt, axis=0)
    for t in range(self.N):
      u = (kp * (self.q_track[t] - d.qpos[7:]) + kd * (v_track[t] - d.qvel[6:])) / gain
      U[t] = T.symmetrize(np.clip(u, -1, 1))
      self.set_ctrl(U[t], t)
      mujoco.mj_step(m, d)
    return U


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
  """Slow-motion video (push + 0.3 s) and a 16-frame strip to 0.1 s after
  takeoff (needs MUJOCO_GL for offscreen rendering)."""
  import render_launch as RL
  from PIL import Image
  t_off = det['takeoff_time'] if det.get('took_off') else 0.6
  end = min(len(traj['time']), int((t_off + 0.3) / T.SIM_DT))
  RL.render_trajectory(traj['time'][:end], traj['qpos'][:end], title,
                       path_base + '.mp4', traj['comvel'][:end], slow=4, m=opt.m)
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


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--torque', type=float, default=6.0)
  p.add_argument('--spring_energy', type=float, default=500.0)
  p.add_argument('--angle', type=float, default=30.0)
  p.add_argument('--iters', type=int, default=60)
  p.add_argument('--steps', type=int, default=len(CHAIN))
  p.add_argument('--out', default=os.path.join(HERE, 'tracked'))
  args = p.parse_args()
  os.makedirs(args.out, exist_ok=True)
  summary_path = os.path.join(args.out, 'summary.json')
  summary = []
  U_prev = None
  for i, (push_time, speed) in enumerate(CHAIN[:args.steps]):
    t0 = time.time()
    solver = TrackILQR(push_time, args.torque, speed, angle=args.angle,
                       spring_energy=args.spring_energy)
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
    name = f'step{i}_T{push_time:g}_v{speed:g}'
    np.save(os.path.join(args.out, name + '_controls.npy'), half)
    traj = opt.record(half, os.path.join(args.out, name + '.npz'))
    if os.environ.get('MUJOCO_GL'):
      render(opt, traj, det, f"{name}: {det['speed']:.1f} m/s @ {det['angle_deg']:.0f} deg",
             os.path.join(args.out, name))
    row = {'name': name, 'push_time': push_time, 'target_speed': speed,
           'cost': float(cost), 'seconds': round(time.time() - t0),
           **{k: det[k] for k in ('took_off', 'speed', 'angle_deg', 'takeoff_time',
                                  'spin_per_mass', 'max_pitch_deg', 'taps',
                                  'calm_rad_s')},
           'contacts': strip}
    summary.append(row)
    json.dump(summary, open(summary_path, 'w'), indent=1)
    print(f"{name}: {det['speed']:.2f} m/s @ {det['angle_deg']:.0f} deg, takeoff "
          f"{det['takeoff_time']:.3f} s, taps {det['taps']}, spin "
          f"{det['spin_per_mass']:.2f}, pitch {det['max_pitch_deg']:.0f} "
          f"[{row['seconds']} s]", flush=True)
    for n, s in strip.items():
      print(f'   {n:10s} {s[:260]}', flush=True)
    U_prev, prev_push = U, push_time


if __name__ == '__main__':
  main()
