#!/usr/bin/env python3
"""Full-model (MuJoCo) check of a reduced-model launch design.

Takes a design from reduced_springs.py (motor torque scale and no-load
speed, limb scale, latched springs, force cap) and builds it in the MuJoCo
pterosaur: limb segments lengthened, motors with a DC torque-speed line,
latched springs as joint actuators released on the reduced model's schedule.
The start crouch is re-solved for the longer limbs (all four limbs on the
ground, joint angles kept), the reduced model's push is mapped onto the
model's joints as the warm start, and launch_ilqr.py's iLQR refines it.

Usage:
  launch_design.py --torque 2 --no_load_speed 20 --limb_scale 1.25 \
      --spring_energy 4000 --force_cap 5000 [--speed 10] [--iters 60] \
      [--out design_dir]
"""
import argparse
import json
import os
import time

import mujoco
import numpy as np
from scipy.optimize import least_squares

import launch_ilqr as L
import launch_study as S
import launch_trajopt as T
import reduced_launch as R

HERE = os.path.dirname(os.path.abspath(__file__))
SCALING = os.path.join(HERE, 'study_results', 'reduced_springs_scaling.json')
CANDIDATES = os.path.join(HERE, 'study_results', 'reduced_candidates.json')
SCALING_NO_PITCH = os.path.join(HERE, 'study_results', 'no_pitch',
                                'reduced_springs_scaling.json')

# sagittal hinges (qpos[7:] indices), left and right
SAGITTAL = {'shoulder2': (1, 4), 'elbow': (2, 5), 'hip2': (7, 10), 'knee': (8, 11)}
MIN_SPRING_J = 20.0   # smaller reduced-model springs are dropped


def reduced_design(torque, no_load_speed, limb_scale, spring_energy, force_cap,
                   mass=50, path=SCALING):
  rows = []
  for f in [path, CANDIDATES]:
    if os.path.exists(f):
      rows += json.load(open(f))
  for r in rows:
    if (r['torque_scale'] == torque and r['no_load_speed'] == no_load_speed and
        r['limb_scale'] == limb_scale and r['budget_J'] == spring_energy and
        r['force_cap'] == force_cap and r['mass'] == mass):
      return r
  raise KeyError('design not in ' + path)


def reduced_trajectory(row):
  """Re-simulate a reduced-model result: joint angles (n, 4) [hip2, knee,
  shoulder2, elbow] in planar coordinates, commands (n, 4), limbs, dt."""
  design = R.Design(**{k: row[k] for k in (
      'torque_scale', 'no_load_speed', 'mass', 'limb_scale', 'joint_margin',
      'friction', 'force_cap')}, pitch_dynamics=row.get('pitch_dynamics', False))
  model = R.ReducedLaunch(design)
  params = np.array(row['params'])[None]
  springs = None
  if row.get('spring_unit'):
    model.spring_budget, model.spring_mask = row['budget_J'], np.ones(4)
    springs = np.array(row['spring_unit'])[None]
  res = model.simulate(params, record=True, springs=springs)
  geom, _ = model.geometry(model.reference_crouch[None])
  angles = []
  for p, _, _, _ in res['traj']:
    angles.append(np.concatenate([R.joint_angles(l, p, g)[0]
                                  for l, g in zip(model.limbs, geom)]))
  q0 = np.concatenate([g['q0'][0] for g in geom])
  return {'angles': np.array(angles), 'q0': q0, 'u': model.controls(params)[0],
          'limbs': model.limbs, 'dt': model.dt, 'takeoff': float(res['t'][0]),
          'liftoff': res['liftoff_t'][0], 'speed': row['speed_along']}


def crouch_qpos(m, q_ref, joint_weight=3.0, pitch_weight=1.0):
  """Start pose for a model with scaled limbs: the reference crouch with the
  base raised/pitched and the sagittal joints adjusted (least change) so the
  hand and foot spheres sit where the reference's do (on the floor)."""
  d = mujoco.MjData(m)
  gid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
         for n in ('FL', 'FR', 'HL', 'HR')]
  target = T.FLOOR_Z

  def pose(x):
    q = q_ref.copy()
    q[2] += x[0]
    half = 0.5 * x[1]
    rot = np.array([np.cos(half), 0, np.sin(half), 0])
    mujoco.mju_mulQuat(q[3:7], rot, q_ref[3:7])
    for j, (name, (l, r)) in enumerate(SAGITTAL.items()):
      q[7 + l] += x[2 + j]
      q[7 + r] += x[2 + j]
    return q

  def residual(x):
    d.qpos[:] = pose(x)
    mujoco.mj_kinematics(m, d)
    h = [d.geom_xpos[g, 2] - m.geom_size[g, 0] - target for g in gid]
    return np.concatenate([10 * np.array(h), [pitch_weight * x[1]],
                           joint_weight * x[2:]])

  sol = least_squares(residual, np.zeros(6))
  return pose(sol.x), sol


def model_springs(row, red, m, q_start):
  """Reduced-model springs -> latched spring actuators (both sides) and
  their release times."""
  sp = row.get('springs')
  if not sp:
    return [], []
  names = list(SAGITTAL)            # shoulder2, elbow, hip2, knee
  # reduced joint order: hind prox, hind mid, fore prox, fore mid
  joints = ['hip2', 'knee', 'shoulder2', 'elbow']
  springs, times = [], []
  for j, name in enumerate(joints):
    energy = sp['energy'][j] / 2          # per side
    if energy < MIN_SPRING_J / 2:
      continue
    limb = red['limbs'][j // 2]
    sign = limb.sign[j % 2]
    release = sp['release'][j // 2]
    if release >= red['liftoff'][j // 2]:
      continue   # released after the limb left the ground: no work done
                 # in the reduced model, a flailing limb in the full model
    k_rel = min(int(round(release / red['dt'])), len(red['angles']) - 1)
    a = abs(sp['travel'][j])
    direction = sign * np.sign(sp['travel'][j])
    f = float(np.clip(sp['exponent'][j], 0.1, 1.0))
    tau0 = energy / (a * (1 - f / 2))
    k = tau0 / (f * a)
    for side in SAGITTAL[name]:
      q_rel = q_start[7 + side] + sign * (red['angles'][k_rel, j] - red['q0'][j])
      springs.append({'joint': m.joint(side + 1).name, 'k': float(k),
                      'rest': float(q_rel + direction * a),
                      'tau0': float(direction * tau0), 'energy_J': energy,
                      'release_s': float(release)})
      times.append(release)
  return springs, times


def warm_start(red, n, dt):
  """Reduced commands -> (n, 6) symmetric model channels
  [shoulder1, shoulder2, elbow, hip1, hip2, knee]."""
  t = np.arange(n) * dt
  tr = np.arange(len(red['u'])) * red['dt']
  u = np.stack([np.interp(t, tr, red['u'][:, c]) for c in range(4)], axis=1)
  hind, fore = red['limbs']
  out = np.zeros((n, 6))
  out[:, 1] = fore.sign[0] * u[:, 2]
  out[:, 2] = fore.sign[1] * u[:, 3]
  out[:, 4] = hind.sign[0] * u[:, 0]
  out[:, 5] = hind.sign[1] * u[:, 1]
  return np.clip(out, -1, 1)


def build(args):
  row = reduced_design(args.torque, args.no_load_speed, args.limb_scale,
                       args.spring_energy, args.force_cap)
  red = reduced_trajectory(row)
  ref = T.reference()
  q_ref = ref['qpos'][T.push_start_index(ref)]
  base = T.load_model(args.torque, limb_scale=args.limb_scale,
                      no_load_speed=args.no_load_speed,
                      trunk_limb_collision=False)
  q_start, _ = crouch_qpos(base, q_ref)
  springs, times = model_springs(row, red, base, q_start)
  push_time = args.push_time or round(red['takeoff'], 3)
  feet_frac, hands_frac = (min(float(t) / red['takeoff'], 1.0) for t in red['liftoff'])
  solver = L.LaunchILQR(push_time, args.torque, args.speed, args.angle,
                        force_cap=args.limb_force_cap, feet_lift_frac=feet_frac,
                        hands_lift_frac=hands_frac,
                        limb_scale=args.limb_scale,
                        no_load_speed=args.no_load_speed,
                        latched_springs=springs, start_qpos=q_start,
                        trunk_limb_collision=False,
                        latch_times=times)
  return row, red, q_start, springs, solver


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--torque', type=float, default=2)
  p.add_argument('--no_load_speed', type=float, default=20)
  p.add_argument('--limb_scale', type=float, default=1.25)
  p.add_argument('--spring_energy', type=float, default=4000)
  p.add_argument('--force_cap', type=float, default=5000)
  p.add_argument('--limb_force_cap', type=float, default=0.0,
                 help='iLQR per-limb normal force cap (N); 0 = off')
  p.add_argument('--speed', type=float, default=10.0)
  p.add_argument('--angle', type=float, default=30.0)
  p.add_argument('--push_time', type=float, default=0.0,
                 help='default: the reduced model takeoff time')
  p.add_argument('--iters', type=int, default=60)
  p.add_argument('--init', default=None, help='controls (.npy) to start from')
  p.add_argument('--out', default=None)
  args = p.parse_args()
  name = (f't{args.torque:g}_w{args.no_load_speed:g}_L{args.limb_scale:g}_'
          f's{args.spring_energy:g}_cap{args.force_cap:g}')
  out = args.out or os.path.join(HERE, 'designs', name)
  os.makedirs(out, exist_ok=True)

  row, red, q_start, springs, solver = build(args)
  print(f'reduced model: {red["speed"]:.2f} m/s along {args.angle:g} deg, '
        f'takeoff {red["takeoff"]:.3f} s, liftoff hind/fore {red["liftoff"]}')
  print(f'start: base z {q_start[2]:.3f}, springs:')
  for s in springs:
    print('  ', {k: round(v, 3) if isinstance(v, float) else v for k, v in s.items()})
  U = (np.load(args.init)[:solver.N] if args.init else
       warm_start(red, solver.N, solver.dt))
  half, det = solver.report(U)
  print('warm start:', {k: det[k] for k in ('speed', 'angle_deg', 'takeoff_time')},
        flush=True)
  t0 = time.time()
  U, cost, history = solver.solve(U, iters=args.iters)
  half, det = solver.report(U)
  metrics = S.launch_metrics(solver, U)
  print(f'iLQR {time.time() - t0:.0f} s, cost {cost:.3f}')
  print('result:', det)
  print('metrics:', metrics, flush=True)
  np.save(os.path.join(out, 'controls.npy'), half)
  solver.opt.record(half, os.path.join(out, 'trajectory.npz'))
  json.dump({'args': vars(args), 'springs': springs,
             'start_qpos': q_start.tolist(), 'reduced': {
                 k: row[k] for k in ('speed_along', 'takeoff_time', 'springs',
                                     'peak_force_N', 'peak_torque_Nm')},
             'result': det, 'metrics': metrics, 'cost': cost},
            open(os.path.join(out, 'design.json'), 'w'), indent=1)
  print('wrote', out)


if __name__ == '__main__':
  main()
