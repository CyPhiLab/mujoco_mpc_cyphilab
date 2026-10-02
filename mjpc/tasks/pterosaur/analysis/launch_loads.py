#!/usr/bin/env python3
"""Structural loads of a launch: are the forces and stresses sane?

Replays a launch (launch_design.py output) and, at every step up to takeoff,
takes the interaction force between each limb segment and its parent
(MuJoCo cfrc_int, after mj_rnePostConstraint), moved to the joint, and
resolves it in the segment: axial force, shear, bending moment and torsion.
Reports the peaks with joint torques (motors and springs), ground forces and
motor power, then sizes a thin-walled tube for each segment and estimates
the spring hardware mass.

Usage:
  launch_loads.py designs/<name> [--json loads.json]
"""
import argparse
import json
import os

import mujoco
import numpy as np

import launch_design as D
import launch_ilqr as L

# segment: body, proximal joint, distal end (child body joint or contact geom)
SEGMENTS = {
    'humerus_L': ('bevel_out', 'radius_and_ulna', None),
    'humerus_R': ('bevel_out_2', 'radius_and_ulna_2', None),
    'forearm_L': ('radius_and_ulna', None, 'FL'),
    'forearm_R': ('radius_and_ulna_2', None, 'FR'),
    'femur_L': ('leg_motor_2', 'tibia', None),
    'femur_R': ('femur', 'tibia_2', None),
    'tibia_L': ('tibia', None, 'HL'),
    'tibia_R': ('tibia_2', None, 'HR'),
}

# tube materials: design stress (MPa, with safety factor), modulus (GPa),
# density (kg/m^3). Rough handbook values.
MATERIALS = {
    'carbon fiber tube (400 MPa / SF 2)': (200.0, 70.0, 1600.0),
    'aluminum 7075-T6 (500 MPa / SF 1.5)': (330.0, 71.0, 2810.0),
}
WALL_RATIO = 20      # outer diameter / wall thickness
BUCKLING_SF = 2.0

# elastic energy per kg of spring hardware (J/kg), rough practical ranges
SPRING_MATERIALS = {
    'steel coil/torsion spring': (100, 250),
    'glass/carbon composite leaf or bow': (300, 1000),
    'natural rubber / latex bands': (1000, 3000),
}


def segment_loads(m, d, ids):
  """Axial (+ = compression), shear, bending and torsion per segment."""
  out = {}
  for name, (body, child, geom) in ids.items():
    root = m.body_rootid[body]
    c = d.subtree_com[root]
    tau, force = d.cfrc_int[body, :3], d.cfrc_int[body, 3:]
    jnt = m.body_jntadr[body]
    p = d.xanchor[jnt]
    tau_p = tau + np.cross(c - p, force)
    end = d.xanchor[m.body_jntadr[child]] if child is not None else d.geom_xpos[geom]
    axis = end - p
    length = np.linalg.norm(axis)
    e = axis / length
    axial = force @ e
    # moment at the distal joint too (humerus/femur carry the child's
    # joint moment there)
    bend = np.linalg.norm(tau_p - (tau_p @ e) * e)
    if child is not None:
      cr = m.body_rootid[child]
      tq = d.cfrc_int[child, :3] + np.cross(d.subtree_com[cr] - end,
                                            d.cfrc_int[child, 3:])
      bend = max(bend, np.linalg.norm(tq - (tq @ e) * e))
    out[name] = {'axial': axial,
                 'shear': np.linalg.norm(force - axial * e),
                 'bending': bend, 'torsion': abs(tau_p @ e), 'length': length}
  return out


def size_tube(moment, compression, length, stress, modulus, density):
  """Smallest thin tube (OD/wall = WALL_RATIO) carrying the bending moment
  and axial compression within the design stress and with BUCKLING_SF
  against Euler buckling (pinned ends)."""
  for od in np.arange(0.005, 0.2, 0.0005):
    t = od / WALL_RATIO
    di = od - 2 * t
    area = np.pi / 4 * (od ** 2 - di ** 2)
    inertia = np.pi / 64 * (od ** 4 - di ** 4)
    sigma = moment * od / 2 / inertia + max(compression, 0) / area
    p_cr = np.pi ** 2 * modulus * 1e9 * inertia / length ** 2
    if sigma <= stress * 1e6 and p_cr >= BUCKLING_SF * max(compression, 0):
      return {'od_mm': round(od * 1000, 1), 'wall_mm': round(t * 1000, 2),
              'mass_kg': round(area * length * density, 3)}
  return None


def analyze(m, qpos0, qvel0, ctrl, last_release, springs):
  """Peak loads of a launch: model, start state, controls (nstep, nu),
  time of the last latch release, latched springs (for the spring sizing)."""
  d = mujoco.MjData(m)
  dt = m.opt.timestep
  mass = m.body_subtreemass[1]
  ids = {}
  for name, (body, child, geom) in SEGMENTS.items():
    bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    ids[name] = (bid(body), bid(child) if child else None,
                 mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, geom) if geom else None)
  limb_geoms = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
                for n in ('FL', 'FR', 'HL', 'HR')]
  floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'floor')
  d.qpos[:], d.qvel[:] = qpos0, qvel0
  mujoco.mj_forward(m, d)
  peaks = {n: {'axial_compression': 0.0, 'axial_tension': 0.0, 'shear': 0.0,
               'bending': 0.0, 'torsion': 0.0} for n in SEGMENTS}
  peak_act = np.zeros(m.nu)
  peak_power = np.zeros(12)
  peak_grf, peak_limb = 0.0, np.zeros(4)
  f6 = np.zeros(6)
  airborne = 0
  for t in range(len(ctrl)):
    d.ctrl[:] = ctrl[t]
    mujoco.mj_forward(m, d)
    mujoco.mj_rnePostConstraint(m, d)
    # ground forces (world frame)
    grf = np.zeros(3)
    limb_f = np.zeros(4)
    for i, c in enumerate(d.contact[:d.ncon]):
      if floor not in (c.geom1, c.geom2):
        continue
      mujoco.mj_contactForce(m, d, i, f6)
      fw = c.frame.reshape(3, 3).T @ f6[:3]
      grf += fw if c.geom1 == floor else -fw
      g = c.geom2 if c.geom1 == floor else c.geom1
      if g in limb_geoms:
        limb_f[limb_geoms.index(g)] += np.linalg.norm(f6[:3])
    if d.ncon == 0 or not any(floor in (c.geom1, c.geom2) for c in d.contact[:d.ncon]):
      airborne += 1
      # 50 ms off the ground after the last latch release: launched
      if airborne > 25 and t * dt > max(last_release, 0.15):
        break
    else:
      airborne = 0
    peak_grf = max(peak_grf, np.linalg.norm(grf))
    peak_limb = np.maximum(peak_limb, limb_f)
    for n, l in segment_loads(m, d, ids).items():
      pk = peaks[n]
      pk['axial_compression'] = max(pk['axial_compression'], l['axial'])
      pk['axial_tension'] = max(pk['axial_tension'], -l['axial'])
      for k in ('shear', 'bending', 'torsion'):
        pk[k] = max(pk[k], l[k])
      pk['length'] = l['length']
    peak_act = np.maximum(peak_act, np.abs(d.actuator_force))
    dof = m.jnt_dofadr[m.actuator_trnid[:12, 0]]
    peak_power = np.maximum(peak_power, d.actuator_force[:12] * d.qvel[dof])
    mujoco.mj_step(m, d)

  names = [m.actuator(i).name for i in range(m.nu)]
  print(f'peak ground force {peak_grf:.0f} N ({peak_grf / (mass * 9.81):.1f} '
        f'body weights); per limb FL/FR/HL/HR {np.round(peak_limb)} N')
  print('peak actuator torques (N m):')
  for n, v in zip(names, peak_act):
    print(f'  {n:24s} {v:7.0f}')
  print('peak motor power (W):', dict(zip(names[:12], np.round(peak_power))))
  report = {'peak_ground_force_N': peak_grf, 'peak_limb_force_N': peak_limb.tolist(),
            'peak_actuator_torque_Nm': dict(zip(names, peak_act.round(1).tolist())),
            'peak_motor_power_W': dict(zip(names[:12], peak_power.round(0).tolist())),
            'segments': {}, 'springs': {}}
  print('\nsegment peaks (N, N m) and tube sizing:')
  for n, pk in peaks.items():
    sizing = {mat: size_tube(pk['bending'], pk['axial_compression'], pk['length'], *v)
              for mat, v in MATERIALS.items()}
    report['segments'][n] = {**{k: round(v, 1) for k, v in pk.items()},
                             'tube': sizing}
    print(f"  {n:10s} L {pk['length']:.2f} m: compression {pk['axial_compression']:6.0f} "
          f"tension {pk['axial_tension']:6.0f} shear {pk['shear']:6.0f} "
          f"bending {pk['bending']:6.0f} torsion {pk['torsion']:5.0f}")
    for mat, sz in sizing.items():
      print(f'      {mat}: {sz}')
  print('\nsprings (per side):')
  for s in springs:
    if s['joint'].startswith('right'):
      continue
    lo_hi = {k: (round(s['energy_J'] / v[1], 2), round(s['energy_J'] / v[0], 2))
             for k, v in SPRING_MATERIALS.items()}
    report['springs'][s['joint']] = {'energy_J': s['energy_J'],
                                     'peak_torque_Nm': abs(s['tau0']),
                                     'mass_kg_range': lo_hi}
    print(f"  {s['joint']:20s} {s['energy_J']:6.0f} J, peak torque {abs(s['tau0']):5.0f} N m, "
          f"mass (kg) {lo_hi}")
  total = sum(s['energy_J'] for s in springs)
  report['springs_total_J'] = total
  print(f'  total {total:.0f} J -> ' + ', '.join(
      f'{k}: {total / v[1]:.1f}-{total / v[0]:.1f} kg' for k, v in SPRING_MATERIALS.items()))
  return report


def main():
  p = argparse.ArgumentParser()
  p.add_argument('design', help='launch_design.py output dir, or '
                 'design_search.py output dir with --key')
  p.add_argument('--key', default=None, help='design_search.py design key')
  p.add_argument('--json', default=None)
  args = p.parse_args()
  if args.key:
    import design_search as DS
    import launch_trajopt as T
    info = json.load(open(os.path.join(args.design, 'summary.json')))[args.key]
    opt = DS.DesignOpt(DS.Design(**info['design']))
    knots = np.load(os.path.join(args.design, args.key, 'knots.npy'))
    release = np.array([info['release_s'][p] for p in opt.pairs])
    ctrl = opt.controls(opt.half(knots)[None], release[None])[0]
    report = analyze(opt.m, opt.start_qpos, np.zeros(opt.m.nv), ctrl,
                     release.max(initial=0), opt.springs)
  else:
    info = json.load(open(os.path.join(args.design, 'design.json')))
    row, red, q_start, springs, solver = D.build(argparse.Namespace(**info['args']))
    U = np.load(os.path.join(args.design, 'controls.npy'))[:solver.N]
    ctrl = np.array([np.r_[L.E @ np.clip(u, -1, 1), solver.opt.latch(t * solver.dt)]
                     for t, u in enumerate(U)])
    report = analyze(solver.m, solver.q0, solver.v0, ctrl,
                     max(solver.opt.latch_times, default=0), springs)
  if args.json:
    json.dump(report, open(args.json, 'w'), indent=1)


if __name__ == '__main__':
  main()
