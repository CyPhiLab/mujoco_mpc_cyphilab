#!/usr/bin/env python3
"""Where a gait's thrust, drag and effort go: a design audit.

  design_audit.py STEP.npz [STEP.npz ...]

For each gait, on its steady limit cycle (STEADY_WARM cycles from rest, 1 ms):

  fluid forces   the swim-direction component of MuJoCo's fluid force on the
                 robot, split by body part (body: plate, servo housings,
                 cylinder; walking legs; paddle legs), averaged over a cycle.
                 Positive is thrust. MuJoCo's ellipsoid model sums per geom,
                 so a part's share is the force with only its geoms
                 interacting. (A body whose geoms all have zero interaction
                 falls back to the inertia-box model, so the others are
                 scaled to 1e-12 rather than to zero.)
  tow test       fluid drag on the body alone (legs not interacting) when the
                 whole robot is moved at constant speed in the swim
                 direction, at the gait's mean posture
  actuators      per joint: RMS torque / limit, time at the torque limit,
                 positive work per cycle, and how much the setpoint filter
                 shrinks the stroke (act amplitude / ctrl amplitude)
"""
import json
import sys

import mujoco
import numpy as np

import crab_model as C
import gait_opt as G

STEADY_WARM = 12
PARTS = {
    'body': lambda name: 'segment' not in name,
    'walking legs': lambda name: 'segment' in name and 'back' not in name,
    'paddle legs': lambda name: 'segment' in name and 'back' in name,
}


def load(path):
  z = np.load(path)
  half = z['half'] if 'half' in z else None
  limit = float(half[0]) if half is not None and np.allclose(half, half[0]) \
      else None
  return z, limit


def steady_cycle(z, limit, nphase=200):
  problem = G.Problem(dt=C.SIM_DT, k=int(z['k']), warm=STEADY_WARM, measure=1,
                      direction=z['direction'], ctrl_limit=limit)
  return problem, problem.limit_cycle(z['theta'], nphase=nphase)


def fluid_split(m, cycle, direction):
  d = mujoco.MjData(m)
  body_name = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
               or '' for g in range(m.ngeom)]
  fluid = m.geom_fluid.copy()
  out = {}
  for part, member in PARTS.items():
    keep = np.array([member(n) for n in body_name])
    m.geom_fluid[:] = fluid
    m.geom_fluid[~keep, 0] = 1e-12
    force = []
    for q, v, a in zip(cycle['qpos'], cycle['qvel'], cycle['act']):
      d.qpos[:], d.qvel[:], d.act[:] = q, v, a
      mujoco.mj_forward(m, d)
      force.append(d.qfrc_fluid[:3] @ direction)   # free joint: world frame
    out[part] = float(np.mean(force))
  m.geom_fluid[:] = fluid
  return out


def tow_test(m, cycle, direction, speeds):
  """Drag on the body alone at constant speed, mean posture, legs still."""
  d = mujoco.MjData(m)
  body_name = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
               or '' for g in range(m.ngeom)]
  fluid = m.geom_fluid.copy()
  out = {}
  for label, members in (('body only', PARTS['body']),
                         ('whole robot, legs still', lambda n: True)):
    keep = np.array([members(n) for n in body_name])
    m.geom_fluid[:] = fluid
    m.geom_fluid[~keep, 0] = 1e-12
    drag = []
    for s in speeds:
      d.qpos[:] = cycle['qpos'].mean(axis=0)
      d.qpos[3:7] = cycle['qpos'][0, 3:7]
      d.qvel[:] = 0
      d.qvel[:3] = s * direction
      mujoco.mj_forward(m, d)
      drag.append(-float(d.qfrc_fluid[:3] @ direction))
    out[label] = drag
  m.geom_fluid[:] = fluid
  return out


def actuators(problem, cycle):
  q = cycle['qpos'][:, 7:]
  qd = cycle['qvel'][:, 6:]
  act, ctrl = cycle['act'], cycle['ctrl']
  force = C.actuator_force(problem.m, q, act)
  period = problem.layout.period(problem_theta)
  dt = period / len(q)
  rows = []
  for i, name in enumerate(C.JOINTS):
    amp_ctrl = np.ptp(ctrl[:, i]) / 2
    amp_act = np.ptp(act[:, i]) / 2
    rows.append(dict(
        joint=name,
        rms_torque=float(np.sqrt(np.mean(force[:, i] ** 2))
                         / problem.m.actuator_forcerange[i, 1]),
        saturated=float(np.mean(
            np.abs(C.servo_command_force(problem.m, q, act)[:, i])
            >= problem.m.actuator_forcerange[i, 1])),
        work=float(np.sum(np.maximum(force[:, i] * qd[:, i], 0)) * dt),
        filter_gain=float(amp_act / amp_ctrl) if amp_ctrl > 1e-3 else None,
        stroke=float(np.ptp(q[:, i]))))
  return rows


if __name__ == '__main__':
  report = {}
  for path in sys.argv[1:]:
    z, limit = load(path)
    problem, cycle = steady_cycle(z, limit)
    problem_theta = z['theta']
    direction = np.asarray(z['direction'], float)
    _, info = problem.evaluate(z['theta'][None])
    speed = info[0]['speed']
    split = fluid_split(problem.m, cycle, direction)
    speeds = [0.05, 0.1, 0.15, 0.2]
    tow = tow_test(problem.m, cycle, direction, speeds)
    acts = actuators(problem, cycle)
    report[path] = dict(speed=speed, period=info[0]['period'],
                        fluid_force=split, tow_speeds=speeds, tow_drag=tow,
                        actuators=acts)
    print(f'\n== {path}: steady speed {speed:.4f} m/s, '
          f'T {info[0]["period"]:.3f} s')
    print(' mean fluid force along the swim direction (N, + = thrust):')
    for part, f in split.items():
      print(f'   {part:14s} {f:+.3f}')
    print(' tow drag (N) at', speeds, 'm/s:')
    for label, drag in tow.items():
      print(f'   {label:24s}', np.round(drag, 3))
    print(' joint       rms_torque  saturated  work/cycle(J)  filter_gain  '
          'stroke(rad)')
    for r in acts:
      fg = '   -  ' if r['filter_gain'] is None else f'{r["filter_gain"]:.2f}'
      print(f'   {r["joint"]:6s}     {r["rms_torque"]:.2f}       '
            f'{r["saturated"]:.2f}       {r["work"]:6.3f}        {fg}       '
            f'{r["stroke"]:.2f}')
  with open('design_audit.json', 'w') as f:
    json.dump(report, f, indent=1)
