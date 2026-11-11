#!/usr/bin/env python3
"""Checks a library of gait primitives the way MJPC will run them.

  primitive_report.py NAME=STEP.npz ... --out runs/primitives_report

For each primitive, at the 1 ms simulation timestep unless noted:
  steady   the limit cycle: 12 cycles from rest, then 2 scored
  rest     cycles 5-6 from rest (what the optimizer's search scores)
  plan     the same steady window in the MJPC planner's 20 ms model
  path     20 s open loop from rest: displacement, heading change, depth

Writes OUT.json, OUT.md (the table) and OUT_paths.png (top view of every
path, and heading against time).
"""
import argparse
import json
import os

import numpy as np

import crab_model as C
import gait_opt as G
from gait_opt import step_design, step_half, step_tau

STEADY_WARM = 12
PATH_SECONDS = 20.0


def problem_for(z, dt, warm, measure=2):
  C.set_robot(C.step_robot(z))
  metrics = json.loads(str(z['metrics']))
  half = step_half(z)
  stock = C.load_model(ground=False).actuator_ctrlrange[:, 1]
  limit = None if np.allclose(half, stock) else float(half[0])
  return G.Problem(dt=dt, k=int(z['k']), warm=warm, measure=measure,
                   speed=float(z['speed_target']), direction=z['direction'],
                   ctrl_limit=limit, design=step_design(z),
                   tau=step_tau(z),
                   yaw_rate=float(metrics.get('yaw_rate_target', 0.0)))


def path(z, seconds=PATH_SECONDS, dt=C.SIM_DT):
  """Open-loop rollout from rest: time, base position, yaw (unwrapped)."""
  p = problem_for(z, dt, warm=1)
  theta = p.fix(z['theta'])
  n = int(round(seconds / dt))
  ctrl, _ = G.controls(p.layout, theta, p.half, n, dt)
  import mujoco.rollout
  states, _ = mujoco.rollout.rollout(p.m, p.d, p.x0, ctrl[None])
  states = np.concatenate([p.x0[None], states[0]])
  L = p.state_layout
  qpos = states[:, L.qpos]
  yaw = np.unwrap(C.yaw_of(C.quat_to_mat(qpos[:, 3:7])))
  t = np.arange(n + 1) * dt
  return t, qpos[:, :3] - qpos[0, :3], yaw - yaw[0]


def check(name, file):
  z = np.load(file, allow_pickle=True)
  out = dict(name=name, file=file, direction=np.round(
      z['direction'], 3).tolist(), design=step_design(z))
  for key, dt, warm in (('steady', C.SIM_DT, STEADY_WARM),
                        ('rest', C.SIM_DT, 4),
                        ('plan', C.PLAN_DT, STEADY_WARM)):
    _, info = problem_for(z, dt, warm).evaluate(z['theta'][None])
    out[key] = info[0]
  t, pos, yaw = path(z)
  d = np.asarray(z['direction'], float)
  d /= np.linalg.norm(d)
  lateral, vertical = G.drift_axes(d)
  out['path'] = dict(seconds=PATH_SECONDS, along=float(pos[-1] @ d),
                     lateral=float(pos[-1] @ lateral),
                     vertical=float(pos[-1] @ vertical),
                     heading_change=float(yaw[-1]),
                     max_heading_error=float(np.max(np.abs(
                         yaw - out['steady']['yaw_rate_target'] * t))))
  return out, (t, pos, yaw)


def table(rows):
  lines = ['| primitive | direction | stroke (s) | speed (m/s) | yaw rate '
           '(rad/s) | lateral drift (m/s) | depth drift (m/s) | '
           'saturated | from rest: speed / yaw | 20 ms planner: speed / yaw '
           '| 20 s open loop: along, lateral, vertical (m), heading change '
           '(rad) |', '|' + '---|' * 10]
  for r in rows:
    s, rest, plan, pa = r['steady'], r['rest'], r['plan'], r['path']
    lines.append(
        f"| {r['name']} | {r['direction']} | {s['period']:.2f} | "
        f"{s['speed']:+.4f} | {s['yaw_rate']:+.3f} | "
        f"{s['lateral_rate']:+.4f} | {s['depth_rate']:+.4f} | "
        f"{s['saturated']:.3f} | {rest['speed']:+.4f} / "
        f"{rest['yaw_rate']:+.3f} | {plan['speed']:+.4f} / "
        f"{plan['yaw_rate']:+.3f} | {pa['along']:+.2f}, {pa['lateral']:+.2f},"
        f" {pa['vertical']:+.2f}, {pa['heading_change']:+.2f} |")
  return '\n'.join(lines) + '\n'


def plot(path_png, rows, paths):
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  fig, (ax, ay) = plt.subplots(1, 2, figsize=(13, 6))
  cmap = plt.get_cmap('tab20')
  for i, (r, (t, pos, yaw)) in enumerate(zip(rows, paths)):
    color = cmap(i % 20)
    ax.plot(pos[:, 0], pos[:, 1], color=color, label=r['name'])
    ax.plot(pos[-1, 0], pos[-1, 1], 'o', color=color)
    ay.plot(t, np.degrees(yaw), color=color, label=r['name'])
  ax.plot(0, 0, 'k+', ms=12)
  ax.set_aspect('equal')
  ax.set_xlabel('world x (m, robot right)')
  ax.set_ylabel('world y (m, robot anterior)')
  ax.set_title(f'{PATH_SECONDS:g} s open loop from rest, top view')
  ax.grid(alpha=0.3)
  ax.legend(fontsize=7, loc='best')
  ay.set_xlabel('time (s)')
  ay.set_ylabel('heading change (deg)')
  ay.set_title('heading')
  ay.grid(alpha=0.3)
  fig.tight_layout()
  fig.savefig(path_png, dpi=110)


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawTextHelpFormatter)
  p.add_argument('gaits', nargs='+', help='NAME=STEP.npz')
  p.add_argument('--out', required=True)
  a = p.parse_args()
  rows, paths = [], []
  for item in a.gaits:
    name, file = item.split('=')
    row, pth = check(name, file)
    rows.append(row)
    paths.append(pth)
    s = row['steady']
    print(f"{name}: v {s['speed']:+.4f} yaw {s['yaw_rate']:+.3f} T "
          f"{s['period']:.2f} plan v {row['plan']['speed']:+.4f}", flush=True)
  os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
  with open(a.out + '.json', 'w') as f:
    json.dump(rows, f, indent=2)
  with open(a.out + '.md', 'w') as f:
    f.write(table(rows))
  plot(a.out + '_paths.png', rows, paths)
  print(table(rows))


if __name__ == '__main__':
  main()
