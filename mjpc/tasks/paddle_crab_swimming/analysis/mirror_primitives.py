#!/usr/bin/env python3
"""Left/right mirror images of gait primitives.

The robot is (to within its CAD) mirror symmetric about its sagittal plane,
so the mirror image of a gait (gait_opt.mirror_theta: left and right legs
swapped, with the joint-axis signs of paddle_crab_swimming.cc MirrorSign)
swims the mirrored direction (x -> -x) and turns the other way. That gives
-X, the -X diagonals and turning right from their +X / left-turning
counterparts without another optimization. Each mirror is re-scored on the
same problem as its source (mirrored), optionally re-polished with a few
Levenberg-Marquardt iterations (--polish N) to take up the small asymmetry,
and saved as a step npz like gait_chain.py's, so export_gaits.py and
render_gait.py read it.

  mirror_primitives.py SRC.npz OUT_DIR [--polish 4] [--processes 2]
"""
import argparse
import json
import os

import numpy as np

import crab_model as C
import gait_opt as G
from gait_opt import step_design, step_half, step_tau


def load_problem(z, dt=None):
  """The Problem a step npz was scored on (or at another dt), mirrored."""
  metrics = json.loads(str(z['metrics']))
  half = step_half(z)
  stock = C.load_model(ground=False).actuator_ctrlrange[:, 1]
  limit = None if np.allclose(half, stock) else float(half[0])
  direction = np.array(z['direction'], float)
  direction[0] *= -1
  return G.Problem(
      dt=float(z['dt']) if dt is None else dt, k=int(z['k']),
      warm=int(z['warm']), measure=int(z['measure']),
      speed=float(z['speed_target']), direction=direction,
      x0=mirror_state(z['x0']), ctrl_limit=limit,
      design=step_design(z), tau=step_tau(z),
      yaw_rate=-float(metrics.get('yaw_rate_target', 0.0)))


def mirror_state(state):
  """A full-physics state reflected through the body's sagittal plane
  (world x -> -x at the rest heading): base position/velocity x negated,
  the quaternion reflected, joints swapped left/right with MirrorSign."""
  m = C.load_model(ground=False)
  L = C.StateLayout(m)
  s = np.array(state, float)
  qpos, qvel, act = s[L.qpos].copy(), s[L.qvel].copy(), s[L.act].copy()
  # Reflection x -> -x: position x, and for a rotation R -> P R P with
  # P = diag(-1, 1, 1) the quaternion (w, x, y, z) -> (w, x, -y, -z).
  qpos[0] *= -1
  qpos[3:7] = qpos[3:7] * [1, 1, -1, -1]
  qvel[0] *= -1                      # world linear velocity
  qvel[3:6] = qvel[3:6] * [1, -1, -1]  # body angular velocity (pseudovector)
  idx, sign = C.MIRROR_INDEX, C.MIRROR_SIGN
  qpos[7:] = sign * qpos[7:][idx]
  qvel[6:] = sign * qvel[6:][idx]
  act = sign * act[idx]
  s[L.qpos], s[L.qvel], s[L.act] = qpos, qvel, act
  return s


def save(path, problem, theta, history):
  res, info = problem.evaluate(theta[None])
  cycle = problem.limit_cycle(theta)
  np.savez(path, theta=theta, k=problem.layout.k, dt=problem.dt,
           speed_target=problem.speed, max_speed=problem.max_speed,
           direction=problem.direction, x0=problem.x0, half=problem.half,
           design=json.dumps(problem.design), tau=problem.tau,
           warm=problem.warm, measure=problem.measure,
           metrics=json.dumps(info[0]), joints=np.array(C.JOINTS),
           actuators=np.array(C.ACTUATORS), residual=res[0],
           **{key: cycle[key] for key in
              ('phase', 'time', 'qpos', 'qvel', 'act', 'ctrl', 'state')})
  with open(path.replace('.npz', '_history.json'), 'w') as f:
    json.dump(history, f)
  return info[0]


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawTextHelpFormatter)
  p.add_argument('src')
  p.add_argument('out')
  p.add_argument('--polish', type=int, default=0,
                 help='LM iterations on the mirrored problem')
  p.add_argument('--processes', type=int, default=2)
  a = p.parse_args()
  z = np.load(a.src, allow_pickle=True)
  problem = load_problem(z)
  layout = problem.layout
  theta = G.mirror_theta(layout, z['theta'])
  source = json.loads(str(z['metrics']))
  os.makedirs(a.out, exist_ok=True)
  _, info = problem.evaluate(theta[None])
  print(f'source  v {source["speed"]:+.4f} lat {source["lateral_rate"]:+.4f} '
        f'yaw {source["yaw_rate"]:+.4f} cost {source["cost"]:.3f}')
  print(f'mirror  v {info[0]["speed"]:+.4f} lat {info[0]["lateral_rate"]:+.4f}'
        f' yaw {info[0]["yaw_rate"]:+.4f} cost {info[0]["cost"]:.3f}')
  history = [dict(info[0], iter=0)]
  base = os.path.splitext(os.path.basename(a.src))[0]
  metrics = save(os.path.join(a.out, f'{base}_mirror.npz'), problem, theta,
                 history)
  rows = [dict(file=f'{base}_mirror.npz', source=a.src, polish=0, **metrics)]
  if a.polish:
    with G.Evaluator(problem, a.processes) as ev:
      theta, _, h = G.lm(ev, layout, theta, iters=a.polish)
    metrics = save(os.path.join(a.out, f'{base}_mirror_polish.npz'), problem,
                   theta, h)
    rows.append(dict(file=f'{base}_mirror_polish.npz', source=a.src,
                     polish=a.polish, **metrics))
  with open(os.path.join(a.out, 'summary.json'), 'w') as f:
    json.dump(rows, f, indent=2)


if __name__ == '__main__':
  main()
