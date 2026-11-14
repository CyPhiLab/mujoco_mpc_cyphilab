#!/usr/bin/env python3
"""Re-polish a saved gait under an extra objective, keeping what it does.

The gait's own problem (mirror_primitives.load_problem with mirror=False:
same start state, warm-up, window, direction and yaw / body-rate /
attitude targets) is re-solved from its theta with --weights added. Its
speed target becomes the speed it achieves, so the twist stays. Used to make
the library free of self-collision: --weights clearance=10 keeps same-side
neighbouring legs 5 mm apart (gait_opt.leg_clearance).

LM at each --dts in turn (--iters each). Every stage writes a step npz like
gait_chain's (theta, state at phase 0, the stored cycle, metrics) to OUT,
then OUT/summary.json with the speed, rates and minimum leg clearance before
and after.

  repolish.py SRC.npz OUT --weights yaw=4,clearance=10 [--hold_legs ...]
"""
import argparse
import json
import os

import numpy as np

import crab_model as C
import gait_opt as G
from gait_chain import held_actuators, parse_weights
from mirror_primitives import load_problem, save


def summary(m):
  return {key: m.get(key) for key in (
      'speed', 'lateral_rate', 'depth_rate', 'yaw_rate', 'body_rate',
      'tilt_mean', 'cot', 'saturated', 'cycle_error', 'min_leg_clearance',
      'period', 'cost')}


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawTextHelpFormatter)
  p.add_argument('src')
  p.add_argument('out')
  p.add_argument('--weights', default='clearance=10',
                 help="the source run's --weights plus the new terms")
  p.add_argument('--hold_legs', default='')
  p.add_argument('--dts', type=float, nargs='+', default=[0.002, 0.001])
  p.add_argument('--iters', type=int, nargs='+', default=[15, 4])
  p.add_argument('--es_iters', type=int, default=0,
                 help='elitist ES iterations before LM at the first dt '
                 '(gait_opt.es), to get out of a stalled start')
  p.add_argument('--keep_speed_target', action='store_true',
                 help='keep the saved speed target instead of the achieved '
                 'speed')
  p.add_argument('--processes', type=int, default=2)
  a = p.parse_args()
  z = np.load(a.src, allow_pickle=True)
  C.set_robot(C.step_robot(z))
  weights = parse_weights(a.weights)
  hold = held_actuators(a.hold_legs)
  os.makedirs(a.out, exist_ok=True)
  base = os.path.splitext(os.path.basename(a.src))[0]
  theta = np.array(z['theta'], float)
  rows = []
  # Before: the source on its own problem, scored with the new terms.
  problem = load_problem(z, weights=weights, hold=hold, mirror=False)
  _, info = problem.evaluate(theta[None])
  before = info[0]
  rows.append(dict(stage='source', file=a.src, **summary(before)))
  print(f'source v {before["speed"]:+.4f} yaw {before["yaw_rate"]:+.3f} '
        f'clearance {1000 * before["min_leg_clearance"]:+.1f} mm', flush=True)
  for dt, iters in zip(a.dts, a.iters):
    problem = load_problem(z, dt=dt, weights=weights, hold=hold, mirror=False)
    if not a.keep_speed_target and not problem.max_speed:
      problem.speed = float(before['speed'])
    tag = f'{base}_clear_dt{dt:g}'
    with G.Evaluator(problem, a.processes) as ev:
      if a.es_iters and dt == a.dts[0]:
        theta, _, _ = G.es(ev, problem.layout, theta, iters=a.es_iters,
                           checkpoint=os.path.join(a.out, f'.ckpt_{tag}_es.pkl'))
      theta, _, history = G.lm(ev, problem.layout, theta, iters=iters,
                               checkpoint=os.path.join(a.out,
                                                       f'.ckpt_{tag}.pkl'))
    m = save(os.path.join(a.out, tag + '.npz'), problem, theta, history)
    rows.append(dict(stage=f'dt{dt:g}', file=os.path.join(a.out, tag + '.npz'),
                     **summary(m)))
    print(f'{tag}: v {m["speed"]:+.4f} yaw {m["yaw_rate"]:+.3f} body rate '
          f'{np.round(m["body_rate"], 3).tolist()} clearance '
          f'{1000 * m["min_leg_clearance"]:+.1f} mm', flush=True)
  with open(os.path.join(a.out, 'summary.json'), 'w') as f:
    json.dump(rows, f, indent=2)


if __name__ == '__main__':
  main()
