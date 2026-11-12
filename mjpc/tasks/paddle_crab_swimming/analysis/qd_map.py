#!/usr/bin/env python3
"""A map of the robot's swimming repertoire (MAP-Elites over gaits).

Every gait (gait_opt's Fourier stroke, theta) is rolled out from rest and
placed in a cell by what it does: its mean body-frame linear velocity and
angular velocity over cycles warm+1..warm+measure, each binned coarsely
(--vel_edges, --rate_edges). A cell keeps its best gait: the most periodic
(lowest cycle error), with saturation and cost of transport as tie-breaks.
New gaits are mutations of archive members (iso+line variation: Gaussian
noise plus a random step along the line to another member). The archive
starts from the library gaits (and their mirror images) and random strokes.

The result is a list of behaviours the robot can do, not polished
primitives: promote a cell with gait_chain.py --init CELL.npz (export
writes a step npz that gait_chain, render_gait and primitive_report read).

Usage:
  qd_map.py run --out DIR [--evals 200000] [--seed_files A.npz ...]
  qd_map.py report DIR                      # projections, cell table
  qd_map.py export DIR --cells 12 40 ...    # step npz per cell
  qd_map.py export DIR --best 30            # the 30 most periodic cells
"""

import argparse
import json
import os
import time

import numpy as np

import crab_model as C
import gait_opt as G
from gait_chain import held_actuators

AXES = ('vx', 'vy', 'vz', 'wx', 'wy', 'wz')


def make_problem(a, dt=None):
  # A rotating problem (target 0) leaves the world attitude out of every
  # term; only the metrics are used here.
  return G.Problem(dt=dt or a.dt, k=a.k, warm=a.warm, measure=a.measure,
                   speed=0.0, body_rate=(0.0, 0.0, 0.0),
                   hold=held_actuators(a.hold_legs), robot=a.robot)


def descriptor(m):
  return np.array(m['body_velocity'] + m['body_rate'])


def cell_of(d, vel_edges, rate_edges):
  return tuple(int(np.searchsorted(vel_edges, x)) for x in d[:3]) + \
      tuple(int(np.searchsorted(rate_edges, x)) for x in d[3:])


def fitness(m):
  if not np.all(np.isfinite(descriptor(m))) or not np.isfinite(
      m['cycle_error']):
    return -np.inf
  return -(m['cycle_error'] + 2.0 * m['saturated'] + 0.02 * m['cot'])


def library_thetas(paths, layout, half):
  out = []
  for path in paths:
    z = np.load(path, allow_pickle=True)
    theta = G.Layout(int(z['k'])).with_k(z['theta'], layout.k)
    old = z['half'] if 'half' in z else half
    theta = G.rescale_theta(layout, theta, old, half)
    out += [theta, G.mirror_theta(layout, theta)]
  return np.array(out).reshape(-1, layout.size)


class Archive:
  def __init__(self, vel_edges, rate_edges):
    self.vel_edges, self.rate_edges = vel_edges, rate_edges
    self.cells = {}          # cell -> (fitness, theta, metrics)
    self.evals = 0

  def add(self, thetas, infos):
    new = 0
    for theta, m in zip(thetas, infos):
      self.evals += 1
      f = fitness(m)
      if not np.isfinite(f):
        continue
      cell = cell_of(descriptor(m), self.vel_edges, self.rate_edges)
      if cell not in self.cells or f > self.cells[cell][0]:
        new += cell not in self.cells
        self.cells[cell] = (f, np.array(theta), m)
    return new

  def save(self, path):
    keys = list(self.cells)
    np.savez(path, cells=np.array(keys), evals=self.evals,
             vel_edges=self.vel_edges, rate_edges=self.rate_edges,
             fitness=np.array([self.cells[c][0] for c in keys]),
             theta=np.array([self.cells[c][1] for c in keys]),
             metrics=json.dumps([self.cells[c][2] for c in keys]))

  @classmethod
  def load(cls, path):
    z = np.load(path, allow_pickle=True)
    self = cls(z['vel_edges'], z['rate_edges'])
    for c, f, t, m in zip(z['cells'], z['fitness'], z['theta'],
                          json.loads(str(z['metrics']))):
      self.cells[tuple(int(i) for i in c)] = (float(f), t, m)
    self.evals = int(z['evals'])
    return self


def run(a):
  os.makedirs(a.out, exist_ok=True)
  with open(os.path.join(a.out, 'args.json'), 'w') as f:
    json.dump(vars(a), f, indent=2)
  problem = make_problem(a)
  layout, half = problem.layout, problem.half
  rng = np.random.default_rng(a.seed)
  path = os.path.join(a.out, 'archive.npz')
  if os.path.exists(path):                       # resume
    archive = Archive.load(path)
  else:
    archive = Archive(np.array(a.vel_edges), np.array(a.rate_edges))
  log = open(os.path.join(a.out, 'log.txt'), 'a')
  t_start = time.time()
  with G.Evaluator(problem, a.processes) as evaluate:
    if archive.evals == 0:
      init = [library_thetas(a.seed_files, layout, half),
              G.seeds(layout, a.init_random, seed=a.seed)]
      init = problem.fix(G.clip_theta(layout, np.concatenate(init)))
      _, info = evaluate(init)
      archive.add(init, info)
    while archive.evals < a.evals:
      keys = list(archive.cells)
      thetas = np.array([archive.cells[k][1] for k in keys])
      i = rng.integers(len(keys), size=a.batch)
      j = rng.integers(len(keys), size=a.batch)
      x = thetas[i] + a.sigma_iso * rng.standard_normal(thetas[i].shape) \
          + a.sigma_line * rng.standard_normal((a.batch, 1)) * (
              thetas[j] - thetas[i])
      x = problem.fix(G.clip_theta(layout, x))
      t0 = time.time()
      _, info = evaluate(x)
      new = archive.add(x, info)
      line = (f'evals {archive.evals} cells {len(archive.cells)} new {new} '
              f'{a.batch / (time.time() - t0):.1f} evals/s')
      print(line, flush=True)
      log.write(line + '\n')
      if archive.evals // a.batch % a.save_every == 0:
        archive.save(path)
  archive.save(path)
  log.write(f'done in {time.time() - t_start:.0f} s\n')
  report(a.out)


def rows(archive):
  out = []
  for cell, (f, theta, m) in archive.cells.items():
    d = descriptor(m)
    out.append(dict(cell=list(cell), fitness=f, period=m['period'],
                    cycle_error=m['cycle_error'], cot=m['cot'],
                    saturated=m['saturated'],
                    **{k: float(v) for k, v in zip(AXES, d)},
                    velocity=m.get('velocity'), tilt=m['tilt_mean']))
  return sorted(out, key=lambda r: -r['fitness'])


def report(out):
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  archive = Archive.load(os.path.join(out, 'archive.npz'))
  rs = rows(archive)
  with open(os.path.join(out, 'cells.json'), 'w') as f:
    json.dump(rs, f, indent=1)
  d = np.array([[r[k] for k in AXES] for r in rs])
  fit = np.array([r['fitness'] for r in rs])
  pairs = [('vx', 'vy'), ('vy', 'vz'), ('vx', 'vz'),
           ('wx', 'wy'), ('wy', 'wz'), ('wx', 'wz'),
           ('vy', 'wx'), ('vx', 'wy'), ('vy', 'wz')]
  fig, ax = plt.subplots(3, 3, figsize=(13, 12))
  for axis, (p, q) in zip(ax.flat, pairs):
    i, j = AXES.index(p), AXES.index(q)
    sc = axis.scatter(d[:, i], d[:, j], c=np.clip(-fit, 0, 3), s=6,
                      cmap='viridis_r')
    axis.set_xlabel(p + (' (m/s)' if p[0] == 'v' else ' (rad/s)'))
    axis.set_ylabel(q + (' (m/s)' if q[0] == 'v' else ' (rad/s)'))
    axis.axhline(0, color='0.8', lw=0.5)
    axis.axvline(0, color='0.8', lw=0.5)
  fig.colorbar(sc, ax=ax, label='cycle error + penalties (lower is better)')
  fig.suptitle(f'{len(rs)} cells from {archive.evals} gaits (body frame, '
               'mean over the scored cycles)')
  fig.savefig(os.path.join(out, 'map.png'), dpi=110)
  print(f'{len(rs)} cells, {archive.evals} evals; '
        f'{os.path.join(out, "map.png")}')
  for r in rs[:20]:
    print(' '.join(f'{k} {r[k]:+.3f}' for k in AXES),
          f"cyc {r['cycle_error']:.3f} cot {r['cot']:.2f}")


def export(a):
  archive = Archive.load(os.path.join(a.dir, 'archive.npz'))
  with open(os.path.join(a.dir, 'args.json')) as f:
    run_args = argparse.Namespace(**json.load(f))
  C.set_robot(run_args.robot)
  rs = rows(archive)
  if a.best:
    picks = rs[:a.best]
  else:
    wanted = {tuple(int(i) for i in c.split(',')) for c in a.cells}
    picks = [r for r in rs if tuple(r['cell']) in wanted]
  problem = make_problem(run_args, dt=a.dt)
  os.makedirs(os.path.join(a.dir, 'cells'), exist_ok=True)
  for r in picks:
    cell = tuple(r['cell'])
    theta = archive.cells[cell][1]
    res, info = problem.evaluate(theta[None])
    cycle = problem.limit_cycle(theta)
    v = np.array(info[0]['velocity'])
    direction = v / max(np.linalg.norm(v), 1e-9)
    tag = 'cell_' + '_'.join(map(str, cell))
    np.savez(os.path.join(a.dir, 'cells', tag + '.npz'), theta=theta,
             k=problem.layout.k, dt=problem.dt, robot=problem.robot,
             speed_target=0.0, max_speed=False, direction=direction,
             x0=problem.x0, half=problem.half, design=json.dumps({}),
             tau=problem.tau, warm=problem.warm, measure=problem.measure,
             metrics=json.dumps(info[0]), joints=np.array(C.JOINTS),
             actuators=np.array(C.ACTUATORS), residual=res[0],
             **{key: cycle[key] for key in
                ('phase', 'time', 'qpos', 'qvel', 'act', 'ctrl', 'state')})
    print(tag, ' '.join(f'{k} {r[k]:+.3f}' for k in AXES))


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawTextHelpFormatter)
  sub = p.add_subparsers(dest='cmd', required=True)
  r = sub.add_parser('run')
  r.add_argument('--out', required=True)
  r.add_argument('--evals', type=int, default=200000)
  r.add_argument('--batch', type=int, default=224)
  r.add_argument('--init_random', type=int, default=512)
  r.add_argument('--seed_files', nargs='*', default=[])
  r.add_argument('--sigma_iso', type=float, default=0.05)
  r.add_argument('--sigma_line', type=float, default=0.3)
  r.add_argument('--vel_edges', type=float, nargs='+',
                 default=[-0.09, -0.05, -0.02, 0.02, 0.05, 0.09],
                 help='bin edges of body-frame velocity (m/s)')
  r.add_argument('--rate_edges', type=float, nargs='+',
                 default=[-0.6, -0.25, -0.08, 0.08, 0.25, 0.6],
                 help='bin edges of body-frame angular rate (rad/s)')
  r.add_argument('--dt', type=float, default=0.002)
  r.add_argument('--k', type=int, default=G.K_DEFAULT)
  r.add_argument('--warm', type=int, default=4)
  r.add_argument('--measure', type=int, default=2)
  r.add_argument('--hold_legs', default='')
  r.add_argument('--robot', default=C.robot(), choices=C.ROBOTS)
  r.add_argument('--processes', type=int, default=None)
  r.add_argument('--save_every', type=int, default=5)
  r.add_argument('--seed', type=int, default=0)
  q = sub.add_parser('report')
  q.add_argument('dir')
  e = sub.add_parser('export')
  e.add_argument('dir')
  e.add_argument('--cells', nargs='*', default=[],
                 help='cells as comma-separated bin indices, e.g. 3,0,3,3,3,3')
  e.add_argument('--best', type=int, default=0)
  e.add_argument('--dt', type=float, default=0.002)
  a = p.parse_args()
  if a.cmd == 'run':
    run(a)
  elif a.cmd == 'report':
    report(a.dir)
  else:
    export(a)


if __name__ == '__main__':
  main()
