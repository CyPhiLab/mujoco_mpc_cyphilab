#!/usr/bin/env python3
"""Transient reorientations: a few strokes that turn the body to a target
attitude and stop there (level -> vertical, vertical -> level, level ->
upside down).

Not a limit cycle, so not gait_chain: the controls are S segments of one
stroke each (gait_opt's Fourier stroke, --k harmonics, its own offsets and
amplitudes per segment, one shared period), then a settle window with every
command frozen at its last value. Scored over the settle window:

  attitude   mean body z against the target's (gait_opt.attitude_z)
  rate       mean and final body angular velocity (want 0: stopped)
  drift      net translation over the whole maneuver
  effort     RMS supply fraction while stroking, and saturation

theta = [log period, segment 0 (c, a, b), segment 1, ...], so gait_opt's
clip_theta and lm (forward-difference Levenberg-Marquardt, checkpointed)
work unchanged. Segments are seeded from a gait's stroke (e.g. a somersault
for a pitch maneuver, --seed_gait, the first --seed_segments segments); the
rest start with the same posture and no stroke (braking).

Writes OUT/<name>.npz: theta, ctrl (the whole maneuver at dt, settle
included), start and end state, half, dt, period, segments, target,
metrics; and OUT/<name>_history.json.

  transient_opt.py --target -1.5708 0 --seed_gait runs/v2_rotXm/...npz \\
      --out runs/v2_tr_level_vertical
  transient_opt.py --start_attitude -1.5708 0 --target 0 0 ...  # back down
"""
import argparse
import json
import multiprocessing as mp
import os
import time

import mujoco
import mujoco.rollout
import numpy as np

import crab_model as C
import gait_opt as G
from gait_chain import held_actuators, turned_state

S_ATT = 0.05      # body z error (about 3 deg)
S_RATE = 0.05     # rad/s
S_DRIFT = 0.3     # m


class Transient:
  def __init__(self, dt=0.002, segments=4, k=1, settle=1.5, target=(0, 0),
               start_attitude=None, hold=None, weights=None, robot=None):
    self.dt, self.segments, self.settle = dt, segments, settle
    self.target = np.asarray(target, float)
    self.start_attitude = start_attitude
    self.hold = sorted(hold or [])
    self.weights = dict(dict(attitude=10.0, rate=3.0, drift=1.0,
                             effort=0.3, saturation=1.0), **(weights or {}))
    self.robot = robot or C.robot()
    self.layout = G.Layout(k)            # one segment's stroke
    self.block = self.layout.size - 1    # its c, a, b (no period)
    self.size = 1 + segments * self.block
    self.m = C.load_model(dt=dt, ground=False, robot=self.robot)
    self.d = mujoco.MjData(self.m)
    self.L = C.StateLayout(self.m)
    self.half = self.m.actuator_ctrlrange[:, 1].copy()
    rest = C.initial_state(self.m)
    self.x0 = rest if start_attitude is None else turned_state(
        rest, self.L, start_attitude)
    self.z_target = G.attitude_z(self.target)
    self.v2 = C.is_v2(self.m)

  def config(self):
    return dict(dt=self.dt, segments=self.segments, k=self.layout.k,
                settle=self.settle, target=self.target.tolist(),
                start_attitude=self.start_attitude, hold=self.hold,
                weights=self.weights, robot=self.robot)

  def segment_theta(self, theta, j):
    return np.concatenate([theta[:1],
                           theta[1 + j * self.block:1 + (j + 1) * self.block]])

  def fix(self, thetas):
    """Held actuators: no stroke (a, b = 0), posture free."""
    thetas = np.array(thetas, float)
    nu, k = self.layout.nu, self.layout.k
    for j in range(self.segments):
      base = 1 + j * self.block
      for i in self.hold:
        thetas[..., base + nu + i * k:base + nu + (i + 1) * k] = 0.0
        thetas[..., base + nu + nu * k + i * k:
               base + nu + nu * k + (i + 1) * k] = 0.0
    return thetas

  def controls(self, theta):
    period = float(np.exp(theta[0]))
    n_seg = int(round(period / self.dt))
    parts, excess = [], []
    for j in range(self.segments):
      c, e = G.controls(self.layout, self.segment_theta(theta, j), self.half,
                        n_seg, self.dt)
      parts.append(c)
      excess.append(e)
    n_settle = int(round(self.settle / self.dt))
    parts.append(np.repeat(parts[-1][-1:], n_settle, axis=0))
    excess.append(np.zeros((n_settle, C.NU)))
    return np.concatenate(parts), np.concatenate(excess), n_seg * self.segments

  def evaluate(self, thetas):
    thetas = self.fix(np.atleast_2d(thetas))
    built = [self.controls(t) for t in thetas]
    n = max(len(b[0]) for b in built)
    ctrl = np.stack([np.concatenate([b[0], np.repeat(b[0][-1:], n - len(b[0]),
                                                     0)]) for b in built])
    states, sensor = mujoco.rollout.rollout(self.m, self.d, self.x0, ctrl)
    res, info = [], []
    for i, (c, ex, n_move) in enumerate(built):
      r, m = self.residual(states[i][:len(c)], sensor[i][:len(c)], ex,
                           n_move, float(np.exp(thetas[i][0])))
      res.append(r)
      info.append(m)
    return np.array(res), info

  def residual(self, states, sensor, excess, n_move, period):
    L, w = self.L, self.weights
    qpos, qvel = states[:, L.qpos], states[:, L.qvel]
    R = C.quat_to_mat(qpos[:, 3:7])
    settle = slice(n_move, None)
    z_err = R[settle, :, 2].mean(0) - self.z_target
    ang = qvel[:, 3:6]
    rate_mean, rate_end = ang[settle].mean(0), ang[-200:].mean(0)
    drift = qpos[-1, :3] - self.x0[L.qpos][:3]
    move = slice(0, n_move)
    qd = qvel[move, 6:]
    if self.v2:
      u = C.motor_fraction(sensor[move], qd)
    else:
      u = C.actuator_force(self.m, qpos[move, 7:], states[move][:, L.act]) / \
          C.FORCE_LIMIT
    effort = float(np.sqrt(np.mean(u ** 2)))
    saturated = float(np.mean(np.abs(u) > 0.99))
    r = np.concatenate([
        np.sqrt(w['attitude']) * z_err / S_ATT,
        np.sqrt(w['rate']) * rate_mean / S_RATE,
        np.sqrt(w['rate']) * rate_end / S_RATE,
        np.sqrt(w['drift']) * drift / S_DRIFT,
        [np.sqrt(w['effort']) * effort / 0.3],
        np.sqrt(w['saturation']) * np.maximum(np.abs(u) - 0.95, 0).mean(0)
        / 0.05,
        np.sqrt(w['saturation']) * np.sqrt(np.mean(excess ** 2, 0)) / 0.05])
    tilt = np.degrees(np.arccos(np.clip(R[-1, 2, 2], -1, 1)))
    metrics = dict(period=period, duration=n_move * self.dt,
                   attitude_error=float(np.linalg.norm(z_err)),
                   rate_mean=rate_mean.tolist(), rate_end=rate_end.tolist(),
                   drift=drift.tolist(), effort=effort, saturated=saturated,
                   tilt_end_deg=float(tilt), body_z_end=R[-1, :, 2].tolist(),
                   target=self.target.tolist(), cost=float(0.5 * r @ r))
    # The keys gait_opt.lm's log line reads, mapped onto this problem: its
    # "v" column shows the attitude error, "yaw" the end rate, "cyc" the
    # drift.
    metrics.update(speed=metrics['attitude_error'], lateral_rate=0.0,
                   yaw_rate=float(np.linalg.norm(rate_end)),
                   cycle_error=float(np.linalg.norm(drift)))
    return r, metrics

  def seed(self, gait=None, seed_segments=2, period=1.0):
    """theta: the first seed_segments segments from a gait's stroke (its
    first k harmonics), the rest with that posture and no stroke."""
    theta = np.zeros(self.size)
    theta[0] = np.log(period)
    block = np.zeros(self.block)
    stroke = np.zeros(self.block)
    if gait:
      z = np.load(gait, allow_pickle=True)
      t = G.Layout(int(z['k'])).with_k(z['theta'], self.layout.k)
      t = G.rescale_theta(self.layout, t, G.step_half(z), self.half)
      theta[0] = t[0]
      stroke = t[1:]
      block[:self.layout.nu] = stroke[:self.layout.nu]       # posture only
    for j in range(self.segments):
      theta[1 + j * self.block:1 + (j + 1) * self.block] = \
          stroke if j < seed_segments else block
    return G.clip_theta(None, theta)


_T = None


def _init(config):
  global _T
  _T = Transient(**config)


def _evaluate(thetas):
  return _T.evaluate(thetas)


class Pool:
  def __init__(self, problem, processes):
    self.processes = processes
    self.pool = mp.get_context('fork').Pool(processes, _init,
                                            (problem.config(),))

  def __call__(self, thetas):
    thetas = np.atleast_2d(thetas)
    n = max(1, int(np.ceil(len(thetas) / (self.processes * 2))))
    out = self.pool.map(_evaluate, [thetas[i:i + n]
                                    for i in range(0, len(thetas), n)])
    return np.concatenate([o[0] for o in out]), [i for o in out for i in o[1]]

  def close(self):
    self.pool.close()
    self.pool.join()


class Size:            # what gait_opt.lm needs of a layout
  def __init__(self, size):
    self.size = size


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawTextHelpFormatter)
  p.add_argument('--target', type=float, nargs=2, required=True,
                 help='end attitude: about world x, then y (rad)')
  p.add_argument('--start_attitude', type=float, nargs=2, default=None)
  p.add_argument('--segments', type=int, default=4)
  p.add_argument('--k', type=int, default=1)
  p.add_argument('--settle', type=float, default=1.5)
  p.add_argument('--seed_gait', default=None)
  p.add_argument('--seed_segments', type=int, default=2)
  p.add_argument('--period', type=float, default=1.0)
  p.add_argument('--hold_legs', default='')
  p.add_argument('--weights', default='')
  p.add_argument('--dts', type=float, nargs='+', default=[0.002, 0.001])
  p.add_argument('--iters', type=int, nargs='+', default=[30, 6],
                 help='LM iterations at each dt')
  p.add_argument('--robot', default=C.robot(), choices=C.ROBOTS)
  p.add_argument('--processes', type=int, default=None)
  p.add_argument('--out', required=True)
  a = p.parse_args()
  from gait_chain import parse_weights
  os.makedirs(a.out, exist_ok=True)
  with open(os.path.join(a.out, 'args.json'), 'w') as f:
    json.dump(vars(a), f, indent=2)
  logf = open(os.path.join(a.out, 'log.txt'), 'a')

  def log(text):
    print(text, flush=True)
    logf.write(text + '\n')
    logf.flush()

  theta = None
  for dt, iters in zip(a.dts, a.iters):
    problem = Transient(dt=dt, segments=a.segments, k=a.k, settle=a.settle,
                        target=a.target, start_attitude=a.start_attitude,
                        hold=held_actuators(a.hold_legs),
                        weights=parse_weights(a.weights), robot=a.robot)
    if theta is None:
      theta = problem.seed(a.seed_gait, a.seed_segments, a.period)
    name = f'transient_dt{dt:g}'
    pool = Pool(problem, a.processes or os.cpu_count())
    t0 = time.time()
    theta, cost, history = G.lm(
        pool, Size(problem.size), problem.fix(theta), iters=iters, log=log,
        checkpoint=os.path.join(a.out, f'.ckpt_{name}_lm.pkl'))
    pool.close()
    res, info = problem.evaluate(theta[None])
    m = info[0]
    ctrl, _, n_move = problem.controls(problem.fix(theta))
    states, _ = mujoco.rollout.rollout(problem.m, problem.d, problem.x0,
                                       ctrl[None])
    np.savez(os.path.join(a.out, name + '.npz'), theta=theta, ctrl=ctrl,
             dt=dt, k=a.k, segments=a.segments, period=m['period'],
             settle=a.settle, half=problem.half, robot=problem.robot,
             start=problem.x0, end=states[0, -1], target=problem.target,
             metrics=json.dumps(m), residual=res[0],
             actuators=np.array(C.ACTUATORS))
    with open(os.path.join(a.out, name + '_history.json'), 'w') as f:
      json.dump(history, f)
    log(f'[{name}] {m["duration"]:.2f} s of strokes + {a.settle} s settle: '
        f'attitude error {m["attitude_error"]:.3f}, end tilt '
        f'{m["tilt_end_deg"]:.0f} deg, rate end '
        f'{np.round(m["rate_end"], 3).tolist()}, drift '
        f'{np.round(m["drift"], 2).tolist()} m, effort {m["effort"]:.2f}, '
        f'{time.time() - t0:.0f} s')


if __name__ == '__main__':
  main()
