#!/usr/bin/env python3
"""Sparse launch design search in the full MuJoCo model.

For each of a few designs (motor torque scale and no-load speed, limb
length, latched spring energy, forearm mass), the push is optimized directly
in MuJoCo with launch_trajopt.py's sampling optimizer and score, extended
with the latch release time of each spring pair (shoulder2, elbow, hip2,
knee) as decision variables and a penalty on peak ground force above a cap.

Springs: latched, near-constant torque (launch_trajopt.add_latched_springs),
one per sagittal joint and side, travelling the joint's excursion in the
reference push (crouch to that limb's liftoff) and stopping at its liftoff
angle. A spring released late (after the joint has moved toward its rest)
does less work; one released after takeoff does none.

Usage:
  design_search.py [--designs A,B,...] [--iters 150] [--out designs_search]
"""
import argparse
import json
import os
import time
from dataclasses import asdict, dataclass

import mujoco
import numpy as np

import launch_design as D
import launch_trajopt as T

HERE = os.path.dirname(os.path.abspath(__file__))

# sagittal spring joints: (name, left/right hinge indices, reference limb)
SPRING_JOINTS = [('shoulder2', (1, 4), 'hands'), ('elbow', (2, 5), 'hands'),
                 ('hip2', (7, 10), 'feet'), ('knee', (8, 11), 'feet')]
SPRING_SPLIT = {'shoulder2': 0.4, 'elbow': 0.1, 'hip2': 0.4, 'knee': 0.1}
RAMP = 0.3            # last fraction of the travel over which torque ramps to 0
MAX_RELEASE = 0.6     # s; later = never (the horizon covers takeoff)
W_FORCE = 10.0        # per fraction of peak ground force above the cap


@dataclass
class Design:
  name: str
  torque: float = 2.0
  no_load_speed: float = 20.0
  limb_scale: float = 1.0
  spring_energy: float = 0.0      # J, total over all springs (both sides)
  forearm_mass: float = 0.0       # kg per forearm; 0 = model (1.61 kg)
  force_cap: float = 5000.0       # N, peak total ground force
  body_scale: float = 1.0         # whole-robot geometric scale
  push_time: float = 0.0          # s; > 0 enforces the reference contact
                                  # order (feet off at 2/3, hands at the end)
  target_speed: float = 0.0       # m/s, for reference (Habib's model)
  legacy_spring_energy: float = 0.0  # J, launch_trajopt.add_springs springs
  trunk_limb_collision: bool = False
  feet_lift_frac: float = T.FEET_LIFT_FRAC  # with push_time: feet leave at
                                            # this fraction of it
  init: str = ''                  # controls (.npy, (nstep, 6)) to start from


DESIGNS = {
    'A': Design('A'),
    'B': Design('B', spring_energy=2000),
    'C': Design('C', limb_scale=1.25, spring_energy=2000),
    'D': Design('D', limb_scale=1.25, spring_energy=4000),
    'E': Design('E', limb_scale=1.25, spring_energy=2000, forearm_mass=0.6),
    'F': Design('F', torque=4.0, limb_scale=1.25, spring_energy=2000),
    # benchmark: candidates/C_t6_s500 (8.16 m/s @ 35 deg) is a known solution
    'K': Design('K', torque=6.0, no_load_speed=0.0, legacy_spring_energy=500,
                trunk_limb_collision=True, force_cap=1e9),
    # candidate C refined toward a fluid push: all four limbs planted, feet
    # leaving ~25 ms before the hands, no re-contact, <= 10 body weights
    'K2': Design('K2', torque=6.0, no_load_speed=0.0, legacy_spring_energy=500,
                 trunk_limb_collision=True, force_cap=10 * 50.13 * 9.81,
                 push_time=0.47, feet_lift_frac=0.95,
                 init='candidates/C_t6_s500/controls.npy'),
}

# Realistic scaled designs: the whole robot scaled by s (mass 50 s^3 kg);
# actuators 30% of body mass at 40 N m/kg peak (quasi-direct-drive class),
# 30 rad/s no-load; springs storing 0/25/50 J per kg of robot (5%/10% of
# body mass at ~500 J/kg); ground force capped at 10 body weights; a
# quadrupedal push of 0.4 sqrt(s) s. Target: Habib's push speed at 30 deg
# with the span scaled from 4.5 m.
MODEL_GAIN_SUM = 730.0        # N m, sum of the 12 model actuator gains
ACTUATOR_FRACTION = 0.30
TORQUE_DENSITY = 40.0         # N m / kg
HABIB_TARGET = {0.5: 8.67, 0.65: 9.88, 0.8: 10.96, 1.0: 12.26}


def scaled_design(s, j_per_kg):
  mass = 50.13 * s ** 3
  torque = ACTUATOR_FRACTION * mass * TORQUE_DENSITY / MODEL_GAIN_SUM
  return Design(f'S{s:g}_E{j_per_kg:g}', torque=torque, no_load_speed=30.0,
                spring_energy=j_per_kg * mass, force_cap=10 * mass * 9.81,
                body_scale=s, push_time=0.4 * s ** 0.5,
                target_speed=HABIB_TARGET.get(s, 0.0))


for _s in HABIB_TARGET:
  for _e in (0, 25, 50):
    _d = scaled_design(_s, _e)
    DESIGNS[_d.name] = _d


def springs_for(design, q_start):
  """Latched springs (both sides) storing the design's energy."""
  ref = T.reference()
  i0 = T.push_start_index(ref)
  q = ref['qpos'][:, 7:]
  off = {'hands': i0 + int(np.argmin(ref['hands_contact'][i0:])),
         'feet': i0 + int(np.argmin(ref['feet_contact'][i0:]))}
  out = []
  for name, sides, limb in SPRING_JOINTS:
    energy = design.spring_energy * SPRING_SPLIT[name] / 2
    if energy <= 0:
      continue
    left = sides[0]
    delta = q[off[limb], left] - q[i0, left]
    a = abs(delta)
    tau0 = energy / (a * (1 - RAMP / 2))
    k = tau0 / (RAMP * a)
    for s in sides:
      out.append({'joint': f'{T_JOINTS[s]}', 'k': float(k),
                  'rest': float(q_start[7 + s] + delta),
                  'tau0': float(np.sign(delta) * tau0), 'energy_J': float(energy),
                  'pair': name})
  return out


T_JOINTS = ['leftarm_shoulder1', 'leftarm_shoulder2', 'leftarm_elbow',
            'rightarm_shoulder1', 'rightarm_shoulder2', 'rightarm_elbow',
            'leftleg_hip1', 'leftleg_hip2', 'leftleg_knee',
            'rightleg_hip1', 'rightleg_hip2', 'rightleg_knee']


class DesignOpt(T.LaunchOpt):
  """LaunchOpt with per-sample latch release times (one per spring pair)
  and a peak ground force penalty."""

  def __init__(self, design, angle=30.0):
    ref = T.reference()
    q_ref = ref['qpos'][T.push_start_index(ref)]
    common = dict(limb_scale=design.limb_scale, no_load_speed=design.no_load_speed,
                  trunk_limb_collision=design.trunk_limb_collision,
                  forearm_mass=design.forearm_mass, body_scale=design.body_scale,
                  spring_energy=design.legacy_spring_energy)
    base = T.load_model(design.torque, **common)
    q_start, _ = D.crouch_qpos(base, q_ref)
    self.springs = springs_for(design, q_start)
    self.pairs = sorted({s['pair'] for s in self.springs},
                        key=[n for n, _, _ in SPRING_JOINTS].index)
    self.spring_pair = np.array([self.pairs.index(s['pair']) for s in self.springs],
                                int)
    super().__init__(design.torque, angle, push_time=design.push_time,
                     latched_springs=self.springs,
                     start_qpos=q_start, latch_times=np.zeros(len(self.springs)),
                     **common)
    self.design = design
    self.grf = T.sensor(self.m, 'to_grf')
    self.min_takeoff_step = int(round(0.15 * design.body_scale ** 0.5 / T.SIM_DT))
    self.feet_lift_frac = design.feet_lift_frac
    self.debounce_steps = 5      # 10 ms

  def controls(self, halves, release):
    """halves (n, nstep, 6), release times (n, npair) -> ctrl (n, nstep, nu)."""
    ctrl = T.expand(halves)
    if not len(self.springs):
      return ctrl
    t = np.arange(ctrl.shape[1]) * T.SIM_DT
    times = release[:, self.spring_pair]                    # (n, nspring)
    latch = (t[None, :, None] >= times[:, None, :]).astype(float)
    return np.concatenate([ctrl, latch], axis=-1)

  def half(self, knots):
    """Knots (.., nknot, 6) -> controls (.., nstep, 6). With an init, the
    knots are a smooth correction added to its controls (which a knot fit
    would blur)."""
    u = T.from_knots(knots)
    if self.design.init:
      if not hasattr(self, '_base'):
        self._base = np.load(os.path.join(HERE, self.design.init))[:u.shape[-2]]
      u = np.clip(self._base + u, -1, 1)
    return u

  def evaluate_full(self, halves, release, detail=False):
    ctrl = self.controls(halves, release)
    n = ctrl.shape[0]
    state, sens = T.rollout.rollout(self.m, self.datas, np.tile(self.x0, (n, 1)),
                                    ctrl)
    out = self.score(ctrl, state, sens, detail)
    scores, details = out if detail else (out, None)
    peak = np.linalg.norm(sens[:, :, self.grf], axis=-1).max(axis=1)
    scores = scores - W_FORCE * np.maximum(peak / self.design.force_cap - 1, 0)
    if detail:
      for dt, pk in zip(details, peak):
        dt['peak_grf_N'] = float(pk)
      return scores, details
    return scores

  def search(self, iters=150, pop=192, elite=16, seed=0, verbose=True):
    rng = np.random.default_rng(seed)
    npair = len(self.pairs)
    # warm starts: the reference push compressed to a few durations, with
    # hind springs released at 0.1 s and fore springs at 0.2 s
    default_release = np.array([0.2 if p in ('shoulder2', 'elbow') else 0.1
                                for p in self.pairs]) * self.design.body_scale ** 0.5
    best, best_score = None, -np.inf
    sc = self.design.body_scale
    pushes = ([f * self.design.push_time for f in (0.8, 1.0, 1.25)]
              if self.design.push_time else [0.35, 0.45, 0.55])
    # PD gains: the same command per radian of error as at 2x torque
    kp, kd = 300.0 * self.design.torque, 10.0 * self.design.torque * sc ** 0.5
    starts = ([np.zeros((T.knot_count(), 6))] if self.design.init else
              [T.to_knots(self.pd_warm_start(push, kp, kd)) for push in pushes])
    for push, knots in zip(pushes, starts):
      s = self.evaluate_full(self.half(knots)[None], default_release[None])[0]
      if verbose:
        print(f'  warm start push {push}: {s:.3f}', flush=True)
      if s > best_score:
        best_score, best = s, (knots, default_release.copy())
    base_k, base_r = best[0].copy(), best[1].copy()
    sigma, sigma_r = (0.1 if self.design.init else 0.25), 0.08
    for it in range(iters):
      K = np.clip(base_k + sigma * rng.standard_normal((pop,) + base_k.shape), -1, 1)
      Rl = np.clip(base_r + sigma_r * rng.standard_normal((pop, npair)), 0, MAX_RELEASE)
      K[0], Rl[0] = base_k, base_r
      scores = self.evaluate_full(self.half(K), Rl)
      order = np.argsort(scores)[::-1]
      if scores[order[0]] > best_score:
        best_score = scores[order[0]]
        best = (K[order[0]].copy(), Rl[order[0]].copy())
      base_k = np.clip(base_k + (K[order[:elite]] - base_k).mean(axis=0), -1, 1)
      base_r = np.clip(base_r + (Rl[order[:elite]] - base_r).mean(axis=0), 0, MAX_RELEASE)
      spread = scores[order[:elite]].std()
      sigma = max(0.03, sigma * (0.97 if spread < 0.05 else 0.99))
      sigma_r = max(0.01, sigma_r * 0.985)
      if verbose and (it % 10 == 0 or it == iters - 1):
        print(f'  iter {it:3d} best {best_score:.3f} (pop best {scores[order[0]]:.3f}, '
              f'sigma {sigma:.3f}, release {np.round(base_r, 3)})', flush=True)
    return best, best_score

  def record_full(self, knots, release, path):
    """Record the trajectory (as LaunchOpt.record) with these latch times."""
    self.latch_times = release[self.spring_pair] if len(self.springs) else self.latch_times
    return self.record(self.half(knots), path)


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--designs', default='A,B,C,D,E,F',
                 help="comma-separated keys, or 'scaled' for the S* designs")
  p.add_argument('--iters', type=int, default=150)
  p.add_argument('--angle', type=float, default=30.0)
  p.add_argument('--out', default=os.path.join(HERE, 'designs_search'))
  args = p.parse_args()
  os.makedirs(args.out, exist_ok=True)
  summary_path = os.path.join(args.out, 'summary.json')
  summary = json.load(open(summary_path)) if os.path.exists(summary_path) else {}
  keys = ([k for k in DESIGNS if k.startswith('S')] if args.designs == 'scaled'
          else args.designs.split(','))
  for key in keys:
    design = DESIGNS[key]
    print(f'design {key}: {design}', flush=True)
    t0 = time.time()
    opt = DesignOpt(design, args.angle)
    (knots, release), score = opt.search(args.iters)
    _, det = opt.evaluate_full(opt.half(knots)[None], release[None], detail=True)
    det = det[0]
    d = os.path.join(args.out, key)
    os.makedirs(d, exist_ok=True)
    np.save(os.path.join(d, 'knots.npy'), knots)
    opt.record_full(knots, release, os.path.join(d, 'trajectory.npz'))
    summary[key] = {'design': asdict(design), 'score': float(score),
                    'release_s': dict(zip(opt.pairs, release.round(3).tolist())),
                    'springs': opt.springs, 'start_qpos': opt.start_qpos.tolist(),
                    'result': det, 'seconds': round(time.time() - t0)}
    print(f"design {key}: {det.get('speed', 0):.2f} m/s @ {det.get('angle_deg', 0):.0f} deg, "
          f"takeoff {det.get('takeoff_time', 0):.2f} s, peak GRF {det['peak_grf_N']:.0f} N, "
          f"spin {det.get('spin_per_mass', 0):.2f}, pitch {det.get('max_pitch_deg', 0):.0f}, "
          f"taps {det.get('taps')}, release {summary[key]['release_s']} "
          f"[{summary[key]['seconds']} s]", flush=True)
    json.dump(summary, open(summary_path, 'w'), indent=1)


if __name__ == '__main__':
  main()
