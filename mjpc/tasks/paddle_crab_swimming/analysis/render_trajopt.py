#!/usr/bin/env python3
"""Side-by-side video and energy comparison of a Fourier gait (gait_chain
step npz) and a periodic_trajopt gait, both on the real robot v2 model
(dcmotor servos, free joint) from their steady strokes.

  MUJOCO_GL=osmesa render_trajopt.py --fourier runs/v2_mY/step6_v0.1458_dt0.001.npz \
      --trajopt runs/tj_mY_smooth2/best_stage10_v0.1570.npz --out runs/tj_mY_compare

Writes OUT.mp4 (left: Fourier gait, right: trajopt gait, camera following
each body) and OUT.json (speed, drift, and energy per stroke and per metre).

Energy, per stroke over the last strokes simulated:
  mech_work   positive mechanical work of the motors, (u x stall torque) x
              joint speed with u the supply fraction (the pipeline's measure,
              gait_opt 'work')
  electrical  electrical energy drawn, V I with V = 12 u and
              I = (V - K qd) / R, positive part only (no regeneration)
  cot         electrical energy / (m g distance)
"""
import argparse
import json
import os

import mujoco
import numpy as np

import crab_model as C
import gait_opt as G
import periodic_trajopt as P

DT = 0.001


def fourier_run(path, m, cycles):
  z = np.load(path, allow_pickle=True)
  layout = G.Layout(int(z['k']))
  theta = z['theta']
  period = layout.period(theta)
  n = int(round(cycles * period / DT))
  u, _ = G.controls(layout, theta, m.actuator_ctrlrange[:, 1], n, DT)
  L = C.StateLayout(m)
  s = z['state']
  return s[L.qpos], s[L.qvel], u, period


def trajopt_run(path, m, cycles):
  r = np.load(path)
  me = P.load_euler_model(float(r['dt']))
  qpos, qvel = P.euler_to_free(me, r['X'][0])
  period = float(r['period'])
  K, dt_opt, Pk = int(r['substeps']), float(r['dt']), r['P']
  n = int(round(cycles * period / DT))
  t = np.arange(n) * DT
  f = (t / (K * dt_opt)) % len(Pk)
  i = np.floor(f).astype(int) % len(Pk)
  w = (f - np.floor(f))[:, None]
  u = (1 - w) * Pk[i] + w * Pk[(i + 1) % len(Pk)]
  return qpos, qvel, u, period


def simulate(m, qpos, qvel, u, period, fps, slow):
  """Steps the model; returns frame qpos, and per-step energy terms."""
  d = mujoco.MjData(m)
  d.qpos[:], d.qvel[:] = qpos, qvel
  mujoco.mj_forward(m, d)
  gain = m.actuator_gainprm
  R, K, vmax = gain[:, 0], gain[:, 1], gain[:, 7]
  frames, pos, mech, elec, sat = [], [], [], [], []
  next_frame = 0.0
  for k, uk in enumerate(u):
    if d.time >= next_frame - 1e-9:
      frames.append(d.qpos.copy())
      next_frame += slow / fps
    pos.append(d.qpos[:3].copy())
    d.ctrl[:] = uk
    mujoco.mj_step(m, d)
    qd = d.qvel[6:]
    frac = C.motor_fraction(d.actuator_force, qd)
    torque = frac * C.V2_STALL
    mech.append(np.sum(np.maximum(torque * qd, 0)) * DT)
    V = np.clip(frac, -1, 1) * vmax
    current = (V - K * qd) / R
    elec.append(np.sum(np.maximum(V * current, 0)) * DT)
    sat.append(np.mean(np.abs(frac) > 0.99))
  pos.append(d.qpos[:3].copy())
  return (np.array(frames), np.array(pos), np.array(mech), np.array(elec),
          np.array(sat))


def stats(pos, mech, elec, sat, period, mass, direction, skip):
  steps = int(round(period / DT))
  ncyc = len(mech) // steps
  a, b = skip * steps, ncyc * steps
  disp = pos[b] - pos[a]
  dur = (b - a) * DT
  lat, vert = G.drift_axes(direction)
  dist = float(direction @ disp)
  E = float(elec[a:b].sum())
  W = float(mech[a:b].sum())
  n = ncyc - skip
  return dict(period=period, speed=dist / dur,
              lateral=float(lat @ disp) / dur, depth=float(vert @ disp) / dur,
              mech_work_per_stroke=W / n, electrical_per_stroke=E / n,
              mech_work_per_m=W / dist, electrical_per_m=E / dist,
              electrical_power=E / dur, cot=E / (mass * 9.81 * dist),
              saturated=float(sat[a:b].mean()))


def label(img, lines):
  from PIL import Image, ImageDraw, ImageFont
  im = Image.fromarray(img)
  dr = ImageDraw.Draw(im)
  try:
    font = ImageFont.truetype('DejaVuSans.ttf', 18)
  except OSError:
    font = ImageFont.load_default()
  y = 8
  for t in lines:
    dr.text((10, y), t, fill=(255, 255, 255), font=font,
            stroke_width=2, stroke_fill=(0, 0, 0))
    y += 24
  return np.asarray(im)


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawTextHelpFormatter)
  p.add_argument('--fourier', required=True)
  p.add_argument('--trajopt', required=True)
  p.add_argument('--out', required=True, help='output path without suffix')
  p.add_argument('--seconds', type=float, default=8.0)
  p.add_argument('--fps', type=int, default=30)
  p.add_argument('--slow', type=float, default=1.0)
  p.add_argument('--width', type=int, default=640)
  p.add_argument('--height', type=int, default=480)
  args = p.parse_args()
  import imageio
  C.set_robot('v2')
  m = C.load_model(dt=DT, ground=True)
  mass = C.total_mass(m)
  direction = np.array([0.0, -1.0, 0.0])
  runs = {}
  for name, fn, path in (('Fourier ladder', fourier_run, args.fourier),
                         ('Trajopt', trajopt_run, args.trajopt)):
    qpos, qvel, u, period = fn(path, m, 1)
    cycles = int(np.ceil(args.seconds / period)) + 1
    qpos, qvel, u, period = fn(path, m, cycles)
    frames, pos, mech, elec, sat = simulate(m, qpos, qvel, u, period,
                                            args.fps, args.slow)
    runs[name] = dict(frames=frames, pos=pos,
                      stats=stats(pos, mech, elec, sat, period, mass,
                                  direction, skip=1), path=path)
    s = runs[name]['stats']
    print(f'{name:15s} T {period:.3f} s  speed {s["speed"]:.4f} m/s  '
          f'lateral {s["lateral"]:+.4f}  electrical {s["electrical_power"]:.1f} W'
          f'  {s["electrical_per_m"]:.0f} J/m  mech {s["mech_work_per_m"]:.0f} J/m'
          f'  CoT {s["cot"]:.2f}  at limit {100 * s["saturated"]:.0f}%',
          flush=True)

  # Video: both from their steady strokes, same camera relative to the body.
  renderer = mujoco.Renderer(m, args.height, args.width)
  d = mujoco.MjData(m)
  plate = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'plate')
  cam = mujoco.MjvCamera()
  cam.type = mujoco.mjtCamera.mjCAMERA_FREE
  cam.distance, cam.azimuth, cam.elevation = 1.4, 150.0, -28.0
  nframe = min(len(r['frames']) for r in runs.values())
  nframe = min(nframe, int(args.seconds * args.fps / args.slow))
  video = []
  for f in range(nframe):
    tiles = []
    for name, r in runs.items():
      d.qpos[:] = r['frames'][f]
      mujoco.mj_forward(m, d)
      cam.lookat[:] = d.xpos[plate]
      renderer.update_scene(d, cam)
      img = renderer.render()
      dist = float(direction @ (r['frames'][f][:3] - r['frames'][0][:3]))
      s = r['stats']
      tiles.append(label(img, [
          f'{name}: {s["speed"]:.3f} m/s, {s["period"]:.2f} s stroke',
          f'{s["electrical_power"]:.0f} W, {s["electrical_per_m"]:.0f} J/m',
          f't {f * args.slow / args.fps:4.1f} s   swum {dist:5.2f} m']))
    video.append(np.concatenate(tiles, axis=1))
  renderer.close()
  os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
  imageio.mimsave(args.out + '.mp4', video, fps=args.fps, macro_block_size=1)
  imageio.imwrite(args.out + '_frame.png', video[len(video) // 2])
  with open(args.out + '.json', 'w') as fh:
    json.dump({k: dict(path=r['path'], **r['stats']) for k, r in runs.items()},
              fh, indent=1)


if __name__ == '__main__':
  main()
