#!/usr/bin/env python3
"""Kinematic (no physics) motion reference for the paddle crab, keyframed by
eye against swimming-crab footage.

  kinematic_ref.py REF.json --out DIR [--compare VIDEO] [--cycles 4]

The reference (JSON) gives every joint as periodic keyframes over one stroke,
[[phase, angle_rad], ...] with phase in [0, 1), interpolated by a periodic
cubic (Catmull-Rom) spline; a single key holds the joint constant. A joint
can instead mirror another: {"mirror_of": "psbr", "sign": -1,
"phase_shift": 0.0}. The base moves at a constant speed along "direction",
tilted leading edge up by "lead_up_deg" and rolled by "roll_deg" about the
swim direction, with an optional periodic "heave" (keys in metres).
"camera": {"tank_elevation": deg, "oblique_elevation": deg} sets the views
(positive looks up from below).

Joint conventions (checked with forward kinematics):
  mid and distal joints: + raises, on both sides
  proximal: + turns a right leg toward +Y and a left leg toward -Y
  rear legs (paddles): with the proximal at +0.79 (right) / -0.79 (left) the
    leg points along +X / -X; raised vertical (mid ~1.57) a feather (distal)
    of 0 puts the blade broadside to a sweep along X, +-1.57 edge-on

Writes to DIR:
  robot_tank.mp4      the animation from the side, like the tank camera
                      (looking along the paddles' +Y, travel to the right)
  robot_oblique.mp4   three-quarter view from above
  compare.mp4         with --compare: the footage (tracked, cropped) and the
                      robot side by side, the robot timed to the crab's stroke
  reference.npz       one stroke sampled at 200 phases: qpos, and the period
"""
import argparse
import json
import os

import mujoco
import numpy as np

import crab_model as C

JOINT_INDEX = {name: 7 + i for i, name in enumerate(C.JOINTS)}


def periodic_spline(keys, phase):
  """Periodic Catmull-Rom interpolation of [[phase, value], ...]."""
  keys = sorted((float(p) % 1.0, float(v)) for p, v in keys)
  if len(keys) == 1:
    return np.full_like(np.asarray(phase, float), keys[0][1])
  ph = np.array([k[0] for k in keys])
  val = np.array([k[1] for k in keys])
  n = len(keys)
  # extended knots for wrap-around
  ph_ext = np.concatenate([ph[-2:] - 1, ph, ph[:2] + 1])
  val_ext = np.concatenate([val[-2:], val, val[:2]])
  phase = np.asarray(phase, float) % 1.0
  out = np.empty_like(phase)
  for idx, x in np.ndenumerate(phase):
    j = np.searchsorted(ph_ext, x, side='right') - 1
    j = min(max(j, 1), len(ph_ext) - 3)
    t0, t1 = ph_ext[j], ph_ext[j + 1]
    v0, v1 = val_ext[j], val_ext[j + 1]
    m0 = (val_ext[j + 1] - val_ext[j - 1]) / (ph_ext[j + 1] - ph_ext[j - 1])
    m1 = (val_ext[j + 2] - val_ext[j]) / (ph_ext[j + 2] - ph_ext[j])
    h = t1 - t0
    s = (x - t0) / h
    h00, h10 = 2 * s ** 3 - 3 * s ** 2 + 1, s ** 3 - 2 * s ** 2 + s
    h01, h11 = -2 * s ** 3 + 3 * s ** 2, s ** 3 - s ** 2
    out[idx] = h00 * v0 + h10 * h * m0 + h01 * v1 + h11 * h * m1
  return out


def joint_angles(ref, phase):
  """(len(phase), 18) joint angles."""
  phase = np.atleast_1d(np.asarray(phase, float))
  q = np.zeros((len(phase), C.NU))
  spec = ref['joints']
  for name in C.JOINTS:
    entry = spec.get(name, {'keys': [[0, 0.0]]})
    if 'mirror_of' in entry:
      src = spec[entry['mirror_of']]
      q[:, JOINT_INDEX[name] - 7] = entry.get('sign', 1.0) * periodic_spline(
          src['keys'], phase - entry.get('phase_shift', 0.0)) + \
          entry.get('offset', 0.0)
    else:
      q[:, JOINT_INDEX[name] - 7] = periodic_spline(entry['keys'], phase)
  return q


def axis_angle_quat(axis, angle):
  axis = np.asarray(axis, float)
  axis = axis / np.linalg.norm(axis)
  return np.concatenate([[np.cos(angle / 2)], np.sin(angle / 2) * axis])


def quat_mul(a, b):
  w1, x1, y1, z1 = a
  w2, x2, y2, z2 = b
  return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                   w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                   w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                   w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qpos_at(ref, m, t, period):
  """Full qpos at time t for a stroke period."""
  base = ref.get('base', {})
  direction = np.asarray(base.get('direction', [1, 0, 0]), float)
  direction /= np.linalg.norm(direction)
  phase = (t / period + ref.get('phase0', 0.0)) % 1.0
  qpos = m.qpos0.copy()
  heave = periodic_spline(base.get('heave', [[0, 0.0]]), phase)
  qpos[:3] = m.qpos0[:3] + direction * base.get('speed', 0.0) * t \
      + np.array([0, 0, float(heave)])
  side = np.cross(direction, [0.0, 0.0, 1.0])     # lead-up axis
  quat = quat_mul(axis_angle_quat(direction,
                                  np.radians(base.get('roll_deg', 0.0))),
                  axis_angle_quat(side, np.radians(base.get('lead_up_deg', 0.0))))
  qpos[3:7] = quat
  qpos[7:] = joint_angles(ref, [phase])[0]
  return qpos


def make_camera(m, view, direction, distance=1.2, elevation=None):
  cam = mujoco.MjvCamera()
  cam.type = mujoco.mjtCamera.mjCAMERA_FREE
  heading = np.degrees(np.arctan2(direction[1], direction[0]))
  if view == 'tank':
    cam.azimuth, cam.elevation = heading + 90, -5.0 if elevation is None \
        else elevation
  else:
    cam.azimuth, cam.elevation = heading + 140, -28.0 if elevation is None \
        else elevation
  cam.distance = distance
  return cam


def render_frames(ref, times, period, view, width=640, height=480):
  m = C.load_model(ground=True, ctrl_limit=np.pi)
  d = mujoco.MjData(m)
  renderer = mujoco.Renderer(m, height, width)
  direction = np.asarray(ref.get('base', {}).get('direction', [1, 0, 0]),
                         float)
  cam = make_camera(m, view, direction,
                    elevation=ref.get('camera', {}).get(f'{view}_elevation'))
  plate = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'plate')
  frames = []
  for t in times:
    d.qpos[:] = qpos_at(ref, m, t, period)
    mujoco.mj_kinematics(m, d)
    cam.lookat[:] = d.xpos[plate]
    renderer.update_scene(d, cam)
    frames.append(renderer.render())
  return frames


def crab_frames(video, first, last, width=640, height=480):
  """Footage frames first..last-1, cropped around the (dark) crab, whose
  centre is tracked and smoothed."""
  import imageio.v3 as iio
  import PIL.Image
  raw, centres = [], []
  for i, frame in enumerate(iio.imiter(video, plugin='pyav')):
    if i < first:
      continue
    if i >= last:
      break
    g = frame.mean(axis=-1)
    mask = g < 0.45 * np.median(g)
    mask[int(0.85 * g.shape[0]):] = False
    ys, xs = np.nonzero(mask)
    centres.append((np.median(xs), np.median(ys)) if len(xs) > 200
                   else centres[-1] if centres else (g.shape[1] / 2,
                                                     g.shape[0] / 2))
    raw.append(frame)
  c = np.array(centres)
  k = 15
  smooth = np.array([c[max(0, i - k):i + k + 1].mean(axis=0)
                     for i in range(len(c))])
  out = []
  cw, ch = 1100, 825                        # crop, same aspect as the render
  for frame, (cx, cy) in zip(raw, smooth):
    x0 = int(np.clip(cx - 0.6 * cw, 0, frame.shape[1] - cw))
    y0 = int(np.clip(cy - 0.5 * ch, 0, frame.shape[0] - ch))
    crop = frame[y0:y0 + ch, x0:x0 + cw]
    out.append(np.asarray(PIL.Image.fromarray(crop).resize((width, height))))
  return out


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawTextHelpFormatter)
  p.add_argument('reference')
  p.add_argument('--out', required=True)
  p.add_argument('--cycles', type=float, default=4)
  p.add_argument('--fps', type=float, default=30)
  p.add_argument('--compare', help='footage to show side by side')
  p.add_argument('--slow', type=float, default=6,
                 help='slow motion of the comparison video')
  args = p.parse_args()
  import imageio
  with open(args.reference) as f:
    ref = json.load(f)
  os.makedirs(args.out, exist_ok=True)
  period = ref['period']
  times = np.arange(0, args.cycles * period, 1 / args.fps)
  for view in ('tank', 'oblique'):
    frames = render_frames(ref, times, period, view)
    imageio.mimsave(os.path.join(args.out, f'robot_{view}.mp4'), frames,
                    fps=args.fps, macro_block_size=1)
  m = C.load_model(ground=False, ctrl_limit=np.pi)
  phase = np.arange(200) / 200
  qpos = np.array([qpos_at(ref, m, ph * period, period) for ph in phase])
  np.savez(os.path.join(args.out, 'reference.npz'), phase=phase, qpos=qpos,
           period=period, joints=np.array(C.JOINTS),
           reference=json.dumps(ref))
  if args.compare:
    crab = ref['crab']
    fps_crab = crab['fps']
    first, last = crab['frames']
    footage = crab_frames(args.compare, first, last)
    # output frame k shows real time tau = k / (fps * slow) of the footage
    n_out = int((last - first) / fps_crab * args.fps * args.slow)
    taus = np.arange(n_out) / (args.fps * args.slow)
    crab_idx = np.minimum((taus * fps_crab).astype(int), len(footage) - 1)
    # the robot runs at the crab's stroke period, phase-aligned to the
    # footage: phase 0 at crab frame phase0_frame
    t_robot = taus + (first - crab['phase0_frame']) / fps_crab
    robot = render_frames(dict(ref, phase0=0.0), t_robot, crab['period'],
                          'tank')
    frames = [np.concatenate([footage[i], r], axis=1)
              for i, r in zip(crab_idx, robot)]
    imageio.mimsave(os.path.join(args.out, 'compare.mp4'), frames,
                    fps=args.fps, macro_block_size=1)
  print('wrote', args.out)


if __name__ == '__main__':
  main()
