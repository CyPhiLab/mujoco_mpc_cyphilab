#!/usr/bin/env python3
"""Kinematic (no physics) motion reference for the paddle crab, keyframed by
eye against swimming-crab footage.

  kinematic_ref.py REF.json --out DIR [--video NAME=PATH ...] [--cycles 4]

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
  compare_NAME.mp4    per --video clip: the footage (tracked, cropped, flipped
                      if the clip says so) and the robot side by side, the
                      robot timed to the clip's stroke and seen from the
                      clip's camera ("footage": {NAME: {fps, frames,
                      phase0_frame, period, camera, base, flip, crop}};
                      "base" overrides the body attitude for that clip only)
  sheet_NAME.png      one stroke at 8 phases: footage above, robot below
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


VIEWS = {'tank': dict(azimuth_offset=90.0, elevation=-5.0, distance=1.2),
         'oblique': dict(azimuth_offset=140.0, elevation=-28.0, distance=1.2)}


def make_camera(direction, azimuth_offset, elevation, distance=1.2):
  """A free camera tracking the body; the azimuth is relative to the swim
  direction (90: looking along +Y for +X travel, from the paddles' side,
  travel to the right; -90: from the front, travel to the left)."""
  cam = mujoco.MjvCamera()
  cam.type = mujoco.mjtCamera.mjCAMERA_FREE
  heading = np.degrees(np.arctan2(direction[1], direction[0]))
  cam.azimuth = heading + azimuth_offset
  cam.elevation = elevation
  cam.distance = distance
  return cam


def view_camera(ref, view):
  """Camera settings of a named view, with the reference's overrides
  (camera: {"tank_elevation": ...} etc.)."""
  cam = dict(VIEWS[view])
  for key in ('azimuth_offset', 'elevation', 'distance'):
    if f'{view}_{key}' in ref.get('camera', {}):
      cam[key] = ref['camera'][f'{view}_{key}']
  return cam


def render_frames(ref, times, period, camera, width=640, height=480,
                  ground=True):
  """Robot frames at the given times; camera is a dict of make_camera
  arguments (azimuth_offset, elevation, distance)."""
  m = C.load_model(ground=ground, ctrl_limit=np.pi)
  d = mujoco.MjData(m)
  renderer = mujoco.Renderer(m, height, width)
  direction = np.asarray(ref.get('base', {}).get('direction', [1, 0, 0]),
                         float)
  cam = make_camera(direction, **camera)
  plate = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'plate')
  frames = []
  for t in times:
    d.qpos[:] = qpos_at(ref, m, t, period)
    mujoco.mj_kinematics(m, d)
    cam.lookat[:] = d.xpos[plate]
    renderer.update_scene(d, cam)
    frames.append(renderer.render())
  return frames


def crab_frames(video, first, last, width=640, height=480, crop=(1100, 825),
                flip=False):
  """Footage frames first..last-1, cropped around the (dark) crab, whose
  centre is tracked and smoothed; flip mirrors them left-right."""
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
  cw, ch = crop                             # same aspect as the render
  for frame, (cx, cy) in zip(raw, smooth):
    x0 = int(np.clip(cx - 0.6 * cw, 0, frame.shape[1] - cw))
    y0 = int(np.clip(cy - 0.5 * ch, 0, frame.shape[0] - ch))
    tile = frame[y0:y0 + ch, x0:x0 + cw]
    if flip:
      tile = tile[:, ::-1]
    out.append(np.asarray(PIL.Image.fromarray(tile).resize((width, height))))
  return out


def compare_clip(ref, name, video, out, fps, slow):
  """compare_NAME.mp4 (footage | robot, synced by stroke phase) and
  sheet_NAME.png (one stroke at 8 phases, footage above robot)."""
  import imageio
  import PIL.Image
  clip = ref['footage'][name]
  fps_crab = clip['fps']
  first, last = clip['frames']
  footage = crab_frames(video, first, last, crop=tuple(clip.get('crop',
                                                                (1100, 825))),
                        flip=clip.get('flip', False))
  camera = dict(VIEWS['tank'], **clip.get('camera', {}))
  # The joint motion is shared by every clip; the body's attitude in the
  # tank (and the camera) are per clip.
  robot_ref = dict(ref, phase0=0.0,
                   base=dict(ref.get('base', {}), **clip.get('base', {})))
  # output frame k shows real time tau = k / (fps * slow) of the footage;
  # the robot runs on the clip's stroke period, phase 0 at phase0_frame
  n_out = int((last - first) / fps_crab * fps * slow)
  taus = np.arange(n_out) / (fps * slow)
  crab_idx = np.minimum((taus * fps_crab).astype(int), len(footage) - 1)
  t_robot = taus + (first - clip['phase0_frame']) / fps_crab
  robot = render_frames(robot_ref, t_robot, clip['period'], camera,
                        ground=False)
  imageio.mimsave(os.path.join(out, f'compare_{name}.mp4'),
                  [np.concatenate([footage[i], r], axis=1)
                   for i, r in zip(crab_idx, robot)],
                  fps=fps, macro_block_size=1)
  phases = np.arange(8) / 8
  frames_at = [int(round(clip['phase0_frame'] - first
                         + p * clip['period'] * fps_crab)) for p in phases]
  frames_at = [min(max(f, 0), len(footage) - 1) for f in frames_at]
  robot = render_frames(robot_ref, phases * clip['period'], clip['period'],
                        camera, ground=False)
  top = np.concatenate([footage[f] for f in frames_at], axis=1)
  bottom = np.concatenate(robot, axis=1)
  sheet = np.concatenate([top, bottom], axis=0)
  img = PIL.Image.fromarray(sheet)
  img.resize((sheet.shape[1] // 2, sheet.shape[0] // 2)).save(
      os.path.join(out, f'sheet_{name}.png'))


def main():
  p = argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawTextHelpFormatter)
  p.add_argument('reference')
  p.add_argument('--out', required=True)
  p.add_argument('--cycles', type=float, default=4)
  p.add_argument('--fps', type=float, default=30)
  p.add_argument('--video', action='append', default=[],
                 help='NAME=PATH of a clip listed under "footage"; repeatable')
  p.add_argument('--slow', type=float, default=6,
                 help='slow motion of the comparison videos')
  p.add_argument('--no_views', action='store_true',
                 help='skip robot_tank/robot_oblique (comparisons only)')
  args = p.parse_args()
  import imageio
  with open(args.reference) as f:
    ref = json.load(f)
  os.makedirs(args.out, exist_ok=True)
  period = ref['period']
  times = np.arange(0, args.cycles * period, 1 / args.fps)
  for view in ([] if args.no_views else VIEWS):
    frames = render_frames(ref, times, period, view_camera(ref, view))
    imageio.mimsave(os.path.join(args.out, f'robot_{view}.mp4'), frames,
                    fps=args.fps, macro_block_size=1)
  m = C.load_model(ground=False, ctrl_limit=np.pi)
  phase = np.arange(200) / 200
  qpos = np.array([qpos_at(ref, m, ph * period, period) for ph in phase])
  np.savez(os.path.join(args.out, 'reference.npz'), phase=phase, qpos=qpos,
           period=period, joints=np.array(C.JOINTS),
           reference=json.dumps(ref))
  for item in args.video:
    name, path = item.split('=', 1)
    compare_clip(ref, name, path, args.out, args.fps, args.slow)
  print('wrote', args.out)


if __name__ == '__main__':
  main()
