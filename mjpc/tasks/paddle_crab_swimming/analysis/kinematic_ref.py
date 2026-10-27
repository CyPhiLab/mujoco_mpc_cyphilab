#!/usr/bin/env python3
"""Kinematic (no physics) motion reference for the paddle crab, keyframed by
eye against swimming-crab footage.

  kinematic_ref.py REF.json --out DIR [--video NAME=PATH ...] [--cycles 4]

The reference (JSON) has one paddle "stroke" shared by every swimming mode
and the "modes" that use it (e.g. sideways, paddles_first). A stroke channel
is periodic keyframes over one stroke, [[phase, angle_rad], ...] with phase
in [0, 1), interpolated by a periodic cubic (Catmull-Rom) spline; phase 0 is
the paddle upright at the start of its power stroke. A mode has a "base"
and a "joints" table, each joint one of:
  {"keys": [[phase, angle], ...]}         its own keyframes (one key: held)
  {"stroke": "mid", "sign": 1, "offset": 0, "phase_shift": 0}
                                          a shared stroke channel
  {"mirror_of": "psbr", "sign": -1, "offset": 0, "phase_shift": 0}
                                          another joint of the same mode
The paddle's sweep plane is set by the proximal "offset" (its heading), so
one stroke drives a sideways and a paddles-first sweep alike. The base moves
at a constant speed along "direction", tilted leading edge up by
"lead_up_deg" and rolled by "roll_deg" about the swim direction, with an
optional periodic "heave" (keys in metres).

Joint conventions (checked with forward kinematics):
  mid and distal joints: + raises, on both sides
  proximal: + turns a right leg toward +Y (forward) and a left leg toward -Y
  rear legs (paddles): proximal +0.79 (right) / -0.79 (left) points the leg
    out along +X / -X, -0.79 (right) / +0.79 (left) points it back (-Y);
    raised vertical (mid ~1.57) a feather (distal) of 0 puts the blade
    broadside to its sweep, +-1.57 edge-on
  front legs: proximal +0.79 (right) / -0.79 (left) points them forward (+Y)

Footage ("footage": {NAME: {fps, frames, phase0_frame, period, mode,
mirror, camera, base, crop}}) is never flipped. When the crab swims toward
its left, "mirror": true mirrors the robot instead (left and right legs
swapped, travel along -X). "camera" is {"azimuth": world deg (the camera
looks along (cos, sin); -90 looks along -Y, at the robot's front),
"elevation": deg (+ looks up from below), "distance": m}; "azimuth_offset"
instead is relative to the swim heading. "base" overrides the mode's body
attitude for that clip only.

Robot colours: right legs orange, left legs blue, paddles the darker shade.

Writes to DIR:
  robot_MODE.mp4      each mode from the side and from above-behind
  compare_NAME.mp4    per --video clip, synced by stroke phase: the raw
                      footage | the robot from the clip's camera | the robot
                      from above-behind (fixed overview)
  sheet_NAME.png      one stroke at 8 phases, the same three rows
  reference.npz       one stroke of each mode sampled at 200 phases: qpos
"""
import argparse
import functools
import json
import os

import mujoco
import numpy as np

import crab_model as C

JOINT_INDEX = {name: i for i, name in enumerate(C.JOINTS)}


def periodic_spline(keys, phase):
  """Periodic Catmull-Rom interpolation of [[phase, value], ...]."""
  keys = sorted((float(p) % 1.0, float(v)) for p, v in keys)
  if len(keys) == 1:
    return np.full_like(np.asarray(phase, float), keys[0][1])
  ph = np.array([k[0] for k in keys])
  val = np.array([k[1] for k in keys])
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


def pose_spec(ref, mode, clip=None):
  """Everything the animation of one mode needs: the shared stroke, the
  mode's joints and base (with a clip's base overrides) and the mirror."""
  clip = clip or {}
  spec = ref['modes'][mode]
  return dict(stroke=ref['stroke'], joints=spec['joints'],
              base=dict(spec.get('base', {}), **clip.get('base', {})),
              mirror=clip.get('mirror', False), mode=mode)


def joint_value(pose, name, phase, depth=0):
  assert depth < 18, f'mirror_of loop at {name}'
  entry = pose['joints'].get(name, {'keys': [[0, 0.0]]})
  if 'keys' in entry:
    return periodic_spline(entry['keys'], phase)
  shifted = phase - entry.get('phase_shift', 0.0)
  if 'stroke' in entry:
    value = periodic_spline(pose['stroke'][entry['stroke']], shifted)
  else:
    value = joint_value(pose, entry['mirror_of'], shifted, depth + 1)
  return entry.get('sign', 1.0) * value + entry.get('offset', 0.0)


def joint_angles(pose, phase):
  """(len(phase), 18) joint angles, mirrored left-right if the pose says."""
  phase = np.atleast_1d(np.asarray(phase, float))
  q = np.stack([joint_value(pose, name, phase) for name in C.JOINTS], axis=1)
  if pose['mirror']:
    q = C.MIRROR_SIGN * q[:, C.MIRROR_INDEX]
  return q


def swim_direction(pose):
  direction = np.asarray(pose['base'].get('direction', [1, 0, 0]), float)
  direction = direction / np.linalg.norm(direction)
  if pose['mirror']:
    direction = direction * [-1, 1, 1]
  return direction


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


def qpos_at(pose, m, t, period):
  """Full qpos at time t for a stroke period."""
  base = pose['base']
  direction = swim_direction(pose)
  phase = (t / period) % 1.0
  qpos = m.qpos0.copy()
  heave = periodic_spline(base.get('heave', [[0, 0.0]]), phase)
  qpos[:3] = m.qpos0[:3] + direction * base.get('speed', 0.0) * t \
      + np.array([0, 0, float(heave)])
  side = np.cross(direction, [0.0, 0.0, 1.0])     # lead-up axis
  roll = base.get('roll_deg', 0.0) * (-1 if pose['mirror'] else 1)
  quat = quat_mul(axis_angle_quat(direction, np.radians(roll)),
                  axis_angle_quat(side, np.radians(base.get('lead_up_deg',
                                                            0.0))))
  qpos[3:7] = quat
  qpos[7:] = joint_angles(pose, [phase])[0]
  return qpos


# ----- Rendering ----- #

OVERVIEW = dict(azimuth_offset=45.0, elevation=-35.0, distance=1.0)
SIDE = dict(azimuth_offset=90.0, elevation=-10.0, distance=1.0)
COLOURS = {'right': (0.95, 0.55, 0.15), 'left': (0.2, 0.45, 0.9)}
BACKGROUND = (np.array([238, 242, 247]), np.array([196, 208, 222]))


@functools.lru_cache(maxsize=1)
def render_model():
  """The crab with a directional light and colour-coded legs (no floor)."""
  xml = C.patched_xml().replace('<include file="hfield_ground.xml" />',
                                '<compiler meshdir="asset" />')
  xml = xml.replace('<worldbody>', '<worldbody>\n<light directional="true" '
                    'pos="0 0 5" dir="0.3 0.4 -1" diffuse=".55 .55 .55" '
                    'specular=".1 .1 .1" castshadow="false"/>', 1)
  m = C._compile(xml)
  m.vis.headlight.ambient[:] = 0.45
  m.vis.headlight.diffuse[:] = 0.35
  m.vis.headlight.specular[:] = 0.05
  for g in range(m.ngeom):
    body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
    if m.geom_rgba[g, 3] == 0 or body is None:
      continue
    side = next((s for s in COLOURS if body.endswith(s)), None)
    if side is None:
      m.geom_rgba[g] = (0.55, 0.55, 0.58, 1)       # body
      continue
    shade = 0.6 if 'back' in body else 1.0         # paddles darker
    m.geom_rgba[g, :3] = np.array(COLOURS[side]) * shade
    m.geom_rgba[g, 3] = 1
  return m


def make_camera(direction, azimuth=None, azimuth_offset=0.0, elevation=-10.0,
                distance=1.0):
  """A free camera tracking the body. azimuth is in world degrees (looking
  along (cos, sin)); azimuth_offset is relative to the swim heading (90:
  travel toward screen right; 0: looking along the travel, from behind).
  elevation: + looks up from below."""
  cam = mujoco.MjvCamera()
  cam.type = mujoco.mjtCamera.mjCAMERA_FREE
  if azimuth is None:
    azimuth = np.degrees(np.arctan2(direction[1], direction[0])) \
        + azimuth_offset
  cam.azimuth = azimuth
  cam.elevation = elevation
  cam.distance = distance
  return cam


def render_frames(pose, times, period, camera, width=480, height=360):
  """Robot frames at the given times on a light background; camera is a
  dict of make_camera arguments."""
  m = render_model()
  d = mujoco.MjData(m)
  renderer = mujoco.Renderer(m, height, width)
  depth = mujoco.Renderer(m, height, width)
  depth.enable_depth_rendering()
  cam = make_camera(swim_direction(pose), **camera)
  plate = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'plate')
  ramp = np.linspace(0, 1, height)[:, None, None]
  sky = ((1 - ramp) * BACKGROUND[0] + ramp * BACKGROUND[1]).astype(np.uint8)
  frames = []
  for t in times:
    d.qpos[:] = qpos_at(pose, m, t, period)
    mujoco.mj_kinematics(m, d)
    cam.lookat[:] = d.xpos[plate]
    renderer.update_scene(d, cam)
    depth.update_scene(d, cam)
    rgb = renderer.render()
    z = depth.render()
    empty = (z >= z.max() - 1e-6)[..., None] & (rgb.sum(-1) == 0)[..., None]
    frames.append(np.where(empty, np.broadcast_to(sky, rgb.shape), rgb))
  renderer.close()
  depth.close()
  return frames


def label(frame, lines, colour=(20, 20, 20)):
  """Text lines drawn on the top-left of a frame."""
  import PIL.Image
  import PIL.ImageDraw
  import PIL.ImageFont
  img = PIL.Image.fromarray(frame)
  draw = PIL.ImageDraw.Draw(img)
  try:
    font = PIL.ImageFont.load_default(size=15)
  except TypeError:
    font = PIL.ImageFont.load_default()
  y = 6
  for line in lines:
    x0, y0, x1, y1 = draw.textbbox((8, y), line, font=font)
    draw.rectangle((x0 - 3, y0 - 2, x1 + 3, y1 + 2), fill=(255, 255, 255))
    draw.text((8, y), line, fill=colour, font=font)
    y = y1 + 6
  return np.asarray(img)


def screen_direction(direction, camera):
  """'left', 'right', 'toward' or 'away': how travel looks on screen."""
  cam = make_camera(direction, **camera)
  a = np.radians(cam.azimuth)
  look = np.array([np.cos(a), np.sin(a), 0.0])
  right = np.cross(look, [0, 0, 1])
  r, f = direction @ right, direction @ look
  if abs(r) >= abs(f):
    return 'swims to screen right' if r > 0 else 'swims to screen left'
  return 'swims away from camera' if f > 0 else 'swims toward camera'


def crab_frames(video, first, last, width=480, height=360, crop=(1100, 825)):
  """Raw footage frames first..last-1 (never mirrored), cropped around the
  (dark) crab, whose centre is tracked and smoothed."""
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
    x0 = int(np.clip(cx - 0.5 * cw, 0, frame.shape[1] - cw))
    y0 = int(np.clip(cy - 0.5 * ch, 0, frame.shape[0] - ch))
    tile = frame[y0:y0 + ch, x0:x0 + cw]
    out.append(np.asarray(PIL.Image.fromarray(tile).resize((width, height))))
  return out


def compare_clip(ref, name, video, out, fps, slow):
  """compare_NAME.mp4 (footage | robot, clip view | robot, overview; synced
  by stroke phase) and sheet_NAME.png (one stroke at 8 phases)."""
  import imageio
  import PIL.Image
  clip = ref['footage'][name]
  fps_crab = clip['fps']
  first, last = clip['frames']
  period = clip['period']
  footage = crab_frames(video, first, last,
                        crop=tuple(clip.get('crop', (1100, 825))))
  pose = pose_spec(ref, clip['mode'], clip)
  camera = dict(SIDE, **clip.get('camera', {}))
  where = screen_direction(swim_direction(pose), camera)
  mirrored = ' (mirrored: left legs lead)' if pose['mirror'] else ''
  titles = ([f'crab {name}, raw footage', clip.get('describe', '')],
            [f'robot "{clip["mode"]}"{mirrored}', 'same view as footage',
             where],
            ['robot, overview from above-behind',
             'orange = right legs, blue = left',
             'dark = paddles'])
  # output frame k shows real time tau = k / (fps * slow) of the footage;
  # the robot runs on the clip's stroke period, phase 0 at phase0_frame
  n_out = int((last - first) / fps_crab * fps * slow)
  taus = np.arange(n_out) / (fps * slow)
  crab_idx = np.minimum((taus * fps_crab).astype(int), len(footage) - 1)
  t_robot = taus + (first - clip['phase0_frame']) / fps_crab
  match = render_frames(pose, t_robot, period, camera)
  over = render_frames(pose, t_robot, period, OVERVIEW)
  video_frames = []
  for i, (a, b, t) in enumerate(zip(match, over, t_robot)):
    phase = f'stroke phase {(t / period) % 1:.2f}'
    video_frames.append(np.concatenate(
        [label(footage[crab_idx[i]], titles[0] + [phase]),
         label(a, titles[1]), label(b, titles[2])], axis=1))
  imageio.mimsave(os.path.join(out, f'compare_{name}.mp4'), video_frames,
                  fps=fps, macro_block_size=1)
  phases = np.arange(8) / 8
  frames_at = [int(round(clip['phase0_frame'] - first + p * period * fps_crab))
               for p in phases]
  frames_at = [min(max(f, 0), len(footage) - 1) for f in frames_at]
  rows = [[label(footage[f], [f'frame {first + f}', f'phase {p:.3f}'])
           for f, p in zip(frames_at, phases)],
          [label(r, [f'phase {p:.3f}'])
           for r, p in zip(render_frames(pose, phases * period, period,
                                         camera), phases)],
          [label(r, [f'phase {p:.3f}'])
           for r, p in zip(render_frames(pose, phases * period, period,
                                         OVERVIEW), phases)]]
  rows = [np.concatenate(r, axis=1) for r in rows]
  head = np.full((44, rows[0].shape[1], 3), 255, np.uint8)
  head = label(head, [f'{name}: rows = raw footage | robot, same view '
                      f'({where}){mirrored} | robot from above-behind '
                      f'(orange right, blue left, dark = paddles)'])
  sheet = np.concatenate([head] + rows, axis=0)
  img = PIL.Image.fromarray(sheet)
  img.resize((sheet.shape[1] * 2 // 3, sheet.shape[0] * 2 // 3)).save(
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
                 help='skip robot_MODE.mp4 (comparisons only)')
  args = p.parse_args()
  import imageio
  with open(args.reference) as f:
    ref = json.load(f)
  os.makedirs(args.out, exist_ok=True)
  period = ref['period']
  times = np.arange(0, args.cycles * period, 1 / args.fps)
  m = C.load_model(ground=False, ctrl_limit=np.pi)
  phase = np.arange(200) / 200
  table = {}
  for mode in ref['modes']:
    pose = pose_spec(ref, mode)
    table[mode] = np.array([qpos_at(pose, m, ph * period, period)
                            for ph in phase])
    if args.no_views:
      continue
    side = render_frames(pose, times, period, SIDE)
    over = render_frames(pose, times, period, OVERVIEW)
    imageio.mimsave(
        os.path.join(args.out, f'robot_{mode}.mp4'),
        [np.concatenate([label(a, [f'"{mode}", side view',
                                   screen_direction(swim_direction(pose),
                                                    SIDE)]),
                         label(b, ['from above-behind',
                                   'orange = right, blue = left'])], axis=1)
         for a, b in zip(side, over)], fps=args.fps, macro_block_size=1)
  np.savez(os.path.join(args.out, 'reference.npz'), phase=phase,
           period=period, joints=np.array(C.JOINTS),
           reference=json.dumps(ref),
           **{f'qpos_{mode}': q for mode, q in table.items()})
  for item in args.video:
    name, path = item.split('=', 1)
    compare_clip(ref, name, path, args.out, args.fps, args.slow)
  print('wrote', args.out)


if __name__ == '__main__':
  main()
