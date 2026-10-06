#!/usr/bin/env python3
"""Pterosaur robot with wings for flight studies (MuJoCo's ellipsoid fluid
model).

Adds the CAD wing fingers (commented out in pterosaur_torque.xml) with a
fold/extend motor each, and a pterosaur-like membrane: thin ellipsoid panels
whose leading edge runs along the humerus, forearm and wing finger, chord
tapering from the body to the wingtip, placed in the spread-wing flight pose
so they fold with their bones. Panels have collisions off, membrane mass, and
fluidshape="ellipsoid" (lift from the Kutta term, CL ~ ck sin(2 alpha); drag
~ 2 cb for a thin panel above ~1 deg). The rest of the robot keeps MuJoCo's
default inertia-box fluid model once the air density is on.

The finger joints come after each elbow in joint order, so this model's
joint indices differ from launch_trajopt.load_model's; use names.

Usage (checks and renders):
  wing_model.py [--render out.png]
"""
import argparse
import os

import mujoco
import numpy as np

import launch_trajopt as T

LIMB_JOINTS = ['leftarm_shoulder1', 'leftarm_shoulder2', 'leftarm_elbow',
               'rightarm_shoulder1', 'rightarm_shoulder2', 'rightarm_elbow',
               'leftleg_hip1', 'leftleg_hip2', 'leftleg_knee',
               'rightleg_hip1', 'rightleg_hip2', 'rightleg_knee']
FINGER_JOINTS = ['leftarm_finger', 'rightarm_finger']

# spread-wing pose (left side; right mirrored with MIRROR_ARM): arm and
# finger as a straight lateral spar with the trunk level (span 4.96 m)
FLIGHT_LEFT = {'leftarm_shoulder1': -1.491, 'leftarm_shoulder2': 0.258,
               'leftarm_elbow': -0.679, 'leftarm_finger': -3.595}
MIRROR_ARM = {'shoulder1': -1.0, 'shoulder2': 1.0, 'elbow': 1.0, 'finger': 1.0}

# membrane panels (per wing): (parent body of the left wing, spar start,
# spar end, chord m); spar points are joint anchors or the wingtip site;
# 'mid' splits the finger in two
PANELS = [('bevel_out', 'shoulder2', 'elbow', 0.62),
          ('radius_and_ulna', 'elbow', 'finger', 0.52),
          ('finger', 'finger', 'mid', 0.42),
          ('finger', 'mid', 'tip', 0.24)]
RIGHT_BODY = {'bevel_out': 'bevel_out_2', 'radius_and_ulna': 'radius_and_ulna_2',
              'finger': 'finger_2'}
MEMBRANE_DENSITY = 0.3          # kg/m^2
PANEL_THICKNESS = 0.002         # m
# fluidcoef: blunt drag, slender drag, angular drag, Kutta lift, Magnus lift
FLUIDCOEF = [0.0175, 0.01, 1.5, 2.8, 0.0]
AIR_DENSITY = 1.225
AIR_VISCOSITY = 1.8e-5
FINGER_STALL = 40.0             # N m, finger fold/extend motor
# body drag: a sphere at the trunk's center of mass with drag area
# CdA = 2 * C_blunt * pi r^2 = 0.1 m^2 (trunk, motors, hanging legs),
# independent of attitude; the other non-wing bodies get no air forces
# (MuJoCo's default inertia-box model would treat the trunk as a
# 0.11 x 0.37 x 0.6 m box, and a flat ellipsoid gives plate-like drag at
# any angle of attack)
FUSELAGE = [0.178, 0.178, 0.178]
FUSELAGE_COEF = [0.5, 0.0, 0.0, 0.0, 0.0]


def flight_pose():
  """{joint name: angle} of the spread-wing pose, both sides."""
  pose = dict(FLIGHT_LEFT)
  for name, v in FLIGHT_LEFT.items():
    pose[name.replace('left', 'right')] = v * MIRROR_ARM[name.split('_')[1]]
  return pose


def add_fingers(spec):
  """The CAD wing fingers (pterosaur_torque.xml, commented out there)."""
  for side, parent in (('left', 'radius_and_ulna'), ('right', 'radius_and_ulna_2')):
    b = spec.body(parent).add_body()
    b.name = 'finger' if side == 'left' else 'finger_2'
    b.pos = [-0.6604, 0, -0.0127]
    b.quat = [0, 0.224528, 0.974468, 0]
    j = b.add_joint()
    j.name = f'{side}arm_finger'
    j.type = mujoco.mjtJoint.mjJNT_HINGE
    j.axis = [0, 0, 1]
    b.mass = 0.0823581
    b.ipos = [-0.346533, 0.00351695, -0.00634993]
    b.fullinertia = [1.11507e-05, 0.00430132, 0.00431156, 0.000126744, 9.11e-10, -8.1e-11]
    b.explicitinertial = True
    g = b.add_geom()
    g.type = mujoco.mjtGeom.mjGEOM_MESH
    g.meshname = 'finger'
    g.pos = [1.47195, -0.890746, -0.1651]
    g.quat = [0.707107, 0.707107, 0, 0]
    g.contype = g.conaffinity = 0
    g.group = 2
    s = b.add_site()
    s.name = f'{side}_wingtip'
    s.pos = [-1.0, 0, 0]
    s.size = [0.02, 0, 0]


def add_membrane(spec, m, d, fluidcoef, membrane_density):
  """Membrane panels: thin ellipsoids with the leading edge on the spar and
  the chord toward the tail (+x), placed in the current pose of d (the
  flight pose) in their parent bodies' frames."""
  # parent handles first: name lookups after adding bodies can return the
  # wrong body
  parents = {n: spec.body(n) for p in PANELS for n in (p[0], RIGHT_BODY[p[0]])}
  for side in ('left', 'right'):
    def point(name):
      if name == 'tip':
        return d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f'{side}_wingtip')]
      if name == 'mid':
        return 0.5 * (point('finger') + point('tip'))
      j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'{side}arm_{name}')
      return d.xanchor[j]
    for k, (body, a, b, chord) in enumerate(PANELS):
      body = body if side == 'left' else RIGHT_BODY[body]
      p0, p1 = point(a).copy(), point(b).copy()
      span = np.linalg.norm(p1 - p0)
      ex = (p1 - p0) / span                          # along the spar
      ey = np.array([1.0, 0, 0]) - ex[0] * ex        # chord: toward the tail
      ey /= np.linalg.norm(ey)
      ez = np.cross(ex, ey)
      center = 0.5 * (p0 + p1) + 0.5 * chord * ey
      bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
      Rb, xb = d.xmat[bid].reshape(3, 3), d.xpos[bid]
      quat = np.zeros(4)
      mujoco.mju_mat2Quat(quat, (Rb.T @ np.stack([ex, ey, ez], axis=1)).flatten())
      pb = parents[body].add_body()
      pb.name = f'{side}_membrane_{k}'
      pb.pos = Rb.T @ (center - xb)
      pb.quat = quat
      area = np.pi / 4 * span * chord
      g = pb.add_geom()
      g.name = f'{side}_membrane_{k}'
      g.type = mujoco.mjtGeom.mjGEOM_ELLIPSOID
      g.size = [span / 2, chord / 2, PANEL_THICKNESS / 2]
      g.mass = membrane_density * area
      g.contype = g.conaffinity = 0
      g.group = 1
      g.rgba = [0.85, 0.55, 0.35, 0.6]
      g.fluid_ellipsoid = 1
      g.fluid_coefs = list(fluidcoef)


def add_body_air(spec):
  """Air forces on the non-wing bodies: a body-drag sphere on the trunk,
  none on the rest (a tiny zero-coefficient fluid geom per body switches
  off MuJoCo's inertia-box model there)."""
  for b in spec.bodies:
    if b.name in ('', 'world') or 'membrane' in b.name:
      continue
    g = b.add_geom()
    g.type = mujoco.mjtGeom.mjGEOM_SPHERE
    g.contype = g.conaffinity = 0
    g.group = 5
    g.mass = 0
    g.fluid_ellipsoid = 1
    if b.name == 'body':
      g.name = 'fuselage'
      g.type = mujoco.mjtGeom.mjGEOM_ELLIPSOID
      g.size = FUSELAGE
      g.pos = b.ipos
      g.fluid_coefs = FUSELAGE_COEF
    else:
      g.size = [1e-3, 0, 0]
      g.fluid_coefs = [0, 0, 0, 0, 0]


def build(torque=2.0, no_load_speed=20.0, fluidcoef=FLUIDCOEF,
          membrane_density=MEMBRANE_DENSITY, air_density=AIR_DENSITY,
          joint_margin=T.JOINT_MARGIN):
  """The winged robot: X2 motors (2x gains, shoulder differential, shoulder
  motors on the hips, elbow motors on the knees, 20 rad/s torque-speed
  line) plus finger motors, joint limits covering the launch reference and
  the flight pose, air on."""
  spec = mujoco.MjSpec.from_file(os.path.join(T.TASK_DIR, 'task.xml'))
  T.add_shoulder_differential(spec)
  add_fingers(spec)
  m0 = spec.compile()
  d0 = mujoco.MjData(m0)
  d0.qpos[3] = 1.0
  for name, v in flight_pose().items():
    d0.qpos[m0.jnt_qposadr[mujoco.mj_name2id(m0, mujoco.mjtObj.mjOBJ_JOINT, name)]] = v
  mujoco.mj_kinematics(m0, d0)
  add_membrane(spec, m0, d0, fluidcoef, membrane_density)
  add_body_air(spec)
  for name in FINGER_JOINTS:
    a = spec.add_actuator()
    a.name = name
    a.target = name
    a.trntype = mujoco.mjtTrn.mjTRN_JOINT
    a.ctrllimited = True
    a.ctrlrange = [-1, 1]
  m = spec.compile()
  m.opt.timestep = T.SIM_DT
  m.opt.density = air_density
  m.opt.viscosity = AIR_VISCOSITY
  aid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
  # motors: the model's arm gains on the legs too, x torque, finger motors
  arm = {n: m.actuator_gainprm[aid(n), 0] for n in LIMB_JOINTS[:6]}
  for n in LIMB_JOINTS:
    base = arm[n.replace('leg', 'arm').replace('hip', 'shoulder').replace('knee', 'elbow')]
    m.actuator_gainprm[aid(n), 0] = base * torque
  for n in FINGER_JOINTS:
    m.actuator_gainprm[aid(n), 0] = FINGER_STALL
  for n in LIMB_JOINTS + FINGER_JOINTS:
    i = aid(n)
    g = m.actuator_gainprm[i, 0]
    m.actuator_biastype[i] = mujoco.mjtBias.mjBIAS_AFFINE
    m.actuator_biasprm[i, :3] = [0, 0, -g / no_load_speed]
    m.actuator_forcelimited[i] = 1
    m.actuator_forcerange[i] = [-g, g]
  # joint limits: launch reference range and flight pose, plus margin
  ref = T.reference()
  names = [str(n) for n in ref['joint_names']]
  pose = flight_pose()
  for n in LIMB_JOINTS + FINGER_JOINTS:
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    vals = [pose.get(n, 0.0)]
    if n in names:
      vals += list(ref['qpos'][:, 7 + names.index(n)])
    if n in FINGER_JOINTS:
      vals += [0.0]                                  # folded along the forearm
    m.jnt_limited[j] = 1
    m.jnt_range[j] = [min(vals) - joint_margin, max(vals) + joint_margin]
  for name in ['FL', 'FR', 'HL', 'HR']:
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
    m.geom_solimp[g, :3] = T.FOOT_SOLIMP
    m.geom_solref[g, :2] = T.FOOT_SOLREF
  return m


def set_pose(m, d, pose):
  for name, v in pose.items():
    d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]] = v


def wind_tunnel(m, speeds=(10, 15, 20), alphas=(0, 4, 8, 12, 16, 20, 30)):
  """Total air force on the robot in the flight pose, trunk pitched nose-up
  by alpha into a head wind (flight direction -x), gravity off."""
  d = mujoco.MjData(m)
  rows = []
  for v in speeds:
    for a in alphas:
      mujoco.mj_resetData(m, d)
      th = np.radians(a)
      d.qpos[3:7] = [np.cos(th / 2), 0, np.sin(th / 2), 0]   # nose (-x) up
      set_pose(m, d, flight_pose())
      m.opt.wind[:] = [v, 0, 0]                    # air moving +x = flying -x
      mujoco.mj_forward(m, d)
      f = d.qfrc_fluid[:3]
      rows.append((v, a, f[2], f[0]))      # lift up, drag along the air
  m.opt.wind[:] = 0
  return rows


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--render', default=None)
  args = p.parse_args()
  m = build()
  area = sum(np.pi * m.geom_size[g, 0] * m.geom_size[g, 1] for g in range(m.ngeom)
             if m.geom(g).name.startswith(('left_membrane', 'right_membrane')))
  weight = m.body_subtreemass[1] * 9.81
  print(f'mass {m.body_subtreemass[1]:.2f} kg, membrane area {area:.2f} m^2, '
        f'nq {m.nq}, nu {m.nu}')
  print('speed alpha  lift N  drag N  L/D  lift/weight')
  for v, a, lift, drag in wind_tunnel(m):
    print(f'{v:5.0f} {a:5.0f} {lift:7.0f} {drag:7.1f} {lift / max(drag, 1e-9):5.1f} {lift / weight:6.2f}')
  if args.render:
    from PIL import Image
    d = mujoco.MjData(m)
    d.qpos[3] = 1.0
    d.qpos[2] = 0.3
    set_pose(m, d, flight_pose())
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 480, 640)
    imgs = []
    for az, el in ((150, -20), (90, -89), (180, -5)):
      cam = mujoco.MjvCamera()
      cam.lookat[:] = d.subtree_com[1]
      cam.distance, cam.azimuth, cam.elevation = 5.5, az, el
      r.update_scene(d, cam)
      imgs.append(r.render())
    Image.fromarray(np.concatenate(imgs, axis=1)).save(args.render)
    print('wrote', args.render)


if __name__ == '__main__':
  main()
