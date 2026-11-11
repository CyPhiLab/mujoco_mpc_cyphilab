#!/usr/bin/env python3
"""The paddle crab model, with the same physics as the MJPC simulation.

MJPC builds robot_xdomain_modified.xml at build time by running
patch_paddle_crab_robot.py on robot_xdomain.xml (mjpc/tasks/CMakeLists.txt).
load_model() runs the same patch, so the physics here is the physics the
task sees: implicitfast at 1 ms, zero gravity, water density 1000 on
MuJoCo's ellipsoid fluid model, contacts off, kp 50 servos limited to
+-3.1 N m.

The patched servos are dyntype="user"; paddle_crab_swimming.cc supplies
their dynamics through the global mjcb_act_dyn callback, a first-order lag
da/dt = (ctrl - a) / max(tau, timestep). MuJoCo's built-in "filter" dynamics
is the same law, so the servos are switched to it here. That keeps every
rollout free of Python callbacks (and thread-safe in mujoco.rollout).
"""
import os
import subprocess
import sys
import tempfile

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TASK_DIR = os.path.dirname(HERE)
SOURCE_XML = os.path.join(TASK_DIR, 'robot_xdomain.xml')
PATCH = os.path.join(TASK_DIR, 'patch_paddle_crab_robot.py')

# Two robots:
#   v1  robot_xdomain.xml through the MJPC build patch (flat plate, kp 50
#       position servos behind the task's 0.08 s setpoint filter, joint
#       damping 1 as the XW430 speed limit). MuJoCo 3.2.3, what MJPC builds.
#   v2  robot_v2/Robot.xml, the collaborators' updated robot (chassis mesh,
#       foam, foot meshes, lighter servo housings) with MuJoCo dcmotor
#       actuators (XW430 at 12 V: 3.1 N m stall, 3.25 rad/s no load, PID
#       position control inside the motor). Needs MuJoCo >= 4. Loaded with:
#       joint damping 0 (the dcmotor already models the back-EMF; the file's
#       damping 1 would count it twice and halve the servo speed), the task's
#       command ranges (as v1), no floor, contacts off, no setpoint filter.
# The robot comes from set_robot() / the CRAB_ROBOT environment variable
# (default v2); every step npz records the robot it was made for.
ROBOTS = ('v1', 'v2')
V2_DIR = os.path.join(TASK_DIR, 'robot_v2')
V2_XML = os.path.join(V2_DIR, 'Robot.xml')
_ROBOT = os.environ.get('CRAB_ROBOT', 'v2')


def set_robot(robot):
  global _ROBOT
  assert robot in ROBOTS, robot
  _ROBOT = robot


def robot():
  return _ROBOT


def step_robot(z):
  """The robot a step npz was made for (files older than the field: v1)."""
  return str(z['robot']) if 'robot' in z else 'v1'

SIM_DT = 0.001      # the MJPC simulation timestep (patch_paddle_crab_robot.py)
PLAN_DT = 0.02      # the MJPC planner's agent_timestep (task.xml)
TAU = 0.08          # residual_ControlSmoothingTau default (task.xml)
KP = 50.0           # servo gain (patch_paddle_crab_robot.py)
FORCE_LIMIT = 3.1   # servo force range (patch_paddle_crab_robot.py)

# Swim-forward is world -Y (task.xml residual_ForwardAxis); in the body frame
# at the start pose that is the body's -y axis.
FORWARD = np.array([0.0, -1.0, 0.0])
LATERAL = np.cross([0.0, 0.0, 1.0], FORWARD)   # +X, the task's Lateral axis

# Joints in actuator order: R1 R2 R3 L1 L2 L3, (proximal, mid, distal) each.
JOINTS = ['psfr', 'msfr', 'dsfr', 'pscr', 'mscr', 'dscr',
          'psbr', 'msbr', 'dsbr', 'psfl', 'msfl', 'dsfl',
          'pscl', 'mscl', 'dscl', 'psbl', 'msbl', 'dsbl']
ACTUATORS = ['R1_1', 'R1_2', 'R1_3', 'R2_1', 'R2_2', 'R2_3',
             'R3_1', 'R3_2', 'R3_3', 'L1_1', 'L1_2', 'L1_3',
             'L2_1', 'L2_2', 'L2_3', 'L3_1', 'L3_2', 'L3_3']
NU = len(ACTUATORS)

# Left actuator i mirrors right actuator MIRROR_INDEX[i] with sign MIRROR_SIGN
# (paddle_crab_swimming.cc MirrorSign): mid and front/centre distal axes are
# already mirror images, proximal and rear-paddle axes are not.
MIRROR_SIGN = np.array([-1, 1, 1, -1, 1, 1, -1, 1, -1] * 2, dtype=float)
MIRROR_INDEX = np.r_[np.arange(9, 18), np.arange(0, 9)]


def patched_xml():
  """Returns the patched robot XML text, exactly as the MJPC build makes it."""
  with tempfile.TemporaryDirectory() as tmp:
    out = os.path.join(tmp, 'robot.xml')
    subprocess.run([sys.executable, PATCH, SOURCE_XML, out], check=True,
                   stdout=subprocess.DEVNULL)
    with open(out) as f:
      return f.read()


def _compile(xml):
  # Written next to robot_xdomain.xml so its relative includes and the mesh
  # directory resolve the way they do in the build directory.
  fd, path = tempfile.mkstemp(suffix='.xml', prefix='.crab_', dir=TASK_DIR)
  try:
    with os.fdopen(fd, 'w') as f:
      f.write(xml)
    return mujoco.MjModel.from_xml_path(path)
  finally:
    os.remove(path)


# Design variants for "what limits speed" studies (all 1.0 is the robot):
#   torque         every servo's torque limit, x
#   paddle_torque  the rear (paddle) legs' servo torque limits, x
#   kp             every servo's position gain, x
#   body_drag      drag coefficients (not added mass) of the body's geoms:
#                  plate, servo housings, cylinder, x
#   paddle_area    paddle mesh area, x (scaled uniformly by its square root)
#   damping        every joint's damping, x (1 N m s/rad on every joint
#                  caps a 3.1 N m servo at 3.1 rad/s: the servo speed limit)
#   paddle_damping the rear (paddle) legs' joint damping, x
#   paddle_mass    mass and inertia of the paddle legs' mid and distal
#                  segments (the outer two servos and the blade), x
DESIGN_DEFAULTS = dict(torque=1.0, paddle_torque=1.0, kp=1.0, body_drag=1.0,
                       paddle_area=1.0, damping=1.0, paddle_damping=1.0,
                       paddle_mass=1.0)
PADDLE_ACTUATORS = [i for i, a in enumerate(ACTUATORS) if a[:2] in ('R3', 'L3')]


def load_model(dt=SIM_DT, tau=TAU, ground=True, ctrl_limit=None, design=None,
               robot=None):
  """The patched crab with filter servos (the MJPC callback's law), or the
  v2 robot (see ROBOTS).

  ground=False drops the (visual only, contacts are off) heightfield floor.
  ctrl_limit replaces every servo's command range with +-ctrl_limit.
  design: a dict of DESIGN_DEFAULTS overrides.
  robot: 'v1' or 'v2' (default: robot()).
  """
  design = dict(DESIGN_DEFAULTS, **(design or {}))
  if (robot or _ROBOT) == 'v2':
    return load_v2(dt, ground, ctrl_limit, design)
  xml = patched_xml()
  if not ground:
    # hfield_ground.xml also sets the mesh directory.
    xml = xml.replace('<include file="hfield_ground.xml" />',
                      '<compiler meshdir="asset" />')
  if design['paddle_area'] != 1.0:
    scale = 0.001 * np.sqrt(design['paddle_area'])
    xml = xml.replace('scale=".001 .001 .001"', f'scale="{scale} {scale} {scale}"')
  m = _compile(xml)
  set_timestep(m, dt, tau)
  if ctrl_limit is not None:
    m.actuator_ctrlrange[:] = [-ctrl_limit, ctrl_limit]
  m.actuator_forcerange[:] *= design['torque']
  m.actuator_forcerange[PADDLE_ACTUATORS] *= design['paddle_torque']
  m.actuator_gainprm[:, 0] *= design['kp']
  m.actuator_biasprm[:, 1] *= design['kp']
  m.dof_damping[6:] *= design['damping']
  for i in PADDLE_ACTUATORS:
    m.dof_damping[m.jnt_dofadr[m.actuator_trnid[i, 0]]] *= design[
        'paddle_damping']
  if design['paddle_mass'] != 1.0:
    for b in range(m.nbody):
      name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or ''
      if name.startswith(('mid segment back', 'distal segment back')):
        m.body_mass[b] *= design['paddle_mass']
        m.body_inertia[b] *= design['paddle_mass']
  if design['body_drag'] != 1.0:
    for g in range(m.ngeom):
      name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
      if name and 'segment' not in name and m.geom_fluid[g, 0] > 0:
        m.geom_fluid[g, 1:6] *= design['body_drag']
  return m


# The task's command ranges (patch_paddle_crab_robot.py), used for both robots.
TASK_RANGES = {}
for _names, _r in ((('R1_1', 'R2_1', 'R3_1', 'L1_1', 'L2_1', 'L3_1'), 1.20),
                   (('R1_2', 'R2_2', 'R3_2', 'L1_2', 'L2_2', 'L3_2'), 1.57),
                   (('R1_3', 'R2_3', 'L1_3', 'L2_3'), 1.05),
                   (('R3_3', 'L3_3'), 1.57)):
  for _n in _names:
    TASK_RANGES[_n] = _r


def _mujoco_major():
  return int(mujoco.__version__.split('.')[0])


def load_v2(dt=SIM_DT, ground=True, ctrl_limit=None, design=None):
  """robot_v2/Robot.xml for this pipeline (see ROBOTS)."""
  if _mujoco_major() < 4:
    raise RuntimeError('robot v2 (dcmotor actuators) needs MuJoCo >= 4; this '
                       f'is {mujoco.__version__}. Use the MuJoCo 4 env, or '
                       'CRAB_ROBOT=v1 for the old robot.')
  design = dict(DESIGN_DEFAULTS, **(design or {}))
  for key in ('kp', 'damping', 'paddle_damping'):
    if design[key] != 1.0:
      raise ValueError(f'design {key} does not apply to robot v2 (the '
                       'dcmotor sets the servo gain and speed limit)')
  with open(V2_XML) as f:
    xml = f.read()
  # MuJoCo 4.0 spells the dcmotor position input "pos"; the file was written
  # for a newer MuJoCo (mjlab) that spells it "position".
  try:
    mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body><joint name="j"/><geom size=".1"/></body>'
        '</worldbody><actuator><dcmotor joint="j" nominal="12 3.1 3.2" '
        'input="position" controller="1 0 0 0 0 12"/></actuator></mujoco>')
  except ValueError:
    xml = xml.replace('input="position"', 'input="pos"')
  if not ground:
    xml = xml.replace('<include file="hfield_ground.xml" />', '')
  if design['paddle_area'] != 1.0:
    scale = 0.001 * np.sqrt(design['paddle_area'])
    xml = xml.replace(
        '<mesh name="paddle" file="Swimming_Paddle.stl" scale=".001 .001 .001"',
        f'<mesh name="paddle" file="Swimming_Paddle.stl" '
        f'scale="{scale} {scale} {scale}"')
  # Actuator force sensors: the dcmotor force is not a function of the state
  # alone, so the optimizer reads it from rollout sensor data.
  sensors = ''.join(f'<actuatorfrc name="force_{a}" actuator="{a}"/>'
                    for a in ACTUATORS)
  xml = xml.replace('</mujoco>', f'<sensor>{sensors}</sensor></mujoco>')
  fd, path = tempfile.mkstemp(suffix='.xml', prefix='.crab_', dir=V2_DIR)
  try:
    with os.fdopen(fd, 'w') as f:
      f.write(xml)
    m = mujoco.MjModel.from_xml_path(path)
  finally:
    os.remove(path)
  m.opt.timestep = dt
  m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
  m.dof_damping[6:] = 0.0
  for i, name in enumerate(ACTUATORS):
    r = TASK_RANGES[name] if ctrl_limit is None else ctrl_limit
    m.actuator_ctrlrange[i] = [-r, r]
  m.actuator_forcerange[:] *= design['torque']
  m.actuator_forcerange[PADDLE_ACTUATORS] *= design['paddle_torque']
  if design['paddle_mass'] != 1.0:
    for b in range(m.nbody):
      name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or ''
      if name.startswith(('mid segment back', 'distal segment back')):
        m.body_mass[b] *= design['paddle_mass']
        m.body_inertia[b] *= design['paddle_mass']
  if design['body_drag'] != 1.0:
    for g in range(m.ngeom):
      name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
      if name and 'segment' not in name and m.geom_fluid[g, 0] > 0:
        m.geom_fluid[g, 1:6] *= design['body_drag']
  return m


def _v2_nominal():
  """(stall torque N m, no-load speed rad/s) of the v2 dcmotor (its nominal
  attribute: voltage, stall torque, no-load speed)."""
  import re
  with open(V2_XML) as f:
    v = [float(x) for x in re.search(r'nominal="([^"]+)"', f.read()).group(1).split()]
  return v[1], v[2]


V2_STALL, V2_NOLOAD = _v2_nominal()


def motor_fraction(force, qvel):
  """v2: the fraction of the supply voltage each dcmotor needs,
  u = torque / stall + speed / no-load speed (1 at the voltage limit; the
  quantity v1's damping-inclusive servo torque / 3.1 N m measured)."""
  return force / V2_STALL + qvel / V2_NOLOAD


def is_v2(m):
  """A v2 model (dcmotor actuators, force sensors)."""
  return m.nsensordata == NU and m.actuator_dyntype[0] != mujoco.mjtDyn.mjDYN_FILTER


def set_timestep(m, dt, tau=TAU):
  m.opt.timestep = dt
  m.actuator_dyntype[:] = mujoco.mjtDyn.mjDYN_FILTER
  # The callback clamps tau to the timestep; filter would ring below it too.
  m.actuator_dynprm[:, 0] = max(tau, dt)


def check_layout(m):
  """The index assumptions below hold for this model."""
  assert m.nu == NU, m.nu
  assert m.nq == 7 + NU and m.nv == 6 + NU
  for i, (a, j) in enumerate(zip(ACTUATORS, JOINTS)):
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) == i, a
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
    assert m.actuator_trnid[i, 0] == jid, a
    assert m.jnt_qposadr[jid] == 7 + i and m.jnt_dofadr[jid] == 6 + i, j


def ctrl_range(m):
  return m.actuator_ctrlrange.copy()


# ----- State layout of mujoco.rollout (mjSTATE_FULLPHYSICS) ----- #

STATE_SPEC = mujoco.mjtState.mjSTATE_FULLPHYSICS


class StateLayout:
  """Slices of a full-physics state vector: time, qpos, qvel, act, ..."""

  def __init__(self, m):
    self.size = mujoco.mj_stateSize(m, STATE_SPEC)
    self.time = 0
    self.qpos = slice(1, 1 + m.nq)
    self.qvel = slice(1 + m.nq, 1 + m.nq + m.nv)
    self.act = slice(1 + m.nq + m.nv, 1 + m.nq + m.nv + m.na)


def initial_state(m, qpos=None, qvel=None, act=None):
  d = mujoco.MjData(m)
  if qpos is not None:
    d.qpos[:] = qpos
  if qvel is not None:
    d.qvel[:] = qvel
  if act is not None:
    d.act[:] = act
  state = np.empty(mujoco.mj_stateSize(m, STATE_SPEC))
  mujoco.mj_getState(m, d, state, STATE_SPEC)
  return state


# ----- Kinematics helpers on trajectories ----- #

def quat_to_mat(q):
  """(..., 4) wxyz quaternions -> (..., 3, 3) rotation matrices."""
  w, x, y, z = np.moveaxis(q, -1, 0)
  return np.stack([
      np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
                2 * (x * z + y * w)], -1),
      np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
                2 * (y * z - x * w)], -1),
      np.stack([2 * (x * z - y * w), 2 * (y * z + x * w),
                1 - 2 * (x * x + y * y)], -1)], -2)


def yaw_of(R):
  """Heading of the body x-axis in the world xy-plane."""
  return np.arctan2(R[..., 1, 0], R[..., 0, 0])


def servo_command_force(m, q_joint, act):
  """kp * (act - q) before the force limit: the patched servo law."""
  return m.actuator_gainprm[:, 0] * (act - q_joint)


def actuator_force(m, q_joint, act):
  """The servo force: kp * (act - q), clipped to the force range."""
  limit = m.actuator_forcerange[:, 1]
  return np.clip(servo_command_force(m, q_joint, act), -limit, limit)


def total_mass(m):
  plate = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'plate')
  return float(m.body_subtreemass[plate])
