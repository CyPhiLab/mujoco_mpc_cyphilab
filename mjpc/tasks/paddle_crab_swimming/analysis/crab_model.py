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


def load_model(dt=SIM_DT, tau=TAU, ground=True, ctrl_limit=None, design=None):
  """The patched crab with filter servos (the MJPC callback's law).

  ground=False drops the (visual only, contacts are off) heightfield floor.
  ctrl_limit replaces every servo's command range with +-ctrl_limit.
  design: a dict of DESIGN_DEFAULTS overrides.
  """
  design = dict(DESIGN_DEFAULTS, **(design or {}))
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


def set_timestep(m, dt, tau=TAU):
  m.opt.timestep = dt
  m.actuator_dyntype[:] = mujoco.mjtDyn.mjDYN_FILTER
  # The callback clamps tau to the timestep; filter would ring below it too.
  m.actuator_dynprm[:, 0] = max(tau, dt)


def check_layout(m):
  """The index assumptions below hold for this model."""
  assert m.nu == NU and m.na == NU, (m.nu, m.na)
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
