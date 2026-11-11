# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Pterosaur Launch mode on MJWarp (port of mjpc/tasks/pterosaur/pterosaur.cc).

MJPC's Launch mode is stateful: `TransitionLocked` records the launch start
time, torso orientation, head position and ground height when the mode is
entered, then rewrites the cost weights as the launch proceeds through its
phases (preload, rise, pivot, jump, flight, land). For offline trajectory
optimization the launch starts at the first knot, so this state is frozen in
`bind()` and the weight schedule becomes a table indexed by phase, evaluated
at every stage time. That is the intended semantics of the schedule; in MPC
it is applied to the whole horizon at the current time instead.

Differences from the C++ residual:
* `Gait` residuals are left at zero. Launch mode sets their weight to 0, so
  the cost is unchanged.
* The ground height is ray-cast once in `bind()` (flat ground), as MJPC does
  at the start of Launch.

The residual layout, norms and default weights come from task.xml, so editing
the XML user sensors stays in sync with MJPC. The per-phase weight table is
exposed as `LaunchSchedule.phase_weights` for tuning without recompiling.
"""

import dataclasses
import math
from typing import Optional

import mujoco
import numpy as np
import warp as wp

from mujoco_mpc.mjwarp import model as model_lib
from mujoco_mpc.mjwarp import state as state_lib
from mujoco_mpc.mjwarp.tasks import base

XML = "pterosaur/task.xml"

# pterosaur.h constants
HEIGHT_QUADRUPED = 0.8
FOOT_RADIUS = 0.02
LAUNCH_PRELOAD_TIME = 0.5
LAUNCH_RISE_TIME = 0.25
LAUNCH_PIVOT_TIME = 0.5
LAUNCH_PUSH_TIME = 0.18
LAUNCH_LAND_TIME = 0.25
LAUNCH_PIVOT_SPEED = 7.0
LAUNCH_SPEED = 12.96
LAUNCH_ANGLE = 35.0 * math.pi / 180.0

PHASES = ("preload", "rise", "pivot", "jump", "flight", "land")


def default_phase_weights(spec: model_lib.CostSpec) -> np.ndarray:
  """Weight table (nphase, nterm) reproducing Pterosaur::TransitionLocked."""
  base_w = spec.weights.copy()
  launch = {
      "Upright": 0.2, "Height": 1.0, "Position": 0.4, "LaunchVelocity": 1.0,
      "Gait": 0.0, "Balance": 0.6, "Effort": 0.005, "Posture": 0.1,
  }
  for name, value in launch.items():
    base_w[spec.index(name)] = value
  per_phase = {
      "preload": {"Balance": 0.0, "GroundContact": 3.0, "LaunchVelocity": 0.0,
                  "Posture": 0.055},
      "rise": {"Balance": 1.0, "GroundContact": 3.6, "LaunchVelocity": 0.0,
               "Posture": 0.05},
      "pivot": {"Balance": 0.0, "GroundContact": 0.8, "LaunchVelocity": 0.0,
                "Posture": 3.0},
  }
  effort_only = {n: 0.0 for n in ("Upright", "Height", "Position",
                                  "LaunchVelocity", "Balance",
                                  "GroundContact", "Posture")}
  for p in ("jump", "flight", "land"):
    per_phase[p] = effort_only
  table = np.tile(base_w, (len(PHASES), 1))
  for i, p in enumerate(PHASES):
    for name, value in per_phase[p].items():
      table[i, spec.index(name)] = value
  return table


@dataclasses.dataclass
class LaunchSchedule:
  """Phase durations (s) and the per-phase cost weights."""

  preload: float = LAUNCH_PRELOAD_TIME
  rise: float = LAUNCH_RISE_TIME
  pivot: float = LAUNCH_PIVOT_TIME
  jump: float = LAUNCH_PUSH_TIME
  flight: Optional[float] = None  # None -> ballistic time from launch speed
  land: float = LAUNCH_LAND_TIME
  phase_weights: Optional[np.ndarray] = None  # (6, nterm)

  def durations(self, gravity: float) -> np.ndarray:
    flight = self.flight
    if flight is None:
      flight = 2 * LAUNCH_SPEED * math.sin(LAUNCH_ANGLE) / gravity
    return np.array(
        [self.preload, self.rise, self.pivot, self.jump, flight, self.land]
    )

  def total(self, gravity: float) -> float:
    return float(self.durations(gravity).sum())


@wp.struct
class LaunchParams:
  torso: int
  head: int
  foot_fl: int
  foot_hl: int
  foot_fr: int
  foot_hr: int
  adr_compos: int
  adr_comvel: int
  adr_angmom: int
  adr_touch_fr: int
  adr_touch_fl: int
  adr_touch_rr: int
  adr_touch_rl: int
  # residual offsets
  o_upright: int
  o_height: int
  o_position: int
  o_launchvel: int
  o_gait: int
  o_balance: int
  o_effort: int
  o_posture: int
  o_orientation: int
  o_angmom: int
  o_contact: int
  # frozen launch state
  start_time: float
  position: wp.vec3
  orientation: wp.quat
  ground: float
  heading: float
  # phase timing
  t_preload: float
  t_rise: float
  t_pivot: float
  t_jump: float
  t_flight: float
  t_land: float


@wp.func
def _phase(launch_time: float, p: LaunchParams) -> int:
  t = launch_time
  if t < p.t_preload:
    return 0
  t -= p.t_preload
  if t < p.t_rise:
    return 1
  t -= p.t_rise
  if t < p.t_pivot:
    return 2
  t -= p.t_pivot
  if t < p.t_jump:
    return 3
  t -= p.t_jump
  if t < p.t_flight:
    return 4
  return 5


@wp.func
def _launch_height(time: float, p: LaunchParams) -> float:
  start_h = p.position[2]
  crouch_h = p.ground + 0.5
  rise_h = p.ground + HEIGHT_QUADRUPED
  pivot_h = p.ground + 1.1
  jump_h = p.ground + 1.25
  landing_h = p.ground + 0.85
  if time < 0.0:
    return start_h
  if time < p.t_preload:
    a = time / p.t_preload
    a = a * a * (3.0 - 2.0 * a)
    return start_h + (crouch_h - start_h) * a
  time -= p.t_preload
  if time < p.t_rise:
    a = time / p.t_rise
    return crouch_h + (rise_h - crouch_h) * a * a
  time -= p.t_rise
  if time < p.t_pivot:
    a = time / p.t_pivot
    return rise_h + (pivot_h - rise_h) * a * a
  time -= p.t_pivot
  if time < p.t_jump:
    return pivot_h + (jump_h - pivot_h) * (time / p.t_jump)
  time -= p.t_jump
  if time < p.t_flight:
    mid = p.t_flight * 0.5
    if time <= mid:
      return jump_h + (2.8 - jump_h) * (time / mid)
    return 2.8 + (landing_h - 2.8) * ((time - mid) / mid)
  time -= p.t_flight
  if time < p.t_land:
    return landing_h + (p.ground + HEIGHT_QUADRUPED - landing_h) * (time / p.t_land)
  return p.ground + HEIGHT_QUADRUPED


@wp.func
def _touch_penalty(touch: float) -> float:
  return wp.max(0.0, 0.05 - touch)


@wp.func
def _normalize3(v: wp.vec3) -> wp.vec3:
  n = wp.length(v)
  if n < 1.0e-15:
    return v
  return v / n


@wp.kernel
def launch_residual_kernel(
    p: LaunchParams,
    stage_time: wp.array(dtype=float),
    geom_xpos: wp.array2d(dtype=wp.vec3),
    xmat: wp.array2d(dtype=wp.mat33),
    xquat: wp.array2d(dtype=wp.quat),
    xipos: wp.array2d(dtype=wp.vec3),
    site_xpos: wp.array2d(dtype=wp.vec3),
    actuator_force: wp.array2d(dtype=float),
    qpos: wp.array2d(dtype=float),
    qvel: wp.array2d(dtype=float),
    sensordata: wp.array2d(dtype=float),
    key_home: wp.array(dtype=float),
    key_crouch: wp.array(dtype=float),
    key_pivot: wp.array(dtype=float),
    key_pivot_qvel: wp.array(dtype=float),
    phase_weights: wp.array2d(dtype=float),
    nu: int,
    # outputs
    residual: wp.array2d(dtype=float),
    weight: wp.array2d(dtype=float),
):
  w = wp.tid()
  launch_time = stage_time[w] - p.start_time
  phase = _phase(launch_time, p)
  rise_end = p.t_preload + p.t_rise
  pivot_end = rise_end + p.t_pivot

  for k in range(phase_weights.shape[1]):
    weight[w, k] = phase_weights[phase, k]

  fl = geom_xpos[w, p.foot_fl]
  hl = geom_xpos[w, p.foot_hl]
  fr = geom_xpos[w, p.foot_fr]
  hr = geom_xpos[w, p.foot_hr]
  avg_foot = 0.25 * (fl + hl + fr + hr)
  R = xmat[w, p.torso]
  torso_quat = xquat[w, p.torso]
  compos = wp.vec3(
      sensordata[w, p.adr_compos],
      sensordata[w, p.adr_compos + 1],
      sensordata[w, p.adr_compos + 2],
  )
  comvel = wp.vec3(
      sensordata[w, p.adr_comvel],
      sensordata[w, p.adr_comvel + 1],
      sensordata[w, p.adr_comvel + 2],
  )
  forward = _normalize3(wp.vec3(R[0, 0], R[1, 0], R[2, 0]))
  up = _normalize3(wp.vec3(R[0, 2], R[1, 2], R[2, 2]))

  # ---------- Upright ----------
  o = p.o_upright
  if launch_time >= rise_end and launch_time < pivot_end:
    for i in range(3):
      residual[w, o + i] = 0.0
  elif launch_time >= p.t_preload and launch_time < rise_end:
    a = wp.clamp((launch_time - p.t_preload) / p.t_rise, 0.0, 1.0)
    q = wp.quat(
        (1.0 - a) * key_crouch[3] + a * key_pivot[3],
        (1.0 - a) * key_crouch[4] + a * key_pivot[4],
        (1.0 - a) * key_crouch[5] + a * key_pivot[5],
        (1.0 - a) * key_crouch[6] + a * key_pivot[6],
    )
    q = state_lib._normalize_quat(q)
    v = state_lib.quat_sub(torso_quat, q)
    for i in range(3):
      residual[w, o + i] = v[i]
  else:
    v = state_lib.quat_sub(torso_quat, p.orientation)
    for i in range(3):
      residual[w, o + i] = v[i]

  # ---------- Height ----------
  torso_pos = xipos[w, p.torso]
  residual[w, p.o_height] = torso_pos[2] - _launch_height(launch_time, p)

  # ---------- Position ----------
  head = site_xpos[w, p.head]
  horizontal_speed = LAUNCH_SPEED * wp.cos(LAUNCH_ANGLE)
  dist = float(0.0)
  if launch_time < p.t_preload:
    a = launch_time / p.t_preload
    a = a * a * (3.0 - 2.0 * a)
    dist = -0.05 + 0.10 * a
  elif launch_time < rise_end:
    dist = 0.05
  elif launch_time < pivot_end:
    dist = 0.5 * (launch_time - rise_end) / p.t_pivot
  else:
    dist = (0.05 + 1.5 * horizontal_speed * p.t_pivot) + horizontal_speed * (
        launch_time - pivot_end
    )
  o = p.o_position
  residual[w, o] = head[0] - (p.position[0] + forward[0] * dist)
  residual[w, o + 1] = head[1] - (p.position[1] + forward[1] * dist)
  residual[w, o + 2] = 0.0

  # ---------- Launch velocity ----------
  desired_speed = float(0.0)
  desired_dir = wp.vec3(0.0)
  if launch_time < pivot_end:
    desired_dir = forward + up
    desired_speed = LAUNCH_PIVOT_SPEED
  elif launch_time < pivot_end + p.t_jump + p.t_flight:
    desired_dir = forward + up
    desired_speed = LAUNCH_SPEED
  if desired_speed > 0.0:
    desired_dir = _normalize3(desired_dir)
  residual[w, p.o_launchvel] = wp.dot(comvel, desired_dir) - desired_speed

  # ---------- Gait (weight 0 in Launch) ----------
  for i in range(4):
    residual[w, p.o_gait + i] = 0.0

  # ---------- Balance ----------
  fall_time = wp.sqrt(2.0 * HEIGHT_QUADRUPED / 9.81)
  capture = compos + comvel * fall_time
  support = avg_foot
  if launch_time >= rise_end and launch_time < pivot_end:
    support = 0.5 * (fl + fr)
  residual[w, p.o_balance] = capture[0] - support[0]
  residual[w, p.o_balance + 1] = capture[1] - support[1]

  # ---------- Effort ----------
  o = p.o_effort
  for i in range(nu):
    s = float(2.0e-2)
    if launch_time < pivot_end:
      if i == 0 or i == 3 or i == 6 or i == 9:
        s *= 3.0
      if launch_time >= p.t_preload:  # rise or pivot
        if i == 6 or i == 9:
          s *= 1.5
        if i == 1 or i == 2 or i == 4 or i == 5 or i == 8 or i == 11:
          s *= 0.5
        if i == 7 or i == 10:
          s *= 0.35
    residual[w, o + i] = s * actuator_force[w, i]

  # ---------- Posture ----------
  o = p.o_posture
  for i in range(3):
    residual[w, o + i] = 0.0
  if launch_time < rise_end:
    a = float(0.0)
    if launch_time < p.t_preload:
      a = wp.clamp(launch_time / p.t_preload, 0.0, 1.0)
    else:
      a = wp.clamp((launch_time - p.t_preload) / p.t_rise, 0.0, 1.0)
    for i in range(nu):
      target = float(0.0)
      if launch_time < p.t_preload:
        target = (1.0 - a) * key_home[7 + i] + a * key_crouch[7 + i]
      else:
        target = (1.0 - a) * key_crouch[7 + i] + a * key_home[7 + i]
      r = qpos[w, 7 + i] - target
      if launch_time < p.t_preload:
        if i < 6:
          r = 0.0
        else:
          r *= 1.0 / wp.sqrt(6.0)
      residual[w, o + 3 + i] = r
  elif launch_time < pivot_end:
    a = wp.clamp((launch_time - rise_end) / p.t_pivot, 0.0, 1.0)
    for i in range(nu):
      target = (1.0 - a) * key_home[7 + i] + a * key_pivot[7 + i]
      r = qpos[w, 7 + i] - target
      if i == 1 or i == 4:
        r = 0.25 * (qvel[w, 6 + i] - key_pivot_qvel[6 + i])
      residual[w, o + 3 + i] = r
    tq = state_lib._normalize_quat(
        wp.quat(key_pivot[3], key_pivot[4], key_pivot[5], key_pivot[6])
    )
    v = state_lib.quat_sub(torso_quat, tq)
    for i in range(3):
      residual[w, o + i] = v[i]
    for i in range(3 + nu):
      residual[w, o + i] = 0.5 * residual[w, o + i]
  else:
    for i in range(nu):
      residual[w, o + 3 + i] = 0.0
  for i in range(nu):
    gain = float(1.0)
    if i % 3 == 0:
      gain = 2.0
    residual[w, o + 3 + i] = gain * residual[w, o + 3 + i]

  # ---------- Orientation (yaw) ----------
  o = p.o_orientation
  heading = _normalize3(wp.vec3(R[0, 0], R[1, 0], R[2, 0]))
  residual[w, o] = heading[0] - wp.cos(p.heading)
  residual[w, o + 1] = heading[1] - wp.sin(p.heading)
  residual[w, o + 2] = heading[2]

  # ---------- Angular momentum ----------
  for i in range(3):
    residual[w, p.o_angmom + i] = sensordata[w, p.adr_angmom + i]

  # ---------- Ground contact ----------
  o = p.o_contact
  t_fr = _touch_penalty(sensordata[w, p.adr_touch_fr])
  t_fl = _touch_penalty(sensordata[w, p.adr_touch_fl])
  t_rr = _touch_penalty(sensordata[w, p.adr_touch_rr])
  t_rl = _touch_penalty(sensordata[w, p.adr_touch_rl])
  c0 = float(0.0)
  c1 = float(0.0)
  c2 = float(0.0)
  c3 = float(0.0)
  if launch_time < p.t_preload:
    c0 = t_fr
    c1 = t_fl
  elif launch_time < rise_end:
    c0 = t_fr
    c1 = t_fl
    c2 = t_rr
    c3 = t_rl
  elif launch_time < pivot_end:
    c0 = t_fr
    c1 = t_fl
  elif launch_time < pivot_end + p.t_jump:
    if launch_time - pivot_end <= 0.5 * p.t_jump:
      c0 = t_fr
      c1 = t_fl
  elif launch_time < pivot_end + p.t_jump + p.t_flight:
    c0 = 0.0
  else:
    c0 = t_fr
    c1 = t_fl
    if launch_time - pivot_end - p.t_jump - p.t_flight > 0.5 * p.t_land:
      c2 = t_rr
      c3 = t_rl
  residual[w, o] = c0
  residual[w, o + 1] = c1
  residual[w, o + 2] = c2
  residual[w, o + 3] = c3


def ground_height(mjm: mujoco.MjModel, mjd: mujoco.MjData, pos) -> float:
  """utilities.cc Ground(): ray cast down onto group-0 geoms."""
  query = np.array([pos[0], pos[1], pos[2] + 0.5])
  geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
  geomid = np.zeros(1, dtype=np.int32)
  dist = mujoco.mj_ray(mjm, mjd, query, np.array([0.0, 0.0, -1.0]), geomgroup,
                       1, -1, geomid)
  if dist < 0:
    raise RuntimeError("no group 0 geom below position")
  return float(pos[2] + 0.5 - dist)


class PterosaurLaunch(base.Task):
  """Launch-mode residual of the pterosaur task."""

  def __init__(
      self,
      mjm: mujoco.MjModel,
      schedule: Optional[LaunchSchedule] = None,
      heading: float = 0.0,
  ):
    super().__init__(mjm)
    self.schedule = schedule or LaunchSchedule()
    self.heading = heading
    spec = self.spec
    pw = self.schedule.phase_weights
    if pw is None:
      pw = default_phase_weights(spec)
    pw = np.asarray(pw, dtype=np.float32)
    if pw.shape != (len(PHASES), spec.nterm):
      raise ValueError(f"phase_weights must be {(len(PHASES), spec.nterm)}")
    self.phase_weights = wp.array(pw, dtype=float)
    self.gravity = float(np.linalg.norm(mjm.opt.gravity))
    self.key_home = wp.array(model_lib.key_qpos(mjm, "home").astype(np.float32), dtype=float)
    self.key_crouch = wp.array(model_lib.key_qpos(mjm, "crouch").astype(np.float32), dtype=float)
    self.key_pivot = wp.array(model_lib.key_qpos(mjm, "pivot").astype(np.float32), dtype=float)
    self.key_pivot_qvel = wp.array(model_lib.key_qvel(mjm, "pivot").astype(np.float32), dtype=float)
    self.params = LaunchParams()
    p = self.params
    obj = mujoco.mjtObj
    p.torso = model_lib.obj_id(mjm, obj.mjOBJ_XBODY, "body")
    p.head = model_lib.obj_id(mjm, obj.mjOBJ_SITE, "head")
    p.foot_fl = model_lib.obj_id(mjm, obj.mjOBJ_GEOM, "FL")
    p.foot_hl = model_lib.obj_id(mjm, obj.mjOBJ_GEOM, "HL")
    p.foot_fr = model_lib.obj_id(mjm, obj.mjOBJ_GEOM, "FR")
    p.foot_hr = model_lib.obj_id(mjm, obj.mjOBJ_GEOM, "HR")
    p.adr_compos = model_lib.sensor_adr(mjm, "torso_subtreecom")
    p.adr_comvel = model_lib.sensor_adr(mjm, "torso_subtreelinvel")
    p.adr_angmom = model_lib.sensor_adr(mjm, "torso_angmom")
    p.adr_touch_fr = model_lib.sensor_adr(mjm, "FR_touch")
    p.adr_touch_fl = model_lib.sensor_adr(mjm, "FL_touch")
    p.adr_touch_rr = model_lib.sensor_adr(mjm, "RR_touch")
    p.adr_touch_rl = model_lib.sensor_adr(mjm, "RL_touch")
    off = {n: int(spec.offsets[spec.index(n)]) for n in spec.names}
    p.o_upright = off["Upright"]
    p.o_height = off["Height"]
    p.o_position = off["Position"]
    p.o_launchvel = off["LaunchVelocity"]
    p.o_gait = off["Gait"]
    p.o_balance = off["Balance"]
    p.o_effort = off["Effort"]
    p.o_posture = off["Posture"]
    p.o_orientation = off["Orientation"]
    p.o_angmom = off["Angmom"]
    p.o_contact = off["GroundContact"]
    dur = self.schedule.durations(self.gravity)
    (p.t_preload, p.t_rise, p.t_pivot, p.t_jump, p.t_flight, p.t_land) = (
        float(x) for x in dur
    )
    p.heading = float(heading)
    self.bind(model_lib.key_qpos(mjm, "home"))

  @property
  def duration(self) -> float:
    return self.schedule.total(self.gravity)

  def bind(self, x0: np.ndarray, time0: float = 0.0) -> None:
    mjm = self.mjm
    d = mujoco.MjData(mjm)
    d.qpos[:] = x0[: mjm.nq]
    if len(x0) > mjm.nq:
      d.qvel[:] = x0[mjm.nq : mjm.nq + mjm.nv]
    d.time = time0
    mujoco.mj_forward(mjm, d)
    p = self.params
    p.start_time = float(time0)
    head = d.site_xpos[p.head]
    p.position = wp.vec3(*head.astype(np.float32))
    p.orientation = wp.quat(*d.xquat[p.torso].astype(np.float32))
    compos = d.sensordata[p.adr_compos : p.adr_compos + 3]
    p.ground = ground_height(mjm, d, compos)

  def residual(self, m, d, stage_time, residual, weight) -> None:
    wp.launch(
        launch_residual_kernel,
        dim=d.nworld,
        inputs=[
            self.params, stage_time, d.geom_xpos, d.xmat, d.xquat, d.xipos,
            d.site_xpos, d.actuator_force, d.qpos, d.qvel, d.sensordata,
            self.key_home, self.key_crouch, self.key_pivot,
            self.key_pivot_qvel, self.phase_weights, self.mjm.nu,
        ],
        outputs=[residual, weight],
    )
