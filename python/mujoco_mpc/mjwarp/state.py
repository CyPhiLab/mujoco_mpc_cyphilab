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
"""Batched state handling on device: set/get states and tangent-space maps.

A state is x = [qpos (nq), qvel (nv), act (na)] as in MJPC. Its tangent
dx = [dqpos (nv), dqvel (nv), dact (na)] uses the joint-aware difference of
`mj_differentiatePos` / `mj_integratePos` (quaternions for free and ball
joints), so that finite differences and feedback gains live in R^(2nv+na).
"""

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

# mjtJoint
_FREE = 0
_BALL = 1


@wp.func
def _mul_quat(u: wp.quat, v: wp.quat) -> wp.quat:
  return wp.quat(
      u[0] * v[0] - u[1] * v[1] - u[2] * v[2] - u[3] * v[3],
      u[0] * v[1] + u[1] * v[0] + u[2] * v[3] - u[3] * v[2],
      u[0] * v[2] - u[1] * v[3] + u[2] * v[0] + u[3] * v[1],
      u[0] * v[3] + u[1] * v[2] - u[2] * v[1] + u[3] * v[0],
  )


@wp.func
def _normalize_quat(q: wp.quat) -> wp.quat:
  n = wp.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
  if n < 1.0e-12:
    return wp.quat(1.0, 0.0, 0.0, 0.0)
  return wp.quat(q[0] / n, q[1] / n, q[2] / n, q[3] / n)


@wp.func
def quat_sub(qa: wp.quat, qb: wp.quat) -> wp.vec3:
  """mju_subQuat: 3D rotation vector v with qb * quat(v) = qa (MuJoCo order)."""
  qdif = _mul_quat(wp.quat(qb[0], -qb[1], -qb[2], -qb[3]), qa)
  axis = wp.vec3(qdif[1], qdif[2], qdif[3])
  s = wp.length(axis)
  if s == 0.0:
    return wp.vec3(0.0)
  angle = 2.0 * wp.atan2(s, qdif[0])
  if angle > wp.pi:
    angle -= 2.0 * wp.pi
  return axis * (angle / s)


@wp.func
def quat_integrate(q: wp.quat, v: wp.vec3) -> wp.quat:
  """mju_quatIntegrate with unit time: q * quat(v)."""
  angle = wp.length(v)
  if angle < 1.0e-12:
    return _normalize_quat(q)
  axis = v / angle
  s = wp.sin(0.5 * angle)
  dq = wp.quat(wp.cos(0.5 * angle), axis[0] * s, axis[1] * s, axis[2] * s)
  return _normalize_quat(_mul_quat(_normalize_quat(q), dq))


@wp.func
def _read_quat(x: wp.array2d(dtype=float), w: int, a: int) -> wp.quat:
  return wp.quat(x[w, a], x[w, a + 1], x[w, a + 2], x[w, a + 3])


class StateLayout:
  """Joint layout needed for tangent-space maps, uploaded once."""

  def __init__(self, mjm: mujoco.MjModel):
    self.nq, self.nv, self.na, self.nu = mjm.nq, mjm.nv, mjm.na, mjm.nu
    self.nx = mjm.nq + mjm.nv + mjm.na
    self.ndx = 2 * mjm.nv + mjm.na
    self.jnt_type = wp.array(mjm.jnt_type.astype(np.int32), dtype=int)
    self.jnt_qposadr = wp.array(mjm.jnt_qposadr.astype(np.int32), dtype=int)
    self.jnt_dofadr = wp.array(mjm.jnt_dofadr.astype(np.int32), dtype=int)
    self.njnt = mjm.njnt
    self.ctrlrange = wp.array(
        np.where(
            mjm.actuator_ctrllimited[:, None].astype(bool),
            mjm.actuator_ctrlrange,
            np.array([-1e10, 1e10]),
        ).astype(np.float32),
        dtype=float,
    )

  def integrate_host(self, mjm, x: np.ndarray, dx: np.ndarray) -> np.ndarray:
    """Host reference of x (+) dx, used in tests and on small batches."""
    nq, nv = self.nq, self.nv
    out = np.array(x, dtype=np.float64)
    qpos = out[:nq].copy()
    mujoco.mj_integratePos(mjm, qpos, np.asarray(dx[:nv], dtype=np.float64), 1.0)
    out[:nq] = qpos
    out[nq:] = x[nq:] + dx[nv:]
    return out

  def difference_host(self, mjm, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
    """Host reference of x2 (-) x1."""
    nq, nv = self.nq, self.nv
    dq = np.zeros(nv)
    mujoco.mj_differentiatePos(
        mjm, dq, 1.0, np.asarray(x1[:nq], float), np.asarray(x2[:nq], float)
    )
    return np.concatenate([dq, np.asarray(x2[nq:], float) - x1[nq:]])


@wp.func
def integrate_qpos(
    jnt_type: wp.array(dtype=int),
    jnt_qposadr: wp.array(dtype=int),
    jnt_dofadr: wp.array(dtype=int),
    qpos_in: wp.array2d(dtype=float),
    win: int,
    dq: wp.array2d(dtype=float),
    wdq: int,
    dq_offset: int,
    scale: float,
    qpos_out: wp.array2d(dtype=float),
    wout: int,
):
  """qpos_out[wout] = qpos_in[win] (+) scale * dq[wdq, dq_offset:]."""
  for j in range(jnt_type.shape[0]):
    qa = jnt_qposadr[j]
    da = jnt_dofadr[j] + dq_offset
    t = jnt_type[j]
    if t == 0:  # free
      for i in range(3):
        qpos_out[wout, qa + i] = qpos_in[win, qa + i] + scale * dq[wdq, da + i]
      q = _read_quat(qpos_in, win, qa + 3)
      v = wp.vec3(dq[wdq, da + 3], dq[wdq, da + 4], dq[wdq, da + 5]) * scale
      qn = quat_integrate(q, v)
      for i in range(4):
        qpos_out[wout, qa + 3 + i] = qn[i]
    elif t == 1:  # ball
      q = _read_quat(qpos_in, win, qa)
      v = wp.vec3(dq[wdq, da], dq[wdq, da + 1], dq[wdq, da + 2]) * scale
      qn = quat_integrate(q, v)
      for i in range(4):
        qpos_out[wout, qa + i] = qn[i]
    else:  # hinge / slide
      qpos_out[wout, qa] = qpos_in[win, qa] + scale * dq[wdq, da]


@wp.func
def difference_qpos(
    jnt_type: wp.array(dtype=int),
    jnt_qposadr: wp.array(dtype=int),
    jnt_dofadr: wp.array(dtype=int),
    qpos1: wp.array2d(dtype=float),
    w1: int,
    qpos2: wp.array2d(dtype=float),
    w2: int,
    out: wp.array2d(dtype=float),
    wout: int,
    out_offset: int,
    scale: float,
):
  """out[wout, out_offset:+nv] = scale * (qpos2[w2] (-) qpos1[w1])."""
  for j in range(jnt_type.shape[0]):
    qa = jnt_qposadr[j]
    da = jnt_dofadr[j] + out_offset
    t = jnt_type[j]
    if t == 0:
      for i in range(3):
        out[wout, da + i] = scale * (qpos2[w2, qa + i] - qpos1[w1, qa + i])
      v = quat_sub(_read_quat(qpos2, w2, qa + 3), _read_quat(qpos1, w1, qa + 3))
      for i in range(3):
        out[wout, da + 3 + i] = scale * v[i]
    elif t == 1:
      v = quat_sub(_read_quat(qpos2, w2, qa), _read_quat(qpos1, w1, qa))
      for i in range(3):
        out[wout, da + i] = scale * v[i]
    else:
      out[wout, da] = scale * (qpos2[w2, qa] - qpos1[w1, qa])


@wp.kernel
def set_state_kernel(
    x: wp.array2d(dtype=float),  # (nsrc, nx)
    src_of_world: wp.array(dtype=int),
    time0: wp.array(dtype=float),  # (nsrc,)
    nq: int,
    nv: int,
    na: int,
    qpos: wp.array2d(dtype=float),
    qvel: wp.array2d(dtype=float),
    act: wp.array2d(dtype=float),
    time: wp.array(dtype=float),
    qacc_warmstart: wp.array2d(dtype=float),
):
  """Copies source states into worlds; also resets time and warmstart."""
  w = wp.tid()
  s = src_of_world[w]
  for i in range(nq):
    qpos[w, i] = x[s, i]
  for i in range(nv):
    qvel[w, i] = x[s, nq + i]
    qacc_warmstart[w, i] = 0.0
  for i in range(na):
    act[w, i] = x[s, nq + nv + i]
  time[w] = time0[s]


@wp.kernel
def get_state_kernel(
    qpos: wp.array2d(dtype=float),
    qvel: wp.array2d(dtype=float),
    act: wp.array2d(dtype=float),
    nq: int,
    nv: int,
    na: int,
    stage: int,
    states: wp.array3d(dtype=float),  # (nworld, T+1, nx)
):
  w = wp.tid()
  for i in range(nq):
    states[w, stage, i] = qpos[w, i]
  for i in range(nv):
    states[w, stage, nq + i] = qvel[w, i]
  for i in range(na):
    states[w, stage, nq + nv + i] = act[w, i]


def integrate(m: mjw.Model, d: mjw.Data) -> None:
  """Second half of `mjw.step`: advance a `forward`-ed state in time.

  `step = forward + integrate`. Splitting it lets residuals be evaluated in
  between, at the pre-step state (qpos and qvel included), which is where
  MJPC's residual sensors are evaluated inside `mj_step`.
  """
  integrator = int(m.opt.integrator)
  if integrator == int(mujoco.mjtIntegrator.mjINT_EULER):
    mjw.euler(m, d)
  elif integrator == int(mujoco.mjtIntegrator.mjINT_RK4):
    mjw.rungekutta4(m, d)
  elif integrator in (
      int(mujoco.mjtIntegrator.mjINT_IMPLICIT),
      int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
  ):
    mjw.implicit(m, d)
  else:
    mjw.discrete(m, d)
