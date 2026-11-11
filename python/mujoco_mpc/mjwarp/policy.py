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
"""Control parameterizations evaluated on device, one thread per world.

* Spline: knots evenly spaced over the horizon, zero / linear / cubic
  interpolation (cubic uses the finite-difference Hermite slopes of
  mjpc/spline). Used by the sampling optimizer.
* Feedback: u = u_nom + alpha * k + K (x (-) x_nom), the iLQR line-search
  policy. Each world reads the nominal of its problem.
"""

import numpy as np
import warp as wp

from mujoco_mpc.mjwarp import state as state_lib

INTERP = {"zero": 0, "linear": 1, "cubic": 2}


@wp.func
def _clamp_ctrl(u: float, ctrlrange: wp.array2d(dtype=float), i: int) -> float:
  return wp.clamp(u, ctrlrange[i, 0], ctrlrange[i, 1])


@wp.kernel
def spline_ctrl_kernel(
    knots: wp.array3d(dtype=float),  # (nworld, nknot, nu)
    interp: int,
    stage: int,
    horizon: int,
    ctrlrange: wp.array2d(dtype=float),
    ctrl: wp.array2d(dtype=float),
    actions: wp.array3d(dtype=float),  # (nworld, T, nu) or (0, 0, 0)
    record: int,
):
  w, i = wp.tid()
  nknot = knots.shape[1]
  u = float(0.0)
  if nknot == 1 or horizon <= 1:
    u = knots[w, 0, i]
  else:
    tau = float(stage) * float(nknot - 1) / float(horizon - 1)
    k = wp.min(int(wp.floor(tau)), nknot - 2)
    s = tau - float(k)
    p0 = knots[w, k, i]
    p1 = knots[w, k + 1, i]
    if interp == 0:
      u = p0
      if s >= 1.0:
        u = p1
    elif interp == 1:
      u = (1.0 - s) * p0 + s * p1
    else:
      # Hermite with finite-difference slopes (unit knot spacing)
      m0 = p1 - p0
      if k > 0:
        m0 = 0.5 * (p1 - knots[w, k - 1, i])
      m1 = p1 - p0
      if k + 2 < nknot:
        m1 = 0.5 * (knots[w, k + 2, i] - p0)
      s2 = s * s
      s3 = s2 * s
      u = (
          (2.0 * s3 - 3.0 * s2 + 1.0) * p0
          + (s3 - 2.0 * s2 + s) * m0
          + (-2.0 * s3 + 3.0 * s2) * p1
          + (s3 - s2) * m1
      )
  u = _clamp_ctrl(u, ctrlrange, i)
  ctrl[w, i] = u
  if record != 0:
    actions[w, stage, i] = u


@wp.kernel
def feedback_ctrl_kernel(
    # layout
    jnt_type: wp.array(dtype=int),
    jnt_qposadr: wp.array(dtype=int),
    jnt_dofadr: wp.array(dtype=int),
    nq: int,
    nv: int,
    na: int,
    nu: int,
    # current state
    qpos: wp.array2d(dtype=float),
    qvel: wp.array2d(dtype=float),
    act: wp.array2d(dtype=float),
    # policy, per problem
    prob_of_world: wp.array(dtype=int),
    alpha: wp.array(dtype=float),  # per world
    x_nom: wp.array3d(dtype=float),  # (nprob, T+1, nx)
    u_nom: wp.array3d(dtype=float),  # (nprob, T, nu)
    k_ff: wp.array3d(dtype=float),  # (nprob, T, nu)
    K_fb: wp.array4d(dtype=float),  # (nprob, T, nu, ndx)
    use_feedback: int,
    stage: int,
    ctrlrange: wp.array2d(dtype=float),
    # scratch / outputs
    qpos_nom: wp.array2d(dtype=float),  # (nworld, nq)
    dx: wp.array2d(dtype=float),  # (nworld, ndx)
    ctrl: wp.array2d(dtype=float),
    actions: wp.array3d(dtype=float),
    record: int,
):
  w = wp.tid()
  p = prob_of_world[w]
  if use_feedback != 0:
    for i in range(nq):
      qpos_nom[w, i] = x_nom[p, stage, i]
    state_lib.difference_qpos(
        jnt_type, jnt_qposadr, jnt_dofadr, qpos_nom, w, qpos, w, dx, w, 0, 1.0
    )
    for i in range(nv):
      dx[w, nv + i] = qvel[w, i] - x_nom[p, stage, nq + i]
    for i in range(na):
      dx[w, 2 * nv + i] = act[w, i] - x_nom[p, stage, nq + nv + i]
  ndx = 2 * nv + na
  for i in range(nu):
    u = u_nom[p, stage, i] + alpha[w] * k_ff[p, stage, i]
    if use_feedback != 0:
      for j in range(ndx):
        u += K_fb[p, stage, i, j] * dx[w, j]
    u = _clamp_ctrl(u, ctrlrange, i)
    ctrl[w, i] = u
    if record != 0:
      actions[w, stage, i] = u


def knots_from_actions(actions: np.ndarray, nknot: int) -> np.ndarray:
  """Samples per-step actions (..., T, nu) at the knot times (linear fit)."""
  T = actions.shape[-2]
  if nknot == 1:
    return actions[..., :1, :].copy()
  t = np.linspace(0, T - 1, nknot)
  lo = np.minimum(np.floor(t).astype(int), T - 1)
  hi = np.minimum(lo + 1, T - 1)
  s = (t - lo)[:, None]
  return (1 - s) * actions[..., lo, :] + s * actions[..., hi, :]
