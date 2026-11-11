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
"""Ports of the MJPC norms (mjpc/norm.cc) used to turn residuals into cost.

`stage_cost_kernel` runs on device (one thread per world) and is used inside
rollouts. `norm_derivatives` is a vectorized NumPy version that also returns
the gradient and Hessian of each norm with respect to the residual, used by
the Gauss-Newton cost expansion in iLQR (same scheme as
mjpc/planners/cost_derivatives.cc).
"""

import numpy as np
import warp as wp

from mujoco_mpc.mjwarp import model as model_lib


@wp.func
def _norm(
    residual: wp.array2d(dtype=float),
    world: int,
    offset: int,
    dim: int,
    norm: int,
    p: float,
) -> float:
  y = float(0.0)
  if norm == 0:  # kQuadratic: 0.5 * x'x
    for i in range(dim):
      x = residual[world, offset + i]
      y += x * x
    y *= 0.5
  elif norm == 2:  # kL2: sqrt(x'x + p^2) - p
    for i in range(dim):
      x = residual[world, offset + i]
      y += x * x
    y = wp.sqrt(y + p * p) - p
  elif norm == 6:  # kSmoothAbsLoss: sum_i sqrt(x_i^2 + p^2) - p
    for i in range(dim):
      x = residual[world, offset + i]
      y += wp.sqrt(x * x + p * p) - p
  return y


@wp.kernel
def stage_cost_kernel(
    residual: wp.array2d(dtype=float),
    weight: wp.array2d(dtype=float),
    dims: wp.array(dtype=int),
    offsets: wp.array(dtype=int),
    norms: wp.array(dtype=int),
    params: wp.array2d(dtype=float),
    scale: float,
    stage: int,
    # outputs
    total: wp.array(dtype=float),
    stage_cost: wp.array2d(dtype=float),
):
  """total[w] += scale * sum_k weight[w,k] * norm_k(residual[w]); logs stage."""
  world = wp.tid()
  c = float(0.0)
  for k in range(dims.shape[0]):
    wk = weight[world, k]
    if wk != 0.0:
      c += wk * _norm(residual, world, offsets[k], dims[k], norms[k], params[k, 0])
  # guard against non-finite residuals (diverged worlds)
  if not wp.isfinite(c):
    c = 1.0e10
  total[world] = total[world] + scale * c
  if stage >= 0:
    stage_cost[world, stage] = c


class DeviceCostSpec:
  """CostSpec arrays on device."""

  def __init__(self, spec: model_lib.CostSpec):
    self.spec = spec
    self.dims = wp.array(spec.dims, dtype=int)
    self.offsets = wp.array(spec.offsets, dtype=int)
    self.norms = wp.array(spec.norms, dtype=int)
    self.params = wp.array(spec.params.astype(np.float32), dtype=float)


def norm_derivatives(
    spec: model_lib.CostSpec, residual: np.ndarray, weight: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
  """Cost, gradient and Hessian w.r.t. the residual, batched.

  Args:
    spec: cost layout
    residual: (..., nresidual)
    weight: (..., nterm)

  Returns:
    cost (...), grad (..., nresidual), hess (..., nresidual, nresidual); the
    Hessian is block diagonal with one block per term.
  """
  residual = np.asarray(residual, dtype=np.float64)
  weight = np.asarray(weight, dtype=np.float64)
  batch = residual.shape[:-1]
  nr = spec.nresidual
  cost = np.zeros(batch)
  grad = np.zeros(batch + (nr,))
  hess = np.zeros(batch + (nr, nr))
  for k in range(spec.nterm):
    a, n = int(spec.offsets[k]), int(spec.dims[k])
    x = residual[..., a : a + n]
    w = weight[..., k]
    p = spec.params[k, 0]
    norm = spec.norms[k]
    if norm == model_lib.NORM_QUADRATIC:
      y = 0.5 * np.sum(x * x, axis=-1)
      g = x
      h = np.broadcast_to(np.eye(n), batch + (n, n))
    elif norm == model_lib.NORM_L2:
      s = np.sqrt(np.sum(x * x, axis=-1) + p * p)
      y = s - p
      safe = np.where(s > 0, s, 1.0)[..., None]
      g = np.where(s[..., None] > 0, x / safe, 0.0)
      h = (np.eye(n) - g[..., :, None] * g[..., None, :]) / safe[..., None]
    elif norm == model_lib.NORM_SMOOTH_ABS:
      s = np.sqrt(x * x + p * p)
      y = np.sum(s - p, axis=-1)
      g = x / np.where(s > 0, s, 1.0)
      d = np.where(s > 0, p * p / np.where(s > 0, s, 1.0) ** 3, 0.0)
      h = d[..., :, None] * np.eye(n)
    else:
      raise NotImplementedError(norm)
    cost += w * y
    grad[..., a : a + n] = w[..., None] * g
    hess[..., a : a + n, a : a + n] = w[..., None, None] * h
  return cost, grad, hess
