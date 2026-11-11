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
"""Finite-difference dynamics and residual Jacobians as one batch of worlds.

MJPC differentiates a trajectory one timestep at a time with
`mjd_transitionFD` (mjpc/planners/model_derivatives.cc). Here every
(problem, timestep, perturbation) triple is its own MJWarp world, so a whole
trajectory, or a batch of trajectories, is differentiated with a single
`forward` + integrate: nworld = nproblem * (T + 1) * (1 + ndx + nu), processed in chunks of
at most `max_worlds`. Each block of 1 + ndx + nu worlds holds the unperturbed
point plus one forward-difference perturbation per tangent state and control
coordinate, and the same world also yields the residual Jacobian used for the
Gauss-Newton cost expansion (mjpc/planners/cost_derivatives.cc).

MJWarp runs in float32, so the default step size is larger than MJPC's 1e-6.
"""

import dataclasses
from typing import Optional

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

from mujoco_mpc.mjwarp import model as model_lib
from mujoco_mpc.mjwarp import state as state_lib
from mujoco_mpc.mjwarp.tasks import base


@wp.kernel
def _fd_setup_kernel(
    jnt_type: wp.array(dtype=int),
    jnt_qposadr: wp.array(dtype=int),
    jnt_dofadr: wp.array(dtype=int),
    nq: int,
    nv: int,
    na: int,
    nu: int,
    item0: int,
    nitem_total: int,
    X: wp.array2d(dtype=float),  # (nitem_total, nx) point per item
    U: wp.array2d(dtype=float),  # (nitem_total, nu)
    times: wp.array(dtype=float),  # (nitem_total,)
    eps: float,
    # scratch
    x_in: wp.array2d(dtype=float),  # (nworld, nq)
    dq: wp.array2d(dtype=float),  # (nworld, nv)
    # outputs: data fields
    qpos: wp.array2d(dtype=float),
    qvel: wp.array2d(dtype=float),
    act: wp.array2d(dtype=float),
    ctrl: wp.array2d(dtype=float),
    time: wp.array(dtype=float),
    qacc_warmstart: wp.array2d(dtype=float),
    stage_time: wp.array(dtype=float),
):
  w = wp.tid()
  npert = 1 + 2 * nv + na + nu
  item = item0 + w // npert
  j = w % npert - 1  # -1: nominal
  if item >= nitem_total:
    item = nitem_total - 1  # padding worlds duplicate the last item
    j = -1
  for i in range(nq):
    x_in[w, i] = X[item, i]
  for i in range(nv):
    dq[w, i] = 0.0
  if j >= 0 and j < nv:
    dq[w, j] = eps
  state_lib.integrate_qpos(
      jnt_type, jnt_qposadr, jnt_dofadr, x_in, w, dq, w, 0, 1.0, qpos, w
  )
  for i in range(nv):
    v = X[item, nq + i]
    if j == nv + i:
      v += eps
    qvel[w, i] = v
    qacc_warmstart[w, i] = 0.0
  for i in range(na):
    a = X[item, nq + nv + i]
    if j == 2 * nv + i:
      a += eps
    act[w, i] = a
  for i in range(nu):
    u = U[item, i]
    if j == 2 * nv + na + i:
      u += eps
    ctrl[w, i] = u
  time[w] = times[item]
  stage_time[w] = times[item]


@wp.kernel
def _fd_collect_kernel(
    jnt_type: wp.array(dtype=int),
    jnt_qposadr: wp.array(dtype=int),
    jnt_dofadr: wp.array(dtype=int),
    nq: int,
    nv: int,
    na: int,
    nu: int,
    nres: int,
    nterm: int,
    item0: int,
    nitem_total: int,
    eps: float,
    qpos: wp.array2d(dtype=float),
    qvel: wp.array2d(dtype=float),
    act: wp.array2d(dtype=float),
    residual: wp.array2d(dtype=float),
    weight: wp.array2d(dtype=float),
    # outputs, indexed by item
    F: wp.array3d(dtype=float),  # (nitem, ndx, ndx + nu) dynamics Jacobian
    J: wp.array3d(dtype=float),  # (nitem, nres, ndx + nu) residual Jacobian
    r0: wp.array2d(dtype=float),  # (nitem, nres)
    w0: wp.array2d(dtype=float),  # (nitem, nterm)
    dx: wp.array2d(dtype=float),  # scratch (nworld, ndx)
):
  w = wp.tid()
  npert = 1 + 2 * nv + na + nu
  local = w // npert
  item = item0 + local
  j = w % npert - 1
  if item >= nitem_total:
    return
  nom = local * npert
  if j < 0:
    for i in range(nres):
      r0[item, i] = residual[w, i]
    for k in range(nterm):
      w0[item, k] = weight[w, k]
    return
  inv = 1.0 / eps
  state_lib.difference_qpos(
      jnt_type, jnt_qposadr, jnt_dofadr, qpos, nom, qpos, w, dx, w, 0, inv
  )
  for i in range(nv):
    F[item, i, j] = dx[w, i]
    F[item, nv + i, j] = (qvel[w, i] - qvel[nom, i]) * inv
  for i in range(na):
    F[item, 2 * nv + i, j] = (act[w, i] - act[nom, i]) * inv
  for i in range(nres):
    J[item, i, j] = (residual[w, i] - residual[nom, i]) * inv


@dataclasses.dataclass
class Derivatives:
  """Linearization of a batch of trajectories, host arrays.

  A: (M, T, ndx, ndx), B: (M, T, ndx, nu) dynamics Jacobians.
  r: (M, T+1, nres) residuals, w: (M, T+1, nterm) weights.
  Jx: (M, T+1, nres, ndx), Ju: (M, T+1, nres, nu) residual Jacobians
  (Ju at T is zero by construction of the terminal cost).
  """

  A: np.ndarray
  B: np.ndarray
  r: np.ndarray
  w: np.ndarray
  Jx: np.ndarray
  Ju: np.ndarray


class FiniteDifference:
  """Computes `Derivatives` for (M, T+1) trajectory points in big batches."""

  def __init__(
      self,
      mjm: mujoco.MjModel,
      task: base.Task,
      *,
      max_worlds: int = 16384,
      eps: float = 1e-3,
      probe: Optional[mujoco.MjData] = None,
      nconmax: Optional[int] = None,
      njmax: Optional[int] = None,
      use_graph: Optional[bool] = None,
  ):
    self.mjm = mjm
    self.task = task
    self.eps = eps
    self.layout = lay = state_lib.StateLayout(mjm)
    self.npert = 1 + lay.ndx + lay.nu
    self.max_items_per_chunk = max(1, max_worlds // self.npert)
    if probe is None:
      probe = mujoco.MjData(mjm)
      if mjm.nkey:
        mujoco.mj_resetDataKeyframe(mjm, probe, 0)
    self.probe = probe
    if nconmax is None or njmax is None:
      c, j = model_lib.contact_capacity(mjm, probe)
      nconmax, njmax = nconmax or c, njmax or j
    self.nconmax, self.njmax = nconmax, njmax
    # On CUDA each chunk is captured into a graph: eager MJWarp steps are
    # far slower (pterosaur, L40S: 452 ms eager vs 5.6 ms captured for
    # 16384 worlds).
    if use_graph is None:
      use_graph = wp.get_device().is_cuda
    self.use_graph = use_graph
    self._graphs = {}
    self._task_key = None
    # the world batch is allocated on first use, sized to what is needed
    self.items_per_chunk = 0
    self.nworld = 0
    self._cap = 0

  def _alloc_worlds(self, nitem: int):
    """(Re)allocates the world batch for min(nitem, max) items per chunk."""
    items = min(nitem, self.max_items_per_chunk)
    if items <= self.items_per_chunk:
      return
    lay, task = self.layout, self.task
    self.items_per_chunk = items
    self.nworld = items * self.npert
    self.m, self.d = model_lib.make_batch(
        self.mjm, self.probe, self.nworld, self.nconmax, self.njmax
    )
    self.x_in = wp.zeros((self.nworld, lay.nq), dtype=float)
    self.dq = wp.zeros((self.nworld, lay.nv), dtype=float)
    self.dx = wp.zeros((self.nworld, lay.ndx), dtype=float)
    self.stage_time = wp.zeros(self.nworld, dtype=float)
    self.residual = wp.zeros((self.nworld, task.nresidual), dtype=float)
    self.weight = wp.zeros((self.nworld, task.nterm), dtype=float)
    self._graphs.clear()

  def _ensure(self, nitem: int):
    self._alloc_worlds(nitem)
    if nitem <= self._cap:
      return
    lay, task = self.layout, self.task
    self._cap = nitem
    self.X = wp.zeros((nitem, lay.nx), dtype=float)
    self.U = wp.zeros((nitem, lay.nu), dtype=float)
    self.T = wp.zeros(nitem, dtype=float)
    self.F = wp.zeros((nitem, lay.ndx, lay.ndx + lay.nu), dtype=float)
    self.J = wp.zeros((nitem, task.nresidual, lay.ndx + lay.nu), dtype=float)
    self.r0 = wp.zeros((nitem, task.nresidual), dtype=float)
    self.w0 = wp.zeros((nitem, task.nterm), dtype=float)
    self._graphs.clear()

  def _chunk(self, item0: int, nitem: int):
    """Perturb, step and collect one chunk of items starting at item0."""
    lay, task, d = self.layout, self.task, self.d
    wp.launch(
        _fd_setup_kernel,
        dim=self.nworld,
        inputs=[
            lay.jnt_type, lay.jnt_qposadr, lay.jnt_dofadr,
            lay.nq, lay.nv, lay.na, lay.nu, item0, nitem,
            self.X, self.U, self.T, self.eps, self.x_in, self.dq,
        ],
        outputs=[d.qpos, d.qvel, d.act, d.ctrl, d.time, d.qacc_warmstart,
                 self.stage_time],
    )
    mjw.forward(self.m, d)
    task.residual(self.m, d, self.stage_time, self.residual, self.weight)
    state_lib.integrate(self.m, d)
    wp.launch(
        _fd_collect_kernel,
        dim=self.nworld,
        inputs=[
            lay.jnt_type, lay.jnt_qposadr, lay.jnt_dofadr,
            lay.nq, lay.nv, lay.na, lay.nu, task.nresidual, task.nterm,
            item0, nitem, self.eps, d.qpos, d.qvel, d.act,
            self.residual, self.weight,
        ],
        outputs=[self.F, self.J, self.r0, self.w0, self.dx],
    )

  def compute(
      self, states: np.ndarray, actions: np.ndarray, times: np.ndarray
  ) -> Derivatives:
    """Linearizes trajectories.

    Args:
      states: (M, T+1, nx)
      actions: (M, T, nu)
      times: (M, T+1) stage times

    Returns:
      Derivatives
    """
    lay, task = self.layout, self.task
    M, T1, nx = states.shape
    T = T1 - 1
    nitem = M * T1
    self._ensure(nitem)
    # terminal point reuses the last control; its control columns are unused
    U = np.concatenate([actions, actions[:, -1:]], axis=1)
    self.X.assign(np.ascontiguousarray(states.reshape(nitem, nx), np.float32))
    self.U.assign(np.ascontiguousarray(U.reshape(nitem, lay.nu), np.float32))
    self.T.assign(np.ascontiguousarray(times.reshape(nitem), np.float32))
    task_key = task.graph_key()
    if task_key != self._task_key:
      self._graphs.clear()
      self._task_key = task_key
    for item0 in range(0, nitem, self.items_per_chunk):
      if not self.use_graph:
        self._chunk(item0, nitem)
        continue
      # item0 and nitem are scalar launch arguments, frozen into the graph
      graph = self._graphs.get((item0, nitem))
      if graph is None:
        self._chunk(item0, nitem)  # warm up: compile outside capture
        with wp.ScopedCapture() as capture:
          self._chunk(item0, nitem)
        graph = self._graphs[(item0, nitem)] = capture.graph
      wp.capture_launch(graph)
    model_lib.check_overflow(self.d, "during finite differencing ")
    ndx = lay.ndx
    F = self.F.numpy()[:nitem].astype(np.float64).reshape(M, T1, ndx, -1)
    J = self.J.numpy()[:nitem].astype(np.float64).reshape(M, T1, task.nresidual, -1)
    r = self.r0.numpy()[:nitem].astype(np.float64).reshape(M, T1, -1)
    w = self.w0.numpy()[:nitem].astype(np.float64).reshape(M, T1, -1)
    Jx, Ju = J[..., :ndx], J[..., ndx:].copy()
    Ju[:, T] = 0.0
    return Derivatives(
        A=F[:, :T, :, :ndx], B=F[:, :T, :, ndx:], r=r, w=w, Jx=Jx, Ju=Ju
    )
