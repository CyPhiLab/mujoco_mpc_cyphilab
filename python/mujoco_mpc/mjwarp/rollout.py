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
"""Batched rollouts of MJPC tasks on MJWarp.

`Rollout` owns `nworld` simulation worlds. One call simulates every world for
`horizon` steps from its source state under a device policy, accumulating the
task cost, and optionally records states and actions. On CUDA the whole
horizon (state reset, policy, step, residual, cost) is captured once into a
CUDA graph and relaunched afterwards, so only policy parameters and initial
states cross the host/device boundary.

Cost convention: stage costs l_t for t = 0..T-1 are evaluated at (x_t, u_t)
and a terminal cost l_T at x_T (after a final `forward`), as in
mjpc/trajectory.cc. The returned value is the sum over t = 0..T. Each step is
`forward`, residual, then `state.integrate`, so task kernels see the pre-step
state in every Data field, matching MJPC's residual sensors.
"""

from typing import Optional

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

from mujoco_mpc.mjwarp import model as model_lib
from mujoco_mpc.mjwarp import norms
from mujoco_mpc.mjwarp import policy as policy_lib
from mujoco_mpc.mjwarp import state as state_lib
from mujoco_mpc.mjwarp.tasks import base


@wp.kernel
def _stage_time_kernel(
    time0: wp.array(dtype=float),
    src_of_world: wp.array(dtype=int),
    stage: int,
    dt: float,
    stage_time: wp.array(dtype=float),
):
  w = wp.tid()
  stage_time[w] = time0[src_of_world[w]] + float(stage) * dt


class SplinePolicy:
  """Per-world spline knots, (nworld, nknot, nu) on device."""

  def __init__(self, nworld: int, nknot: int, nu: int, interp: str = "cubic"):
    self.nknot = nknot
    self.interp = policy_lib.INTERP[interp]
    self.knots = wp.zeros((nworld, nknot, nu), dtype=float)

  def set(self, knots: np.ndarray) -> None:
    self.knots.assign(np.asarray(knots, dtype=np.float32))

  def apply(self, r: "Rollout", stage: int) -> None:
    wp.launch(
        policy_lib.spline_ctrl_kernel,
        dim=(r.nworld, r.layout.nu),
        inputs=[self.knots, self.interp, stage, r.horizon, r.layout.ctrlrange],
        outputs=[r.d.ctrl, r.actions, int(r.record_actions)],
    )


class FeedbackPolicy:
  """iLQR policy u = u_nom + alpha k + K (x (-) x_nom), per problem."""

  def __init__(
      self, nworld: int, nprob: int, horizon: int, layout: state_lib.StateLayout
  ):
    nx, nu, ndx = layout.nx, layout.nu, layout.ndx
    self.nprob = nprob
    self.prob_of_world = wp.zeros(nworld, dtype=int)
    self.alpha = wp.zeros(nworld, dtype=float)
    self.x_nom = wp.zeros((nprob, horizon + 1, nx), dtype=float)
    self.u_nom = wp.zeros((nprob, horizon, nu), dtype=float)
    self.k_ff = wp.zeros((nprob, horizon, nu), dtype=float)
    self.K_fb = wp.zeros((nprob, horizon, nu, ndx), dtype=float)
    self.use_feedback = 1
    self._qpos_nom = wp.zeros((nworld, layout.nq), dtype=float)
    self._dx = wp.zeros((nworld, ndx), dtype=float)

  def set(
      self,
      prob_of_world: np.ndarray,
      alpha: np.ndarray,
      u_nom: np.ndarray,
      x_nom: Optional[np.ndarray] = None,
      k_ff: Optional[np.ndarray] = None,
      K_fb: Optional[np.ndarray] = None,
  ) -> None:
    self.prob_of_world.assign(np.asarray(prob_of_world, dtype=np.int32))
    self.alpha.assign(np.asarray(alpha, dtype=np.float32))
    self.u_nom.assign(np.asarray(u_nom, dtype=np.float32))
    if x_nom is not None:
      self.x_nom.assign(np.asarray(x_nom, dtype=np.float32))
    if k_ff is not None:
      self.k_ff.assign(np.asarray(k_ff, dtype=np.float32))
    else:
      self.k_ff.zero_()
    if K_fb is not None:
      self.K_fb.assign(np.asarray(K_fb, dtype=np.float32))
      self.use_feedback = 1
    else:
      self.use_feedback = 0

  def apply(self, r: "Rollout", stage: int) -> None:
    lay = r.layout
    wp.launch(
        policy_lib.feedback_ctrl_kernel,
        dim=r.nworld,
        inputs=[
            lay.jnt_type, lay.jnt_qposadr, lay.jnt_dofadr,
            lay.nq, lay.nv, lay.na, lay.nu,
            r.d.qpos, r.d.qvel, r.d.act,
            self.prob_of_world, self.alpha,
            self.x_nom, self.u_nom, self.k_ff, self.K_fb,
            self.use_feedback, stage, lay.ctrlrange,
        ],
        outputs=[
            self._qpos_nom, self._dx, r.d.ctrl, r.actions,
            int(r.record_actions),
        ],
    )


class Rollout:
  """Simulates `nworld` worlds for `horizon` steps and scores them."""

  def __init__(
      self,
      mjm: mujoco.MjModel,
      task: base.Task,
      nworld: int,
      horizon: int,
      *,
      nsource: int = 1,
      probe: Optional[mujoco.MjData] = None,
      nconmax: Optional[int] = None,
      njmax: Optional[int] = None,
      record_states: bool = True,
      record_actions: bool = True,
      use_graph: Optional[bool] = None,
  ):
    self.mjm = mjm
    self.task = task
    self.nworld = nworld
    self.horizon = horizon
    self.nsource = nsource
    self.dt = float(mjm.opt.timestep)
    self.layout = state_lib.StateLayout(mjm)
    if probe is None:
      probe = mujoco.MjData(mjm)
      if mjm.nkey:
        mujoco.mj_resetDataKeyframe(mjm, probe, 0)
    self.m, self.d = model_lib.make_batch(mjm, probe, nworld, nconmax, njmax)
    lay = self.layout
    self.record_states = record_states
    self.record_actions = record_actions
    self.x_src = wp.zeros((nsource, lay.nx), dtype=float)
    self.time_src = wp.zeros(nsource, dtype=float)
    self.src_of_world = wp.zeros(nworld, dtype=int)
    self.stage_time = wp.zeros(nworld, dtype=float)
    self.residual = wp.zeros((nworld, task.nresidual), dtype=float)
    self.weight = wp.zeros((nworld, task.nterm), dtype=float)
    self.total = wp.zeros(nworld, dtype=float)
    self.stage_cost = wp.zeros((nworld, horizon + 1), dtype=float)
    self.states = wp.zeros(
        (nworld, horizon + 1, lay.nx) if record_states else (1, 1, 1),
        dtype=float,
    )
    self.actions = wp.zeros(
        (nworld, horizon, lay.nu) if record_actions else (1, 1, 1),
        dtype=float,
    )
    if use_graph is None:
      use_graph = wp.get_device().is_cuda
    self.use_graph = use_graph
    self._graphs = {}
    self._task_key = None

  # ---------------------------------------------------------------------------
  def set_sources(
      self,
      x0: np.ndarray,
      time0: np.ndarray | float = 0.0,
      src_of_world: Optional[np.ndarray] = None,
  ) -> None:
    """Initial states (nsource, nx), times and world -> source map."""
    x0 = np.atleast_2d(np.asarray(x0, dtype=np.float32))
    if x0.shape != (self.nsource, self.layout.nx):
      raise ValueError(f"x0 shape {x0.shape} != {(self.nsource, self.layout.nx)}")
    self.x_src.assign(x0)
    self.time_src.assign(
        np.broadcast_to(np.asarray(time0, dtype=np.float32), (self.nsource,))
    )
    if src_of_world is None:
      src_of_world = np.arange(self.nworld) % self.nsource
    self.src_of_world.assign(np.asarray(src_of_world, dtype=np.int32))

  def _score(self, stage: int) -> None:
    wp.launch(
        _stage_time_kernel,
        dim=self.nworld,
        inputs=[self.time_src, self.src_of_world, stage, self.dt],
        outputs=[self.stage_time],
    )
    self.task.residual(self.m, self.d, self.stage_time, self.residual, self.weight)
    ds = self.task.device_spec
    wp.launch(
        norms.stage_cost_kernel,
        dim=self.nworld,
        inputs=[
            self.residual, self.weight, ds.dims, ds.offsets, ds.norms,
            ds.params, 1.0, stage,
        ],
        outputs=[self.total, self.stage_cost],
    )

  def _record(self, stage: int) -> None:
    if self.record_states:
      lay = self.layout
      wp.launch(
          state_lib.get_state_kernel,
          dim=self.nworld,
          inputs=[self.d.qpos, self.d.qvel, self.d.act, lay.nq, lay.nv, lay.na, stage],
          outputs=[self.states],
      )

  def _simulate(self, policy) -> None:
    lay = self.layout
    wp.launch(
        state_lib.set_state_kernel,
        dim=self.nworld,
        inputs=[self.x_src, self.src_of_world, self.time_src, lay.nq, lay.nv, lay.na],
        outputs=[self.d.qpos, self.d.qvel, self.d.act, self.d.time, self.d.qacc_warmstart],
    )
    self.total.zero_()
    self._record(0)
    for t in range(self.horizon):
      policy.apply(self, t)
      mjw.forward(self.m, self.d)
      self._score(t)  # at (x_t, u_t), before integration
      state_lib.integrate(self.m, self.d)
      self._record(t + 1)
    # terminal cost at x_T
    mjw.forward(self.m, self.d)
    self._score(self.horizon)

  def run(self, policy) -> np.ndarray:
    """Simulates all worlds under `policy`; returns total cost (nworld,)."""
    if self.use_graph:
      # non-array launch arguments (task structs, use_feedback) are frozen
      # into the graph: recapture when they change
      task_key = self.task.graph_key()
      if task_key != self._task_key:
        self._graphs.clear()
        self._task_key = task_key
      key = (id(policy), getattr(policy, "use_feedback", None))
      entry = self._graphs.get(key)
      if entry is None:
        self._simulate(policy)  # warm up: compile kernels outside capture
        with wp.ScopedCapture() as capture:
          self._simulate(policy)
        # keep the policy alive so its id() cannot be reused by another
        entry = self._graphs[key] = (capture.graph, policy)
      wp.capture_launch(entry[0])
    else:
      self._simulate(policy)
    total = self.total.numpy().astype(np.float64)
    model_lib.check_overflow(self.d, "during rollout ")
    return total

  # convenience accessors ----------------------------------------------------
  def get_states(self, worlds=None) -> np.ndarray:
    s = self.states.numpy()
    return s if worlds is None else s[worlds]

  def get_actions(self, worlds=None) -> np.ndarray:
    a = self.actions.numpy()
    return a if worlds is None else a[worlds]

  def get_stage_costs(self, worlds=None) -> np.ndarray:
    c = self.stage_cost.numpy()
    return c if worlds is None else c[worlds]
