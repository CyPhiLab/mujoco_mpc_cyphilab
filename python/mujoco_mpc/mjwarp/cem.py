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
"""Batched cross-entropy method over spline knots (offline trajopt).

`nproblem` independent optimizations run side by side in one batch of
`nproblem * nsample` MJWarp worlds, the batching idea of GATO
(arXiv:2510.07625) applied to a sampling optimizer: problems can differ in
random seed, initial guess and exploration scale, and the best one wins. The
current mean of each problem is always evaluated in its sample 0, so the
reported cost is monotone (the best-ever knots are kept).
"""

import dataclasses
import time
from typing import Callable, Optional

import mujoco
import numpy as np

from mujoco_mpc.mjwarp import model as model_lib
from mujoco_mpc.mjwarp import rollout as rollout_lib
from mujoco_mpc.mjwarp.tasks import base


@dataclasses.dataclass
class CEMConfig:
  nsample: int = 256  # rollouts per problem per iteration
  nelite: int = 16
  nknot: int = 10
  interp: str = "cubic"
  init_std: float = 0.3  # in control units
  min_std: float = 0.02
  mean_smoothing: float = 0.2  # mean <- s * mean + (1 - s) * elite mean
  std_smoothing: float = 0.5
  iterations: int = 50
  seed: int = 0


@dataclasses.dataclass
class CEMResult:
  knots: np.ndarray  # (nproblem, nknot, nu) best-ever knots
  cost: np.ndarray  # (nproblem,)
  states: np.ndarray  # (nproblem, T+1, nx)
  actions: np.ndarray  # (nproblem, T, nu)
  stage_costs: np.ndarray  # (nproblem, T+1)
  history: np.ndarray  # (iterations, nproblem) best cost per iteration
  seconds: float

  @property
  def best(self) -> int:
    return int(np.argmin(self.cost))


class CEM:
  """Batched CEM planner over `horizon` steps from a fixed initial state."""

  def __init__(
      self,
      mjm: mujoco.MjModel,
      task: base.Task,
      horizon: int,
      x0: np.ndarray,
      *,
      time0: float = 0.0,
      nproblem: int = 1,
      config: Optional[CEMConfig] = None,
      probe: Optional[mujoco.MjData] = None,
      nconmax: Optional[int] = None,
      njmax: Optional[int] = None,
  ):
    self.config = cfg = config or CEMConfig()
    self.mjm = mjm
    self.task = task
    self.horizon = horizon
    self.nproblem = nproblem
    self.x0 = np.asarray(x0, dtype=np.float64)
    self.time0 = time0
    task.bind(self.x0, time0)
    if probe is None:
      probe = model_lib.data_from_state(mjm, self.x0)
    if nconmax is None or njmax is None:
      c, j = model_lib.contact_capacity(mjm, probe)
      nconmax, njmax = nconmax or c, njmax or j
    self.nconmax, self.njmax = nconmax, njmax
    nworld = nproblem * cfg.nsample
    self.rollout = rollout_lib.Rollout(
        mjm, task, nworld, horizon, probe=probe, nconmax=nconmax, njmax=njmax,
    )
    self.rollout.set_sources(self.x0[None], time0)
    self.policy = rollout_lib.SplinePolicy(nworld, cfg.nknot, mjm.nu, cfg.interp)
    self.rng = np.random.default_rng(cfg.seed)
    lo = np.where(mjm.actuator_ctrllimited, mjm.actuator_ctrlrange[:, 0], -np.inf)
    hi = np.where(mjm.actuator_ctrllimited, mjm.actuator_ctrlrange[:, 1], np.inf)
    self.ctrl_lo, self.ctrl_hi = lo, hi
    # distribution, per problem
    self.mean = np.zeros((nproblem, cfg.nknot, mjm.nu))
    self.std = np.full((nproblem, cfg.nknot, mjm.nu), cfg.init_std)

  def set_initial(self, knots: np.ndarray, std: Optional[np.ndarray] = None):
    """Initial mean (nknot, nu) or (nproblem, nknot, nu), optional std."""
    self.mean[:] = np.broadcast_to(knots, self.mean.shape)
    if std is not None:
      self.std[:] = np.broadcast_to(std, self.std.shape)

  def optimize(
      self,
      iterations: Optional[int] = None,
      callback: Optional[Callable[[int, np.ndarray], None]] = None,
  ) -> CEMResult:
    cfg = self.config
    iterations = iterations or cfg.iterations
    M, K = self.nproblem, cfg.nsample
    best_cost = np.full(M, np.inf)
    best_knots = self.mean.copy()
    history = []
    start = time.time()
    for it in range(iterations):
      noise = self.rng.standard_normal((M, K) + self.mean.shape[1:])
      noise[:, 0] = 0.0  # sample 0 evaluates the mean
      samples = self.mean[:, None] + self.std[:, None] * noise
      samples = np.clip(samples, self.ctrl_lo, self.ctrl_hi)
      self.policy.set(samples.reshape((M * K,) + samples.shape[2:]))
      cost = self.rollout.run(self.policy).reshape(M, K)
      cost = np.where(np.isfinite(cost), cost, np.inf)
      order = np.argsort(cost, axis=1)
      # best-ever bookkeeping
      top = order[:, 0]
      improved = cost[np.arange(M), top] < best_cost
      best_cost = np.where(improved, cost[np.arange(M), top], best_cost)
      best_knots[improved] = samples[np.arange(M), top][improved]
      # refit
      elite_idx = order[:, : cfg.nelite]
      elites = np.take_along_axis(samples, elite_idx[:, :, None, None], axis=1)
      e_mean = elites.mean(axis=1)
      e_std = elites.std(axis=1)
      s = cfg.mean_smoothing
      self.mean = s * self.mean + (1 - s) * e_mean
      s = cfg.std_smoothing
      self.std = np.maximum(s * self.std + (1 - s) * e_std, cfg.min_std)
      history.append(best_cost.copy())
      if callback is not None:
        callback(it, best_cost)
    seconds = time.time() - start
    states, actions, stage = self.evaluate(best_knots)
    return CEMResult(
        knots=best_knots, cost=best_cost, states=states, actions=actions,
        stage_costs=stage, history=np.array(history), seconds=seconds,
    )

  def evaluate(self, knots: np.ndarray):
    """Rolls out (nproblem, nknot, nu) knots; returns states/actions/costs."""
    M, K = self.nproblem, self.config.nsample
    full = np.repeat(knots[:, None], K, axis=1).reshape((M * K,) + knots.shape[1:])
    self.policy.set(full)
    self.rollout.run(self.policy)
    worlds = np.arange(M) * K
    return (
        self.rollout.get_states(worlds),
        self.rollout.get_actions(worlds),
        self.rollout.get_stage_costs(worlds),
    )
