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
"""Batched iLQR on MuJoCo dynamics, structured after GATO (arXiv:2510.07625).

Per iteration, for `nproblem` trajectories at once:

1. Linearize: dynamics and residual Jacobians of every timestep of every
   problem by finite differences in one batch of MJWarp worlds
   (`derivatives.FiniteDifference`).
2. Backward pass: Gauss-Newton Riccati recursion, vectorized over problems,
   with a per-problem Levenberg-Marquardt regularizer on Quu.
3. Parallel line search: every problem tries all step sizes alpha in one
   batch of rollouts under u = u_nom + alpha k + K (x (-) x_nom) (GATO's
   Alg. 1 line 5); each problem keeps its best improving candidate.

GATO solves the KKT system of a direct (multiple-shooting) SQP with a
batched PCG on analytic rigid-body derivatives. That machinery assumes smooth
dynamics without MuJoCo's contact, fluid and actuator models, so here the
dynamics stay in MuJoCo (shooting / iLQR) and only the batching structure is
borrowed.
"""

import dataclasses
import time
from typing import Callable, Optional

import mujoco
import numpy as np

from mujoco_mpc.mjwarp import derivatives as deriv_lib
from mujoco_mpc.mjwarp import norms
from mujoco_mpc.mjwarp import model as model_lib
from mujoco_mpc.mjwarp import rollout as rollout_lib
from mujoco_mpc.mjwarp.tasks import base


@dataclasses.dataclass
class ILQRConfig:
  iterations: int = 30
  alphas: tuple[float, ...] = (1.0, 0.6, 0.35, 0.2, 0.1, 0.05, 0.02, 0.0)
  mu_init: float = 1e-3
  mu_min: float = 1e-6
  mu_max: float = 1e6
  mu_scale: float = 10.0
  fd_eps: float = 1e-3
  max_fd_worlds: int = 8192  # pterosaur, L40S: ~as fast as 16384 at half the memory
  feedback: bool = True
  tol: float = 1e-6  # relative improvement to stop a problem


@dataclasses.dataclass
class ILQRResult:
  states: np.ndarray  # (M, T+1, nx)
  actions: np.ndarray  # (M, T, nu)
  cost: np.ndarray  # (M,)
  stage_costs: np.ndarray  # (M, T+1)
  K: np.ndarray  # (M, T, nu, ndx) last feedback gains
  history: np.ndarray  # (iters + 1, M)
  seconds: float
  timing: dict

  @property
  def best(self) -> int:
    return int(np.argmin(self.cost))


def backward_pass(
    der: deriv_lib.Derivatives,
    spec,
    mu: np.ndarray,
    ctrl_reg: float = 0.0,
):
  """Gauss-Newton Riccati recursion, batched over problems.

  Args:
    der: linearization
    spec: CostSpec
    mu: (M,) regularizer added to Quu
    ctrl_reg: optional constant control regularization

  Returns:
    k (M,T,nu), K (M,T,nu,ndx), expected reduction terms (dV1, dV2) (M,),
    ok (M,) bool whether Quu stayed positive definite for every t.
  """
  _, g, H = norms.norm_derivatives(spec, der.r, der.w)
  Jx, Ju = der.Jx, der.Ju
  lx = np.einsum("mtrx,mtr->mtx", Jx, g)
  lu = np.einsum("mtru,mtr->mtu", Ju, g)
  HJx = np.einsum("mtrs,mtsx->mtrx", H, Jx)
  lxx = np.einsum("mtry,mtrx->mtyx", Jx, HJx)
  lux = np.einsum("mtru,mtrx->mtux", Ju, HJx)
  luu = np.einsum("mtru,mtrs,mtsv->mtuv", Ju, H, Ju)
  M, T = der.A.shape[:2]
  nu = der.B.shape[-1]
  ndx = der.A.shape[-1]
  Vx = lx[:, T]
  Vxx = lxx[:, T]
  k = np.zeros((M, T, nu))
  K = np.zeros((M, T, nu, ndx))
  dV1 = np.zeros(M)
  dV2 = np.zeros(M)
  ok = np.ones(M, dtype=bool)
  eye = np.eye(nu)
  for t in range(T - 1, -1, -1):
    A, B = der.A[:, t], der.B[:, t]
    Qx = lx[:, t] + np.einsum("myx,my->mx", A, Vx)
    Qu = lu[:, t] + np.einsum("myu,my->mu", B, Vx)
    VA = Vxx @ A
    VB = Vxx @ B
    Qxx = lxx[:, t] + np.swapaxes(A, 1, 2) @ VA
    Quu = luu[:, t] + np.swapaxes(B, 1, 2) @ VB + ctrl_reg * eye
    Qux = lux[:, t] + np.swapaxes(B, 1, 2) @ VA
    Quu_reg = 0.5 * (Quu + np.swapaxes(Quu, 1, 2)) + mu[:, None, None] * eye
    # a problem whose rollout blew up has non-finite derivatives: mark it bad
    # (it keeps its trajectory) instead of letting eigvalsh raise for all
    finite = np.isfinite(Quu_reg).all(axis=(1, 2)) & np.isfinite(Qux).all(axis=(1, 2)) & np.isfinite(Qu).all(axis=1)
    Quu_reg = np.where(finite[:, None, None], Quu_reg, eye)
    Quu = np.where(finite[:, None, None], Quu, eye)
    Qu = np.where(finite[:, None], Qu, 0.0)
    Qux = np.where(finite[:, None, None], Qux, 0.0)
    eig_min = np.where(finite, np.linalg.eigvalsh(Quu_reg)[:, 0], -np.inf)
    bad = ~(eig_min > 1e-12)
    ok &= ~bad
    # keep going for the good problems; bad ones get a safe fallback
    shift = np.where(finite, 1e-6 - np.minimum(eig_min, 0), 0.0)
    Quu_safe = np.where(bad[:, None, None], Quu_reg + shift[:, None, None] * eye, Quu_reg)
    kt = -np.linalg.solve(Quu_safe, Qu[..., None])[..., 0]
    Kt = -np.linalg.solve(Quu_safe, Qux)
    k[:, t], K[:, t] = kt, Kt
    dV1 += np.einsum("mu,mu->m", kt, Qu)
    dV2 += 0.5 * np.einsum("mu,muv,mv->m", kt, Quu, kt)
    KT = np.swapaxes(Kt, 1, 2)
    Vx = Qx + np.einsum("mxu,muv,mv->mx", KT, Quu, kt) + np.einsum("mxu,mu->mx", KT, Qu) + np.einsum("mux,mu->mx", Qux, kt)
    Vxx = Qxx + KT @ Quu @ Kt + KT @ Qux + np.swapaxes(Qux, 1, 2) @ Kt
    Vxx = 0.5 * (Vxx + np.swapaxes(Vxx, 1, 2))
  return k, K, dV1, dV2, ok


class ILQR:
  """Batched iLQR over `horizon` steps from a fixed initial state."""

  def __init__(
      self,
      mjm: mujoco.MjModel,
      task: base.Task,
      horizon: int,
      x0: np.ndarray,
      *,
      time0: float = 0.0,
      nproblem: int = 1,
      config: Optional[ILQRConfig] = None,
      probe: Optional[mujoco.MjData] = None,
      nconmax: Optional[int] = None,
      njmax: Optional[int] = None,
  ):
    self.config = cfg = config or ILQRConfig()
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
    self.nalpha = len(cfg.alphas)
    nworld = nproblem * self.nalpha
    self.rollout = rollout_lib.Rollout(
        mjm, task, nworld, horizon, probe=probe, nconmax=nconmax, njmax=njmax
    )
    self.rollout.set_sources(self.x0[None], time0)
    self.layout = self.rollout.layout
    self.policy = rollout_lib.FeedbackPolicy(nworld, nproblem, horizon, self.layout)
    self.fd = deriv_lib.FiniteDifference(
        mjm, task, max_worlds=cfg.max_fd_worlds, eps=cfg.fd_eps, probe=probe,
        nconmax=nconmax, njmax=njmax,
    )
    self.prob_of_world = np.repeat(np.arange(nproblem), self.nalpha)
    self.alpha_of_world = np.tile(np.asarray(cfg.alphas), nproblem)
    self.times = time0 + np.arange(horizon + 1) * mjm.opt.timestep

  def _rollout_open_loop(self, actions: np.ndarray):
    """Rolls out (M, T, nu) controls; returns states, actions, cost, stages."""
    M, A = self.nproblem, self.nalpha
    self.policy.set(self.prob_of_world, np.zeros(M * A), actions)
    cost = self.rollout.run(self.policy)
    worlds = np.arange(M) * A
    return (
        self.rollout.get_states(worlds).astype(np.float64),
        self.rollout.get_actions(worlds).astype(np.float64),
        cost[worlds],
        self.rollout.get_stage_costs(worlds).astype(np.float64),
    )

  def optimize(
      self,
      actions: np.ndarray,
      iterations: Optional[int] = None,
      callback: Optional[Callable[[int, np.ndarray], None]] = None,
  ) -> ILQRResult:
    """Improves initial controls (T, nu) or (M, T, nu)."""
    cfg = self.config
    iterations = iterations or cfg.iterations
    M, A, T = self.nproblem, self.nalpha, self.horizon
    actions = np.broadcast_to(actions, (M, T, self.mjm.nu)).copy()
    timing = {"derivatives": 0.0, "backward": 0.0, "line_search": 0.0}
    start = time.time()
    X, U, cost, stage = self._rollout_open_loop(actions)
    history = [cost.copy()]
    mu = np.full(M, cfg.mu_init)
    K = np.zeros((M, T, self.mjm.nu, self.layout.ndx))
    active = np.ones(M, dtype=bool)
    times = np.broadcast_to(self.times, (M, T + 1))
    spec = self.task.spec
    for it in range(iterations):
      if not active.any():
        break
      t0 = time.time()
      der = self.fd.compute(X, U, times)
      timing["derivatives"] += time.time() - t0
      t0 = time.time()
      k, K_new, dV1, dV2, ok = backward_pass(der, spec, mu)
      timing["backward"] += time.time() - t0
      t0 = time.time()
      self.policy.set(
          self.prob_of_world, self.alpha_of_world, U, X, k,
          K_new if cfg.feedback else None,
      )
      c = self.rollout.run(self.policy).reshape(M, A)
      c = np.where(np.isfinite(c), c, np.inf)
      timing["line_search"] += time.time() - t0
      best_a = np.argmin(c, axis=1)
      c_best = c[np.arange(M), best_a]
      improved = (c_best < cost) & active
      if improved.any():
        worlds = np.flatnonzero(improved) * A + best_a[improved]
        S = self.rollout.get_states(worlds).astype(np.float64)
        Ua = self.rollout.get_actions(worlds).astype(np.float64)
        SC = self.rollout.get_stage_costs(worlds).astype(np.float64)
        X[improved], U[improved], stage[improved] = S, Ua, SC
        K[improved] = K_new[improved]
      rel = (cost - np.where(improved, c_best, cost)) / np.maximum(np.abs(cost), 1e-12)
      cost = np.where(improved, c_best, cost)
      # regularization schedule
      mu = np.where(improved & ok, np.maximum(mu / cfg.mu_scale, cfg.mu_min), mu)
      mu = np.where(~improved | ~ok, np.minimum(mu * cfg.mu_scale, cfg.mu_max), mu)
      active &= ~(improved & (rel < cfg.tol)) & (mu < cfg.mu_max)
      history.append(cost.copy())
      if callback is not None:
        callback(it, cost)
    return ILQRResult(
        states=X, actions=U, cost=cost, stage_costs=stage, K=K,
        history=np.array(history), seconds=time.time() - start, timing=timing,
    )
