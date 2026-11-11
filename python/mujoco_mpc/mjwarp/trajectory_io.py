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
"""Saving, CPU re-simulation and replay of optimized trajectories.

MJWarp simulates in float32 with its own collision pipeline, so an open-loop
control sequence optimized on the GPU drifts when replayed in CPU MuJoCo
(float64), the engine MJPC uses, especially through contact changes. Use
`resimulate` to measure that gap before trusting an open-loop result; for
contact-rich motions, the saved states are better used as a reference for
MJPC to track (or replay with the iLQR feedback gains, `feedback=True`).
"""

import dataclasses
import pathlib
from typing import Optional

import mujoco
import numpy as np

from mujoco_mpc.mjwarp import state as state_lib


def save(path, *, mjm: mujoco.MjModel, time0: float, states, actions,
         stage_costs=None, K=None, **extra) -> pathlib.Path:
  """Writes a trajectory to .npz (time, qpos, qvel, act, ctrl, ...)."""
  path = pathlib.Path(path)
  states = np.asarray(states)
  T = len(actions)
  nq, nv = mjm.nq, mjm.nv
  data = dict(
      time=time0 + np.arange(T + 1) * mjm.opt.timestep,
      qpos=states[:, :nq],
      qvel=states[:, nq : nq + nv],
      act=states[:, nq + nv :],
      ctrl=np.asarray(actions),
      timestep=mjm.opt.timestep,
  )
  if stage_costs is not None:
    data["stage_costs"] = np.asarray(stage_costs)
  if K is not None:
    data["K"] = np.asarray(K)
  data.update(extra)
  np.savez_compressed(path, **data)
  return path


@dataclasses.dataclass
class Resimulation:
  states: np.ndarray  # (T+1, nx) CPU MuJoCo states
  qpos_error: np.ndarray  # (T+1,) max |qpos_cpu - qpos_warp| per step
  tangent_error: np.ndarray  # (T+1,) ||x_cpu (-) x_warp|| per step
  diverged_at: Optional[int]  # first step with tangent error > threshold


def resimulate(
    mjm: mujoco.MjModel,
    states: np.ndarray,
    actions: np.ndarray,
    *,
    time0: float = 0.0,
    K: Optional[np.ndarray] = None,
    threshold: float = 0.1,
) -> Resimulation:
  """Replays controls on CPU MuJoCo and compares with the GPU states.

  Args:
    mjm: host model (pass the original model, e.g. with `implicit`
      integrator, to measure the gap to MJPC's simulator)
    states: (T+1, nx) GPU trajectory
    actions: (T, nu) GPU controls
    time0: start time
    K: optional (T, nu, ndx) feedback gains: u = u_t + K_t (x (-) x_t)
    threshold: tangent-space error that counts as diverged

  Returns:
    Resimulation
  """
  lay = state_lib.StateLayout(mjm)
  nq, nv = mjm.nq, mjm.nv
  d = mujoco.MjData(mjm)
  d.qpos[:] = states[0, :nq]
  d.qvel[:] = states[0, nq : nq + nv]
  d.act[:] = states[0, nq + nv :]
  d.time = time0
  T = len(actions)
  out = np.zeros((T + 1, lay.nx))
  lo = np.where(mjm.actuator_ctrllimited, mjm.actuator_ctrlrange[:, 0], -np.inf)
  hi = np.where(mjm.actuator_ctrllimited, mjm.actuator_ctrlrange[:, 1], np.inf)
  for t in range(T + 1):
    out[t] = np.concatenate([d.qpos, d.qvel, d.act])
    if t == T:
      break
    u = np.array(actions[t], dtype=np.float64)
    if K is not None:
      u = u + K[t] @ lay.difference_host(mjm, states[t], out[t])
    d.ctrl[:] = np.clip(u, lo, hi)
    mujoco.mj_step(mjm, d)
  qerr = np.abs(out[:, :nq] - states[:, :nq]).max(axis=1)
  terr = np.array(
      [np.linalg.norm(lay.difference_host(mjm, states[t], out[t])) for t in range(T + 1)]
  )
  bad = np.flatnonzero(terr > threshold)
  return Resimulation(
      states=out, qpos_error=qerr, tangent_error=terr,
      diverged_at=int(bad[0]) if bad.size else None,
  )


def replay(mjm: mujoco.MjModel, states: np.ndarray, *, loop: bool = True,
           speed: float = 1.0) -> None:
  """Kinematic replay of states in the MuJoCo viewer (needs a display)."""
  import time
  import mujoco.viewer

  d = mujoco.MjData(mjm)
  nq, nv = mjm.nq, mjm.nv
  dt = mjm.opt.timestep / speed
  with mujoco.viewer.launch_passive(mjm, d) as viewer:
    while viewer.is_running():
      for x in states:
        start = time.time()
        d.qpos[:] = x[:nq]
        d.qvel[:] = x[nq : nq + nv]
        mujoco.mj_forward(mjm, d)
        viewer.sync()
        time.sleep(max(0.0, dt - (time.time() - start)))
        if not viewer.is_running():
          return
      if not loop:
        return
