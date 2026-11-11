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
"""Task interface for MJWarp trajectory optimization.

A task is the GPU counterpart of an MJPC `Task`/`ResidualFn`: given batched
MJWarp data after `forward` (and before integration, so every field, qpos and
qvel included, refers to the stage state, exactly like MJPC's residual
sensors inside `mj_step`), it writes the residual vector and the per-term
weights of every world. Weights are per world so that tasks can reproduce MJPC's time-varying
weight schedules (set in `TransitionLocked`) as a pure function of time.
"""

import abc

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

from mujoco_mpc.mjwarp import model as model_lib
from mujoco_mpc.mjwarp import norms


class Task(abc.ABC):
  """Base class: residual + weight kernels for one MJPC task."""

  def __init__(self, mjm: mujoco.MjModel):
    self.mjm = mjm
    self.spec = model_lib.cost_spec(mjm)
    self.device_spec = norms.DeviceCostSpec(self.spec)

  @property
  def nresidual(self) -> int:
    return self.spec.nresidual

  @property
  def nterm(self) -> int:
    return self.spec.nterm

  def bind(self, x0: np.ndarray, time0: float = 0.0) -> None:
    """Freezes task state that MJPC captures at a mode switch.

    Called once per optimization with the initial state. The default does
    nothing; stateful tasks override it.
    """
    del x0, time0

  def graph_key(self):
    """Hashable summary of host-side state passed to kernels by value.

    Launch arguments that are not arrays (e.g. Warp structs) are frozen into
    a captured CUDA graph. `Rollout` recaptures when this value changes, so
    tasks whose `bind` changes such arguments must override it.
    """
    return None

  @abc.abstractmethod
  def residual(
      self,
      m: mjw.Model,
      d: mjw.Data,
      stage_time: wp.array,
      residual: wp.array2d,
      weight: wp.array2d,
  ) -> None:
    """Launches kernels filling residual[w, :] and weight[w, :]."""
