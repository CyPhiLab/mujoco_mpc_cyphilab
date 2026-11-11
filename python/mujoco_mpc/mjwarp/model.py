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
"""Model loading and cost-term metadata for MJWarp trajectory optimization.

MJPC tasks declare their cost terms as `mjSENS_USER` sensors whose `user`
field holds `[norm, weight, weight_lower, weight_upper, norm_params...]`
(see `mjpc/task.cc`). The residual values themselves are computed by C++
callbacks that do not exist on the GPU, so here we only read the metadata and
let a Warp task kernel fill in the residual vector.
"""

import dataclasses
import pathlib
from typing import Optional, Sequence
import warnings

import mujoco
import mujoco_warp as mjw
import numpy as np

# repository root: python/mujoco_mpc/mjwarp/model.py -> repo
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
TASKS_DIR = _REPO_ROOT / "mjpc" / "tasks"

# mjpc/norm.h
NORM_QUADRATIC = 0
NORM_L2 = 2
NORM_SMOOTH_ABS = 6
SUPPORTED_NORMS = (NORM_QUADRATIC, NORM_L2, NORM_SMOOTH_ABS)
MAX_NORM_PARAMS = 3


@dataclasses.dataclass(frozen=True)
class CostSpec:
  """Cost-term layout parsed from a task's user sensors.

  Attributes:
    names: cost term names, in sensor order
    dims: residual dimension of each term
    offsets: start index of each term in the residual vector
    norms: mjpc NormType of each term
    weights: default weight of each term (the GUI default in MJPC)
    params: (nterm, MAX_NORM_PARAMS) norm parameters, zero padded
    nresidual: total residual dimension
  """

  names: tuple[str, ...]
  dims: np.ndarray
  offsets: np.ndarray
  norms: np.ndarray
  weights: np.ndarray
  params: np.ndarray
  nresidual: int

  @property
  def nterm(self) -> int:
    return len(self.names)

  def index(self, name: str) -> int:
    return self.names.index(name)


def cost_spec(mjm: mujoco.MjModel) -> CostSpec:
  """Reads the cost layout from the leading mjSENS_USER sensors of a model."""
  names, dims, norms, weights, params = [], [], [], [], []
  for i in range(mjm.nsensor):
    if mjm.sensor_type[i] != mujoco.mjtSensor.mjSENS_USER:
      break
    user = mjm.sensor_user[i]
    norm = int(user[0])
    if norm not in SUPPORTED_NORMS:
      raise NotImplementedError(
          f"cost term {mjm.sensor(i).name!r} uses norm {norm}; only"
          f" {SUPPORTED_NORMS} are ported to mjwarp/norms.py"
      )
    p = np.zeros(MAX_NORM_PARAMS)
    nparam = max(0, min(MAX_NORM_PARAMS, mjm.nuser_sensor - 4))
    p[:nparam] = user[4 : 4 + nparam]
    names.append(mjm.sensor(i).name)
    dims.append(int(mjm.sensor_dim[i]))
    norms.append(norm)
    weights.append(float(user[1]))
    params.append(p)
  if not names:
    raise ValueError("model has no leading user sensors (MJPC cost terms)")
  dims = np.array(dims, dtype=np.int32)
  offsets = np.concatenate([[0], np.cumsum(dims)[:-1]]).astype(np.int32)
  return CostSpec(
      names=tuple(names),
      dims=dims,
      offsets=offsets,
      norms=np.array(norms, dtype=np.int32),
      weights=np.array(weights, dtype=np.float64),
      params=np.array(params, dtype=np.float64),
      nresidual=int(dims.sum()),
  )


def load_model(
    xml_path: str | pathlib.Path,
    *,
    disable_collision_geoms: Sequence[str] = (),
    disable_collision_classes: Sequence[str] = (),
    self_collision: bool = True,
    integrator: Optional[str] = None,
) -> mujoco.MjModel:
  """Loads an MJPC task model, optionally pruning collisions.

  Args:
    xml_path: path to the task XML (absolute, or relative to mjpc/tasks)
    disable_collision_geoms: geom names whose contype/conaffinity are zeroed
    disable_collision_classes: default classes whose geoms get the same
    self_collision: if False, geoms on bodies collide only with geoms of the
      world body (the ground), not with each other. MJWarp's mesh-mesh
      contacts (GJK/EPA) are far less reproducible than mesh-plane ones; see
      the README.
    integrator: optional override, one of "euler", "implicit",
      "implicitfast", "rk4"

  Returns:
    the compiled host model
  """
  path = pathlib.Path(xml_path)
  if not path.is_absolute():
    path = TASKS_DIR / path
  spec = mujoco.MjSpec.from_file(str(path))
  if disable_collision_geoms or disable_collision_classes:
    names = set(disable_collision_geoms)
    classes = set(disable_collision_classes)
    for geom in spec.geoms:
      cls = geom.classname.name if geom.classname is not None else ""
      if geom.name in names or cls in classes:
        geom.contype = 0
        geom.conaffinity = 0
  if not self_collision:
    world = spec.worldbody
    world_geoms = set(id(g) for g in world.geoms)
    world_contype = 0
    for geom in world.geoms:
      world_contype |= geom.contype
    for geom in spec.geoms:
      if id(geom) not in world_geoms and (geom.contype or geom.conaffinity):
        geom.contype, geom.conaffinity = 0, world_contype
  if integrator is not None:
    spec.option.integrator = {
        "euler": mujoco.mjtIntegrator.mjINT_EULER,
        "rk4": mujoco.mjtIntegrator.mjINT_RK4,
        "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
        "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    }[integrator]
  return spec.compile()


def _round_size(n: int) -> int:
  """Rounds up to MJWarp's preferred sizes 16, 24, 32, 48, 64, ..."""
  sizes = (2 + (np.arange(24) % 2)) * (2 ** (np.arange(24) // 2 + 3))
  return int(sizes[np.searchsorted(sizes, n)])


def contact_capacity(
    mjm: mujoco.MjModel,
    mjd: mujoco.MjData,
    margin: float = 3.0,
    probe_steps: int = 300,
    seed: int = 0,
) -> tuple[int, int]:
  """Per-world contact and constraint capacity from a host probe.

  MJWarp preallocates contacts (nconmax) and constraint rows (njmax) per
  world, and silently drops the excess. Trajectory optimization visits
  states with more contact than the initial one (e.g. after touchdown), so
  the probe simulates `probe_steps` from `mjd` under zero and random
  controls and sizes for the worst state seen, times `margin`.
  """
  rng = np.random.default_rng(seed)
  ncon, nefc = 0, 0
  lo = np.where(mjm.actuator_ctrllimited, mjm.actuator_ctrlrange[:, 0], -1.0)
  hi = np.where(mjm.actuator_ctrllimited, mjm.actuator_ctrlrange[:, 1], 1.0)
  for scale in (0.0, 0.3, 1.0):
    probe = mujoco.MjData(mjm)
    probe.qpos[:] = mjd.qpos
    probe.qvel[:] = mjd.qvel
    probe.act[:] = mjd.act
    mujoco.mj_forward(mjm, probe)
    ncon, nefc = max(ncon, probe.ncon), max(nefc, probe.nefc)
    for _ in range(probe_steps):
      probe.ctrl[:] = np.clip(scale * rng.uniform(lo, hi), lo, hi)
      mujoco.mj_step(mjm, probe)
      if probe.warning[mujoco.mjtWarning.mjWARN_BADQACC].number:
        break
      ncon, nefc = max(ncon, probe.ncon), max(nefc, probe.nefc)
  nconmax = _round_size(max(16, int(np.ceil(margin * ncon))))
  njmax = _round_size(max(64, int(np.ceil(margin * nefc))))
  return nconmax, njmax


def make_batch(
    mjm: mujoco.MjModel,
    mjd: mujoco.MjData,
    nworld: int,
    nconmax: Optional[int] = None,
    njmax: Optional[int] = None,
) -> tuple[mjw.Model, mjw.Data]:
  """Puts the model on device and allocates `nworld` copies of `mjd`."""
  if nconmax is None or njmax is None:
    c, j = contact_capacity(mjm, mjd)
    nconmax = nconmax or c
    njmax = njmax or j
  m = mjw.put_model(mjm)
  # overflow is reported once per batch by `check_overflow` instead of a
  # device print per world and step
  m.opt.warn_overflow = 0
  d = mjw.put_data(mjm, mjd, nworld=nworld, nconmax=nconmax, njmax=njmax)
  return m, d


_OVERFLOW_HINT = {
    1: "nefc > njmax: pass a larger njmax",
    4: "broadphase: pass a larger nconmax",
    8: "narrowphase/contacts: pass a larger nconmax",
}
# mjw.OverflowType bits that are solver limits, not dropped buffer entries
_SOLVER_LIMIT = {512: "solver iterations", 1024: "linesearch iterations"}


def check_overflow(d: mjw.Data, where: str = "") -> int:
  """Warns if any world overflowed its buffers or hit a solver limit.

  Returns the number of worlds with dropped contacts/constraints (solver
  iteration limits are reported but not counted) and clears the flags.
  """
  flags = d.overflow.numpy()
  bad = np.flatnonzero(flags)
  if not bad.size:
    return 0
  limit_bits = sum(_SOLVER_LIMIT)
  buf = np.flatnonzero(flags & ~limit_bits)
  lim = np.flatnonzero(flags & limit_bits)
  if buf.size:
    bits = int(np.bitwise_or.reduce(flags[buf])) & ~limit_bits
    hints = [h for b, h in _OVERFLOW_HINT.items() if bits & b] or [f"flags {bits}"]
    warnings.warn(
        f"MJWarp buffer overflow {where}in {buf.size}/{len(flags)} worlds "
        f"({'; '.join(hints)}; current nconmax={d.naconmax // d.nworld}, "
        f"njmax={d.njmax}). Dropped contacts make the physics wrong.",
        stacklevel=2,
    )
  if lim.size:
    bits = int(np.bitwise_or.reduce(flags[lim]))
    what = " and ".join(n for b, n in _SOLVER_LIMIT.items() if bits & b)
    warnings.warn(
        f"MJWarp {what} limit reached {where}in {lim.size}/{len(flags)} "
        "worlds (constraint forces not converged; usually worlds that blew "
        "up under extreme controls).",
        stacklevel=2,
    )
  d.overflow.zero_()
  return int(buf.size)


def data_from_state(mjm: mujoco.MjModel, x: np.ndarray) -> mujoco.MjData:
  """Host MjData at state x = [qpos, qvel, act] (e.g. as capacity probe)."""
  d = mujoco.MjData(mjm)
  d.qpos[:] = x[: mjm.nq]
  if len(x) > mjm.nq:
    d.qvel[:] = x[mjm.nq : mjm.nq + mjm.nv]
  if len(x) > mjm.nq + mjm.nv:
    d.act[:] = x[mjm.nq + mjm.nv :]
  mujoco.mj_forward(mjm, d)
  return d


def sensor_adr(mjm: mujoco.MjModel, name: str) -> int:
  """Address of a named sensor in sensordata."""
  return int(mjm.sensor_adr[mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_SENSOR, name)])


def obj_id(mjm: mujoco.MjModel, objtype: mujoco.mjtObj, name: str) -> int:
  i = mujoco.mj_name2id(mjm, objtype, name)
  if i < 0:
    raise ValueError(f"{objtype.name} {name!r} not found")
  return i


def key_qpos(mjm: mujoco.MjModel, name: str) -> np.ndarray:
  return mjm.key_qpos[obj_id(mjm, mujoco.mjtObj.mjOBJ_KEY, name)].copy()


def key_qvel(mjm: mujoco.MjModel, name: str) -> np.ndarray:
  return mjm.key_qvel[obj_id(mjm, mujoco.mjtObj.mjOBJ_KEY, name)].copy()
