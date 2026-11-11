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
r"""Checks the MJWarp pterosaur Launch residual against the C++ task.

Build the C++ dump tool (MJPC_BUILD_MJWARP_VALIDATION=ON), then:

  cmake -B build -DMJPC_BUILD_MJWARP_VALIDATION=ON
  cmake --build build --target pterosaur_residual_dump
  cd python && python -m mujoco_mpc.mjwarp.validate_pterosaur \
      ../build/bin/pterosaur_residual_dump

States come from a CPU rollout with small random controls; times are spread
over the whole launch so every phase is visited. Run this after editing
`Pterosaur::ResidualFn::Residual` or the Launch part of `TransitionLocked`,
and port the change to tasks/pterosaur.py until it passes.
"""

import subprocess
import sys

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

from mujoco_mpc.mjwarp import model as model_lib
from mujoco_mpc.mjwarp import norms
from mujoco_mpc.mjwarp.tasks import pterosaur

# residual terms the port leaves at zero on purpose (weight 0 in Launch)
SKIP = ("Gait",)


def main(dump_tool: str, nrow: int = 120, seed: int = 1) -> bool:
  wp.config.quiet = True
  mjm = model_lib.load_model(pterosaur.XML)
  task = pterosaur.PterosaurLaunch(mjm)
  nq, nv, nu = mjm.nq, mjm.nv, mjm.nu
  rng = np.random.default_rng(seed)
  d = mujoco.MjData(mjm)
  mujoco.mj_resetDataKeyframe(mjm, d, 0)
  mujoco.mj_forward(mjm, d)
  # states from a stable rollout, times uniformly over the launch
  times = np.linspace(0.0, task.duration - 1e-3, nrow)
  rows, u = [], np.zeros(nu)
  steps_per_row = 3
  for i in range(nrow * steps_per_row):
    u = np.clip(0.95 * u + 0.05 * rng.normal(size=nu), -0.3, 0.3)
    d.ctrl[:] = u
    if i % steps_per_row == 0:
      rows.append(np.concatenate([[times[i // steps_per_row]], d.qpos, d.qvel, d.ctrl]))
    mujoco.mj_step(mjm, d)
  rows = np.array(rows)

  text = f"{nrow}\n" + "\n".join(" ".join(f"{v:.17g}" for v in r) for r in rows)
  out = subprocess.run(
      [dump_tool, str(model_lib.TASKS_DIR / pterosaur.XML)],
      input=text, capture_output=True, text=True, check=True,
  ).stdout
  ref = np.array([[float(x) for x in line.split()] for line in out.splitlines()])
  spec = task.spec
  w_ref, r_ref = ref[:, : spec.nterm], ref[:, spec.nterm :]

  task.bind(rows[0, 1 : 1 + nq + nv], rows[0, 0])
  m, dw = model_lib.make_batch(mjm, d, nrow)
  dw.qpos.assign(rows[:, 1 : 1 + nq].astype(np.float32))
  dw.qvel.assign(rows[:, 1 + nq : 1 + nq + nv].astype(np.float32))
  dw.ctrl.assign(rows[:, 1 + nq + nv :].astype(np.float32))
  dw.time.assign(rows[:, 0].astype(np.float32))
  mjw.forward(m, dw)
  res = wp.zeros((nrow, spec.nresidual), dtype=float)
  wt = wp.zeros((nrow, spec.nterm), dtype=float)
  task.residual(m, dw, wp.array(rows[:, 0].astype(np.float32), dtype=float), res, wt)
  r_w, w_w = res.numpy(), wt.numpy()

  ok = True
  werr = np.abs(w_w - w_ref).max()
  print(f"{'weights':15s} max|diff| {werr:.2e}")
  ok &= werr < 1e-5
  for k, name in enumerate(spec.names):
    a, b = spec.offsets[k], spec.offsets[k] + spec.dims[k]
    err = np.abs(r_w[:, a:b] - r_ref[:, a:b]).max()
    tol = 1e-2 if name == "GroundContact" else 1e-3  # touch: float32 contact forces
    status = "skipped" if name in SKIP else ("ok" if err < tol else "MISMATCH")
    print(f"{name:15s} max|diff| {err:.2e}  {status}")
    if name not in SKIP:
      ok &= err < tol
  mask = np.ones(spec.nresidual, bool)
  for name in SKIP:
    k = spec.index(name)
    mask[spec.offsets[k] : spec.offsets[k] + spec.dims[k]] = False
  c_w, _, _ = norms.norm_derivatives(spec, np.where(mask, r_w, 0), w_w)
  c_r, _, _ = norms.norm_derivatives(spec, np.where(mask, r_ref, 0), w_ref)
  rel = np.max(np.abs(c_w - c_r) / np.maximum(1e-6, np.abs(c_r)))
  print(f"weighted cost max relative error {rel:.2e}")
  ok &= rel < 1e-3
  print("PASS" if ok else "FAIL")
  return bool(ok)


if __name__ == "__main__":
  if len(sys.argv) < 2:
    sys.exit(__doc__)
  sys.exit(0 if main(sys.argv[1]) else 1)
