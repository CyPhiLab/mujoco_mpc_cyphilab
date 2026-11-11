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
r"""Throughput of batched pterosaur rollouts and trajectory linearization.

Compares, for several batch sizes, the GPU (MJWarp) with single-threaded CPU
MuJoCo, which is what each of MJPC's planner threads runs:

* rollout: simulated world-steps per second, cost included;
* linearization: seconds to finite-difference a whole T-step trajectory
  (GPU: one batch of worlds; CPU: T calls to mjd_transitionFD).

  cd python
  python -m mujoco_mpc.demos.mjwarp.benchmark --nworld 64 256 1024 4096
"""

import argparse
import time

import mujoco
import numpy as np
import warp as wp

from mujoco_mpc.mjwarp import derivatives as deriv_lib
from mujoco_mpc.mjwarp import model as model_lib
from mujoco_mpc.mjwarp import rollout as rollout_lib
from mujoco_mpc.mjwarp import state as state_lib
from mujoco_mpc.mjwarp.tasks import pterosaur


def main():
  p = argparse.ArgumentParser()
  p.add_argument("--nworld", type=int, nargs="+", default=[64, 256, 1024, 4096])
  p.add_argument("--horizon", type=int, default=100)
  p.add_argument("--repeats", type=int, default=3)
  p.add_argument("--max-fd-worlds", type=int, default=16384)
  p.add_argument("--no-mesh-collisions", action="store_true")
  args = p.parse_args()
  dev = str(wp.get_device())
  print(f"warp device: {dev}")
  classes = ("collision",) if args.no_mesh_collisions else ()
  mjm = model_lib.load_model(pterosaur.XML, disable_collision_classes=classes)
  task = pterosaur.PterosaurLaunch(mjm)
  q = model_lib.key_qpos(mjm, "home")
  q[3:7] = [1, 0, 0, 0]
  x0 = np.concatenate([q, np.zeros(mjm.nv)])
  probe = model_lib.data_from_state(mjm, x0)
  nconmax, njmax = model_lib.contact_capacity(mjm, probe)
  T = args.horizon
  rng = np.random.default_rng(0)

  # CPU baseline: one thread, like one MJPC planner worker
  d = mujoco.MjData(mjm)
  d.qpos[:] = q
  ctrl = rng.uniform(-0.3, 0.3, (T, mjm.nu))
  t0 = time.perf_counter()
  for t in range(T):
    d.ctrl[:] = ctrl[t]
    mujoco.mj_step(mjm, d)
  cpu_rate = T / (time.perf_counter() - t0)
  d.qpos[:] = q
  d.qvel[:] = 0
  lay = state_lib.StateLayout(mjm)
  A = np.zeros((lay.ndx, lay.ndx))
  B = np.zeros((lay.ndx, mjm.nu))
  t0 = time.perf_counter()
  for t in range(T):
    d.ctrl[:] = ctrl[t]
    mujoco.mjd_transitionFD(mjm, d, 1e-6, 0, A, B, None, None)
    mujoco.mj_step(mjm, d)
  cpu_fd = time.perf_counter() - t0
  print(f"CPU MuJoCo, 1 thread: {cpu_rate:,.0f} steps/s; "
        f"linearize {T} steps: {cpu_fd:.3f} s")

  print(f"{'nworld':>8} {'MJWarp steps/s':>15} {'vs 1 CPU thread':>16}  ({dev})")
  for n in args.nworld:
    r = rollout_lib.Rollout(mjm, task, n, T, probe=probe, nconmax=nconmax,
                            njmax=njmax, record_states=False,
                            record_actions=False)
    r.set_sources(x0[None])
    pol = rollout_lib.SplinePolicy(n, 8, mjm.nu)
    pol.set(rng.uniform(-0.3, 0.3, (n, 8, mjm.nu)))
    r.run(pol)  # compile / capture
    t0 = time.perf_counter()
    for _ in range(args.repeats):
      r.run(pol)
    rate = n * T * args.repeats / (time.perf_counter() - t0)
    print(f"{n:>8} {rate:>15,.0f} {rate / cpu_rate:>15.1f}x")
    del r

  fd = deriv_lib.FiniteDifference(mjm, task, max_worlds=args.max_fd_worlds,
                                  probe=probe, nconmax=nconmax, njmax=njmax)
  states = np.tile(x0, (1, T + 1, 1))
  actions = np.tile(ctrl[None], (1, 1, 1))
  times = np.arange(T + 1)[None] * mjm.opt.timestep
  fd.compute(states, actions, times)  # compile
  t0 = time.perf_counter()
  for _ in range(args.repeats):
    fd.compute(states, actions, times)
  gpu_fd = (time.perf_counter() - t0) / args.repeats
  print(f"MJWarp ({dev}) linearize {T} steps ({(T + 1) * fd.npert} worlds, chunks of "
        f"{fd.nworld}): {gpu_fd:.3f} s ({cpu_fd / gpu_fd:.1f}x vs 1 CPU thread)")


if __name__ == "__main__":
  main()
