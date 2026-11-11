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
r"""Offline trajectory optimization of the pterosaur launch on the GPU.

Runs batched CEM over spline knots, then batched iLQR (finite-difference
Jacobians as parallel MJWarp worlds) seeded with the CEM solution, saves the
best trajectory and reports the GPU -> CPU MuJoCo re-simulation gap.

  cd python
  python -m mujoco_mpc.demos.mjwarp.pterosaur_launch_trajopt \
      --horizon 1.6 --nproblem 8 --nsample 512 --out launch.npz

Without an NVIDIA GPU, Warp falls back to its CPU device: use tiny settings
(e.g. --nproblem 1 --nsample 16 --cem-iters 2 --ilqr-iters 1 --horizon 0.3).
"""

import argparse
import gc
import time

import numpy as np
import warp as wp

from mujoco_mpc.mjwarp import cem as cem_lib
from mujoco_mpc.mjwarp import ilqr as ilqr_lib
from mujoco_mpc.mjwarp import model as model_lib
from mujoco_mpc.mjwarp import policy as policy_lib
from mujoco_mpc.mjwarp import rollout as rollout_lib
from mujoco_mpc.mjwarp import trajectory_io
from mujoco_mpc.mjwarp.tasks import pterosaur


def main():
  p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  p.add_argument("--horizon", type=float, default=1.6,
                 help="seconds from launch start (full launch ~3.2 s)")
  p.add_argument("--optimizer", choices=["cem", "ilqr", "cem+ilqr"],
                 default="cem+ilqr")
  p.add_argument("--nproblem", type=int, default=4,
                 help="independent problems solved in one batch")
  p.add_argument("--nsample", type=int, default=256, help="CEM samples/problem")
  p.add_argument("--nknot", type=int, default=12)
  p.add_argument("--cem-iters", type=int, default=40)
  p.add_argument("--ilqr-iters", type=int, default=20)
  p.add_argument("--init-std", type=float, default=0.4)
  p.add_argument("--fd-eps", type=float, default=1e-3)
  p.add_argument("--max-fd-worlds", type=int, default=8192)
  p.add_argument("--nconmax", type=int, default=None,
                 help="contacts per world (default: sized by a CPU probe)")
  p.add_argument("--njmax", type=int, default=None,
                 help="constraint rows per world (default: sized by a CPU probe)")
  p.add_argument("--no-mesh-collisions", action="store_true",
                 help="disable the 'collision' mesh class (feet/knees stay)")
  p.add_argument("--no-self-collision", action="store_true",
                 help="robot geoms collide with the ground only; keeps mesh "
                      "ground contacts but drops mesh-mesh self-contacts, "
                      "which MJWarp cannot reproduce (see README)")
  p.add_argument("--integrator", default=None,
                 help="override integrator used on the GPU (e.g. implicitfast)")
  p.add_argument("--seed", type=int, default=0)
  p.add_argument("--device", default=None, help="warp device, e.g. cuda:0")
  p.add_argument("--out", default="pterosaur_launch.npz")
  p.add_argument("--view", action="store_true", help="replay in the viewer")
  args = p.parse_args()

  if args.device:
    wp.set_device(args.device)
  print(f"warp device: {wp.get_device()}")

  classes = ("collision",) if args.no_mesh_collisions else ()
  mjm = model_lib.load_model(
      pterosaur.XML, disable_collision_classes=classes,
      self_collision=not args.no_self_collision, integrator=args.integrator,
  )
  mjm_cpu = model_lib.load_model(pterosaur.XML)  # unmodified, for the check
  task = pterosaur.PterosaurLaunch(mjm)
  horizon = int(round(args.horizon / mjm.opt.timestep))
  print(f"launch duration {task.duration:.3f} s, optimizing {horizon} steps")

  qpos0 = model_lib.key_qpos(mjm, "home")
  qpos0[3:7] = [1, 0, 0, 0] if not np.any(qpos0[3:7]) else qpos0[3:7] / np.linalg.norm(qpos0[3:7])
  x0 = np.concatenate([qpos0, np.zeros(mjm.nv), np.zeros(mjm.na)])

  actions = np.zeros((args.nproblem, horizon, mjm.nu))
  states = stage = K = None
  cost = None
  if "cem" in args.optimizer:
    cfg = cem_lib.CEMConfig(
        nsample=args.nsample, nelite=max(4, args.nsample // 16),
        nknot=args.nknot, init_std=args.init_std,
        iterations=args.cem_iters, seed=args.seed,
    )
    opt = cem_lib.CEM(mjm, task, horizon, x0, nproblem=args.nproblem, config=cfg,
                      nconmax=args.nconmax, njmax=args.njmax)
    # spread the problems' exploration scales (batched hyperparameter sweep)
    opt.std *= np.geomspace(0.5, 2.0, args.nproblem)[:, None, None]
    t0 = time.time()
    res = opt.optimize(callback=lambda it, c: print(
        f"  cem {it:3d}  best {c.min():.4f}  per-problem {np.round(c, 3)}"))
    print(f"CEM: {time.time() - t0:.2f} s, best cost {res.cost.min():.4f}")
    actions, states, stage, cost = res.actions, res.states, res.stage_costs, res.cost
    # free CEM's world batch before iLQR allocates its own (GPU memory)
    del opt, res
    gc.collect()
    rollout_lib.release_graph_memory()
  if "ilqr" in args.optimizer:
    cfg = ilqr_lib.ILQRConfig(
        iterations=args.ilqr_iters, fd_eps=args.fd_eps,
        max_fd_worlds=args.max_fd_worlds,
    )
    opt = ilqr_lib.ILQR(mjm, task, horizon, x0, nproblem=args.nproblem, config=cfg,
                        nconmax=args.nconmax, njmax=args.njmax)
    t0 = time.time()
    res = opt.optimize(actions, callback=lambda it, c: print(
        f"  ilqr {it:3d}  best {c.min():.4f}  per-problem {np.round(c, 3)}"))
    print(f"iLQR: {time.time() - t0:.2f} s, best cost {res.cost.min():.4f}, "
          f"timing {{{', '.join(f'{k}: {v:.2f}s' for k, v in res.timing.items())}}}")
    actions, states, stage, cost, K = (
        res.actions, res.states, res.stage_costs, res.cost, res.K)

  b = int(np.argmin(cost))
  path = trajectory_io.save(
      args.out, mjm=mjm, time0=0.0, states=states[b], actions=actions[b],
      stage_costs=stage[b], K=None if K is None else K[b],
      knots=policy_lib.knots_from_actions(actions[b], args.nknot),
  )
  print(f"saved best trajectory (problem {b}, cost {cost[b]:.4f}) to {path}")

  # unmodified model: gap to MJPC's simulator; with pruned collisions also
  # the same model as on the GPU, which isolates the GPU/float32 gap
  checks = [("", mjm_cpu)] + ([(", GPU model", mjm)]
                                  if classes or args.no_self_collision or args.integrator
                                  else [])
  for suffix, check_model in checks:
    for label, gains in (("open loop", None), ("with iLQR feedback", None if K is None else K[b])):
      if label != "open loop" and gains is None:
        continue
      sim = trajectory_io.resimulate(check_model, states[b], actions[b], K=gains)
      print(f"CPU MuJoCo re-simulation ({label}{suffix}): max qpos error "
            f"{sim.qpos_error.max():.3e}, diverged at step {sim.diverged_at}")

  if args.view:
    trajectory_io.replay(mjm_cpu, states[b])


if __name__ == "__main__":
  main()
