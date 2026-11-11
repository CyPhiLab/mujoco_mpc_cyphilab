# GPU offline trajectory optimization with MJWarp

`mujoco_mpc.mjwarp` runs offline trajectory optimization for MJPC tasks on
[MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp) (MJWarp). Many
simulation worlds run in parallel on an NVIDIA GPU. The C++ MJPC app does not
change. Optimized trajectories are saved to `.npz`, checked against CPU
MuJoCo, and can be replayed.

## Why a separate Python path

- MJWarp is a Python/Warp library. It has no C/C++ API, so it cannot be
  dropped into MJPC's C++ planners.
- MJPC task costs are C++ `ResidualFn::Residual` callbacks behind
  `mjSENS_USER` sensors, which cannot run on the GPU. Each task's residual is
  therefore ported to a Warp kernel (`tasks/`). The cost layout (term names,
  dimensions, norms, default weights) is still read from the task XML, so the
  two stay in sync. A C++ dump tool checks that the port is faithful (see
  [Validating a port](#validating-a-port)).

## Relation to GATO (arXiv:2510.07625)

GATO is a hand-written CUDA batched SQP. It uses analytic rigid-body
derivatives generated from a URDF (GRiD) and a batched PCG KKT solver. Its
dynamics model has no MuJoCo soft contacts, mesh collisions or fluid forces,
and its costs come from a fixed menu. It therefore cannot represent the
pterosaur launch or the swimmers. This package borrows GATO's batching
structure and keeps MuJoCo's dynamics:

| GATO idea | here |
|---|---|
| solve a batch of M trajectory optimization problems at once and keep the best | `nproblem` in `CEM` and `ILQR`: seeds, initial guesses and exploration scales differ per problem |
| form all per-knot-point quantities in parallel | `derivatives.FiniteDifference`: every (problem, timestep, perturbation) is its own MJWarp world, so a whole trajectory is linearized with one physics step instead of T serial `mjd_transitionFD` calls |
| parallel line search over all step sizes and problems | `ILQR`: M × len(alphas) rollouts in one batch |
| keep data on the GPU | rollouts, costs and Jacobians are on device. The small Riccati backward pass runs in NumPy |

## Install

```bash
pip install mujoco-warp absl-py   # pulls in warp-lang and mujoco
```

Fast simulation needs an NVIDIA GPU and driver. Without one, Warp falls back
to its CPU device. The CPU device is fine for tests and small runs but is
slower than CPU MuJoCo.

## Quick start: pterosaur launch

```bash
cd python
python -m mujoco_mpc.demos.mjwarp.pterosaur_launch_trajopt \
    --horizon 1.6 --nproblem 8 --nsample 512 --cem-iters 40 --ilqr-iters 20 \
    --out launch.npz
```

This runs batched CEM over spline knots, then batched iLQR seeded with the CEM
result. It saves the best trajectory, including the iLQR feedback gains `K`,
and re-simulates it on CPU MuJoCo with the original, unmodified model, both
open loop and with feedback. Use `--view` to replay it.

To measure the speedup on your hardware:

```bash
python -m mujoco_mpc.demos.mjwarp.benchmark --nworld 256 1024 4096 16384
```

## Layout

| module | purpose |
|---|---|
| `model.py` | load task XML (optional collision pruning, integrator override); parse the cost layout from user sensors; size MJWarp contact buffers; overflow check |
| `norms.py` | MJPC norms (quadratic, L2, smooth-abs): device stage cost, plus host gradient/Hessian for Gauss-Newton |
| `state.py` | batched state set/get and joint-aware tangent maps (free/ball quaternions); `integrate` = second half of `mjw.step` |
| `policy.py` | device control kernels: spline knots (zero/linear/cubic) and iLQR feedback |
| `rollout.py` | `Rollout`: N worlds × T steps with cost, CUDA-graph captured on GPU |
| `derivatives.py` | batched finite-difference dynamics and residual Jacobians |
| `cem.py`, `ilqr.py` | batched optimizers |
| `trajectory_io.py` | save `.npz`, CPU re-simulation (open loop or with gains), viewer replay |
| `tasks/` | GPU residual ports (`pterosaur.py`: Launch mode) |

Each step runs `forward`, then the task residual, then `state.integrate`. Task
kernels therefore see every `Data` field, `qpos`/`qvel` included, at the
stage state, which is what MJPC's residual sensors see inside `mj_step`.

## Porting another task

1. Subclass `tasks.base.Task` and write a Warp kernel that fills
   `residual[world, :]` in the same order as the C++ `Residual`, and
   `weight[world, :]`. Express MJPC's time-varying weights (set in
   `TransitionLocked`) as a function of the stage time.
2. Freeze the state that `TransitionLocked` records at a mode switch in
   `bind(x0, time0)`, for example start positions or ground height.
3. Validate against C++ as described below.

## Validating a port

```bash
cmake -B build -DMJPC_BUILD_MJWARP_VALIDATION=ON
cmake --build build --target pterosaur_residual_dump
cd python && python -m mujoco_mpc.mjwarp.validate_pterosaur ../build/bin/pterosaur_residual_dump
```

At the time of writing the pterosaur port matches the C++ residual across all
launch phases:
- weights to 1e-7
- residual terms to float32 precision
- weighted cost to a relative 1e-6

`Gait` is excluded: it has weight 0 in Launch and is left at zero in the port.
Run this again after editing `pterosaur.cc`.

## Caveats

- **float32.** MJWarp simulates in float32 with its own collision pipeline.
  Open-loop controls optimized on the GPU drift on CPU MuJoCo once contacts
  change. Track the saved states in MJPC, or apply the saved iLQR gains: in a
  CPU test this reduced the qpos gap about 10×.
- **Contact buffers.** MJWarp preallocates contacts and constraint rows per
  world and drops the excess. Capacities come from a short CPU probe
  simulation (`model.contact_capacity`). If you see an "MJWarp buffer
  overflow" warning, pass a larger `nconmax`/`njmax`, or prune collisions
  (`--no-mesh-collisions` keeps only the feet and knees).
- **Finite-difference step.** float32 needs a larger step than MJPC's 1e-6.
  The default is 1e-3 (`--fd-eps`).
- **Pterosaur weights.** The Launch weight schedule is a direct port of
  `TransitionLocked`, evaluated at each stage time. In that schedule
  `LaunchVelocity` has weight 0 in every phase. Edit
  `LaunchSchedule.phase_weights` to change the schedule for offline
  optimization without touching the C++.
