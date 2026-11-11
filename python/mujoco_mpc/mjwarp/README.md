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
uv venv -p 3.12 && source .venv/bin/activate
uv pip install mujoco-warp absl-py pytest   # pulls in warp-lang and mujoco
python -c "import warp as wp; wp.init(); print(wp.get_device())"   # expect cuda:0
```

The `warp-lang` wheels on PyPI (1.19) are built against CUDA 13 and need an
NVIDIA driver >= 13.0. With an older driver (e.g. 12.8), Warp warns
"Insufficient CUDA driver version" and silently runs on the CPU. Install
the CUDA 12 build from NVIDIA's GitHub release instead:

```bash
uv pip install "warp-lang @ https://github.com/NVIDIA/warp/releases/download/v1.19.0/warp_lang-1.19.0+cu12-py3-none-manylinux_2_28_x86_64.whl"
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
- **Mesh collisions are nondeterministic.** MJWarp's contact pipeline is not
  bitwise reproducible: identical worlds get different contact sets. With
  the pterosaur's mesh collisions this is large (see
  [measurements](#measured-on-nvidia-l40s)): one control sequence replayed in
  256 identical worlds costs anywhere from 532 to 3176. CEM and iLQR then
  optimize noise. Optimize the pterosaur with `--no-mesh-collisions`.
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

## Measured on NVIDIA L40S

46 GB, driver 570.158.01 (CUDA 12.8), warp-lang 1.19.0+cu12,
mujoco-warp 4.0.1, mujoco 4.0.1, 2026-11-11. CPU baseline: one thread of
the same node.

**Rollouts** (`benchmark --horizon 100`, pterosaur, cost included):

| nworld | mesh collisions: steps/s | vs 1 CPU thread | `--no-mesh-collisions`: steps/s | vs 1 CPU thread |
|---:|---:|---:|---:|---:|
| 256 | 66,780 | 3.3× | 219,921 | 5.1× |
| 1024 | 188,626 | 9.3× | 549,708 | 12.6× |
| 4096 | 560,949 | 27.7× | 1,074,469 | 24.7× |
| 16384 | 916,968 | 45.2× | 1,386,241 | 31.9× |

CPU MuJoCo: 20,284 steps/s with meshes, 43,490 without.

**Linearization** of a 100-step trajectory (4949 worlds): 3 ms on the GPU vs
134 ms (meshes) / 80 ms (no meshes) for 100 CPU `mjd_transitionFD` calls,
43× / 31×. Run eagerly instead of as a CUDA graph, one MJWarp step with mesh
collisions takes ~80× longer (452 vs 5.6 ms for 16384 worlds), which is why
every batch is captured. At demo scale (8 problems × 161 points, 63k worlds)
a linearization takes 35 ms with 8192-world chunks (11 GiB) or 33 ms with
16384 (22 GiB). 32768 runs out of memory when the graph is instantiated.

**Contact buffers.** With the probe-sized defaults, 0.3–0.6% of worlds
overflow `njmax=512` under random or CEM controls. `--njmax 1024` cuts this
to at most 8 worlds per batch (≤0.2%); typical worlds use ~100 rows, so the
remainder are worlds that blow up. Many worlds hit MJWarp's solver
iteration limit (reported separately; not a buffer overflow).

**Reproducibility.** The saved best trajectory's controls replayed in 256
identical GPU worlds:

| | cost: median (IQR), range | identical worlds diverge at step | GPU vs CPU diverges at step |
|---|---|---:|---:|
| mesh collisions | 916 (580–1312), 532–3176 | 19 | 19 |
| `--no-mesh-collisions` | spread ~0.2% | 81 | 82 |

Plain `mjw.step` on identical worlds shows the same: with meshes, `nefc`
ranges over 66–78 across worlds in the same state. With mesh collisions the
"CPU re-simulation gap" is the GPU's own noise, and costs reported by the
optimizers are single noisy draws.

**Demo** (`--horizon 1.6 --nsample 512 --cem-iters 40 --ilqr-iters 20`,
wall time, best cost over problems):

| nproblem | collisions | CEM | iLQR (derivatives / backward / line search) | iLQR start → end |
|---:|---|---|---|---|
| 1 | mesh, `--njmax 1024` | 39.9 s, 541.6 | 10.5 s (0.9 / 1.0 / 6.1) | 989.4 → 972.4 |
| 8 | mesh, `--njmax 1024` | 92.4 s, 431.0 | 30.9 s (4.5 / 11.3 / 12.0) | 554.2 → 540.4 |
| 32 | mesh, `--njmax 1024` | 276.3 s, 447.0 | 80.8 s (7.5 / 49.7 / 19.5) | 468.4 → 468.4 |
| 1 | no meshes | 11.3 s, 674.2 | 4.1 s (0.3 / 0.2 / 1.9) | 671.2 → 671.2 |
| 8 | no meshes | 34.6 s, 642.4 | 21.4 s (1.3 / 13.3 / 4.9) | 642.6 → 558.4 |
| 32 | no meshes | 92.4 s, 651.9 | 66.8 s (3.9 / 53.8 / 6.9) | 649.5 → 536.8 |

- With meshes, iLQR's first rollout of the CEM controls already costs far
  more than CEM reported (noise), and iLQR rarely improves on it.
- Without meshes, iLQR starts where CEM ended and improves it by up to 18%.
  CEM's best rises slightly with nproblem because the demo spreads the
  exploration scale over problems; the wider ones do worse. GATO-style
  batching pays off in iLQR: the best of 32 problems is 4% better than the
  best of 8 and 20% better than a single problem.
- The NumPy backward pass dominates iLQR at nproblem 32 (50–54 s of
  67–81 s).
- CPU re-simulation of the best trajectory, no meshes: against the same
  pruned model it diverges at step 67–76 open loop; with the iLQR gains
  it stays within threshold until step 150 of 160 for nproblem 1 (73–75
  for 8 and 32). Against the full mesh model it diverges at step 16–31
  (model mismatch).

`python/launch.npz` is the mesh-collision nproblem 8 run (the
command in the quick start) and is subject to the noise above.
