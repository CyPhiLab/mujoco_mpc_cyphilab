# Paddle crab gait primitives (offline trajectory optimization)

Cyclic swimming gaits for the PaddleCrabSwimming robot, made offline with
MuJoCo in Python and fed back into the MJPC task as gait primitives. This
follows the recipe used for the pterosaur launch on the
`claude/zen-rubin-gbn3pb` branch (`mjpc/tasks/pterosaur/analysis/`):

1. sampling search
2. gradient polish
3. continuation chains with warm starts
4. export

It is adapted to a periodic problem.

| Pterosaur launch | Paddle crab gaits |
|---|---|
| knot controls over one launch (`launch_trajopt.py`) | per-servo Fourier series in phase, period T a decision variable |
| score at takeoff | score on the **limit cycle**: run `warm` cycles from rest, measure the next `measure` |
| iLQR on per-step controls (`launch_ilqr.py`) | Levenberg-Marquardt on the Fourier coefficients (per-step controls would not stay periodic) |
| continuation over push time, takeoff speed, **contact-smoothing margin** | continuation over **target speed** and **timestep** (5 → 2 → 1 ms); there are no contacts to smooth |
| export: standalone MJCF + control table (`export_model.py`) | same, plus `gait_reference.xml` that the MJPC task loads |

There are no contacts (the build patch disables them) and the fluid forces
come from MuJoCo's smooth ellipsoid model. Finite-difference gradients
therefore work directly, and every Jacobian is one batched
`mujoco.rollout` call.

## Physics

`crab_model.load_model()` runs the same `patch_paddle_crab_robot.py` the MJPC
build runs, so the physics is the physics the task sees:
- implicitfast at 1 ms
- zero gravity
- water density 1000
- contacts off
- kp 50 servos limited to ±3.1 N m

The task drives its `dyntype="user"` servos through `mjcb_act_dyn`, a
first-order setpoint lag with tau = 0.08 s. MuJoCo's built-in `filter`
dynamics is the same law and is used here instead, so no Python callbacks are
needed. Over a 3 s stroke the two agree to the last bit: a qpos difference of
0.0.

## The gait parameterization and residuals (`gait_opt.py`)

A gait is

```
u_i(t) = clip(half_i * (c_i + sum_k a_ik cos(2 pi k t / T) + b_ik sin(2 pi k t / T)))
```

for each of the 18 servos, with K = 3 harmonics. That gives 127 parameters
including log T. All 18 channels are free: there is no left/right mirroring.
Straightness comes from the cost.

The residuals are measured over the measured window, i.e. the limit cycle:

| Residual | Meaning | Scale |
|---|---|---|
| speed | mean forward (−Y) speed − target | 5 mm/s, weight 4 |
| lateral, yaw, depth | drift rates | 3 mm/s, 0.02 rad/s, 3 mm/s |
| heading, tilt | mean heading, mean and RMS body tilt | 0.1 rad |
| cycle | state at the start of the last cycle − at its end (joints, servo filters, body-frame velocities, tilt) | 0.02 rad, 0.01 m/s, 0.05 rad/s |
| effort | RMS servo torque / 3.1 N m | 0.1 |
| saturation | torque beyond ±3.1 N m, command beyond the ctrlrange | 5 % |

`--max_speed` replaces the speed target with a constant pull toward faster
gaits and lowers the effort weight.

## Pipeline (`gait_chain.py`)

1. **search** — 256 random first-harmonic strokes, half of them left/right
   mirrored and half not. The best `--starts` are grown by an evolution
   strategy, and the best of those is polished by LM. All of this runs at the
   coarse 5 ms timestep.
2. **speeds** — one LM solve per target speed, each warm-started from the
   previous speed's gait, period included.
3. **max** — optionally, a max-speed solve starting from the fastest gait.
4. **dts** — each gait is re-polished at 2 ms and then at 1 ms (the MJPC
   simulation timestep).
5. **check** — each final gait is also rolled out at the MJPC planner's 20 ms
   timestep (`plan_dt_check` in `summary.json`).

Each step writes the following to the run directory:
- `stepK_<name>.npz`: theta, metrics, and the limit cycle sampled at 200 phases
- `_controls.npy`: one cycle of commands
- `_history.json`
- with `MUJOCO_GL=osmesa` (or egl): `.mp4` and `_plots.png`

The run also writes `summary.json`, `args.json` and `log.txt`.

```sh
python3.12 -m venv venv && venv/bin/pip install mujoco==3.2.3 numpy imageio imageio-ffmpeg matplotlib
cd mjpc/tasks/paddle_crab_swimming/analysis
MUJOCO_GL=osmesa ../../../../venv/bin/python gait_chain.py \
    --speeds 0.04 0.06 0.08 0.10 0.12 --max_speed --lm_iters 30 --out runs/A
python export_gaits.py --run runs/A --mjpc ../gait_reference.xml --package export_A
MUJOCO_GL=osmesa python render_gait.py runs/A/step12_v0.08_dt0.001.npz
```

MuJoCo 3.2.3 matches the MuJoCo commit that the C++ build pins
(`CMakeLists.txt`, `MUJOCO_MPC_MUJOCO_GIT_TAG`). It has no wheel for Python
3.13, hence the 3.12 venv.

## In MJPC

`export_gaits.py --mjpc ../gait_reference.xml` writes the gait library that
`task.xml` includes. The task's **Gait** drop-down (`residual_select_Gait`)
picks a primitive; **Off** is the original emergent-stroke behaviour.

The selected primitive is fed forward through the servo callback, so the
servo setpoint is `clamp(ctrl + u_gait(t))`. This holds both in the
simulation and in every planner rollout:
- The planner's `ctrl` becomes a correction on top of the primitive.
- `Effort` now prices deviating from the primitive.
- The emergent-rhythm terms (Symmetry, RowPhase, Periodicity) are zeroed
  while a gait runs.
- The gait clock restarts whenever the selection changes. `GaitPhase` shifts
  it, in cycles.

`cost_sweep` (built with MJPC) runs the task headless and closed loop:

```sh
cmake --build build --target cost_sweep
build/bin/cost_sweep --seconds 20 --out sweep.csv Gait=Off Gait=v0.08 Gait=max
build/bin/cost_sweep --open_loop --seconds 20 Gait=v0.08   # primitive alone
```

## Results

(see below)
