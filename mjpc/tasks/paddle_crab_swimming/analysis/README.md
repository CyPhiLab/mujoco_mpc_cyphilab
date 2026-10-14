# Paddle crab gait primitives (offline trajectory optimization)

Cyclic swimming gaits for the PaddleCrabSwimming robot, made offline with
MuJoCo in Python and fed back into the MJPC task as gait primitives. The
recipe follows the pterosaur launch work on the `claude/zen-rubin-gbn3pb`
branch (`mjpc/tasks/pterosaur/analysis/`):

1. sampling search
2. gradient polish
3. continuation chains with warm starts
4. export

It is adapted to a periodic problem:

| Pterosaur launch | Paddle crab gaits |
|---|---|
| knot controls over one launch (`launch_trajopt.py`) | per-servo Fourier series in phase; the period T is a decision variable |
| score at takeoff | score on the **limit cycle**: run `warm` cycles, then measure the next `measure` |
| iLQR on per-step controls (`launch_ilqr.py`) | Levenberg-Marquardt on the Fourier coefficients (per-step controls would not stay periodic) |
| continuation over push time, takeoff speed and **contact-smoothing margin** | continuation over **target speed** (a ladder) and **timestep** (2 → 1 ms); there are no contacts to smooth |
| export: standalone MJCF + control table (`export_model.py`) | the same, plus `gait_reference.xml`, which the MJPC task loads |

There are no contacts (the build patch disables them), and the fluid forces
come from MuJoCo's smooth ellipsoid model. Finite-difference gradients
therefore work directly, and each Jacobian is one batched `mujoco.rollout`
call over a process pool.

## Physics (`crab_model.py`)

`load_model()` runs the same `patch_paddle_crab_robot.py` that the MJPC build
runs, so the physics is the physics the task sees:
- implicitfast integrator at 1 ms
- zero gravity
- water density 1000
- contacts off
- kp 50 servos limited to ±3.1 N m

The task drives its `dyntype="user"` servos through `mjcb_act_dyn`, a
first-order setpoint lag with tau = 0.08 s. MuJoCo's built-in `filter`
dynamics follows the same law and is used here instead, so no Python
callbacks are needed. Over a 3 s stroke the two agree to the bit (qpos
difference 0.0).

`load_model(ctrl_limit=...)` lifts the task's command ranges, which are
±1.2 rad proximal, ±1.57 mid, ±1.05 front/centre distal and ±1.57 for the
paddles.

The servo stiffness makes the dynamics stiff. At a 5 ms timestep the same
stroke moves about 20% differently than at 1 ms, and at the planner's 20 ms
it can even go backwards. Searches therefore run at 2 ms and finish at 1 ms.

## Gaits and residuals (`gait_opt.py`)

A gait is

```
u_i(t) = clip(half_i * (c_i + sum_k a_ik cos(2 pi k t / T) + b_ik sin(2 pi k t / T)))
```

for each of the 18 servos, with K = 3 harmonics. That gives 127 parameters,
including log T. All 18 channels are free; there is no left/right mirroring.
The swim direction is a parameter, and the body keeps its heading, so the
direction picks which side leads.

All residuals are measured over the scored window:

| Residual | Meaning | Scale |
|---|---|---|
| speed | mean speed along the swim direction − target | 5 mm/s, weight 4 |
| lateral, yaw, depth | drift rates | 3 mm/s, 0.02 rad/s, 3 mm/s |
| heading | mean yaw off the start heading | 0.1 rad |
| tilt / wobble | steady body tilt / its oscillation | 0.1 rad (tilt weight 0 for the free-posture runs) |
| cycle | state at the start of the last cycle − at its end | 0.02 rad, 0.01 m/s, 0.05 rad/s |
| effort | RMS servo torque / 3.1 N m | 0.1 |
| saturation | torque beyond ±3.1 N m, command beyond the range | 5% |

There are three families of seed strokes:

- `seeds`: random first-harmonic strokes about the zero pose.
- `seeds(offset=0.8)`: the same, with posture offsets anywhere in ±0.8 of the
  command range.
- `crab_seeds`: modelled on blue crab footage (lateral swimming). The rear
  paddles are raised (beside the body up to over the back) and scull, with
  the proximal sweep and the feather in quadrature and the two sides offset
  in phase. The walking legs hold streamlined offsets with small
  oscillations.

## Pipeline (`gait_chain.py`)

1. **search**: score `--seeds` strokes (families joined with `+` in
   `--seed_style`), grow the best `--starts` with a few LM iterations each,
   then polish the best through an elitist ES and LM.
2. **ladder**: target speeds from `--speed_start`, multiplied by
   `--speed_growth` per rung. Each rung is an LM solve warm-started from the
   previous rung. A missed rung is retried for as long as each retry gains
   `--min_gain`. The last attempt, if faster and straight, is kept as `max`.

   LM suits target-speed residuals. A constant "go faster" pull stalls it
   (its damping climbs), hence the ladder.
3. **dts**: the fastest gait (or every rung, with `--refine_all`) is
   re-polished at 1 ms.
4. **check**: the final gaits are also rolled out at the planner's 20 ms
   timestep (`plan_dt_check` in `summary.json`).

Steps after the search start from the previous gait's steady limit-cycle
state, so 2 warm cycles suffice. A gait can, however, settle differently from
rest, as the free-tilt sideways gait does. `--from_rest` scores every step
from rest instead, the way the MJPC task starts.

Reruns resume: finished steps are reloaded. Each step writes:
- `stepK_<name>.npz`: theta, metrics, the limit cycle at 200 phases, and the
  command half-ranges
- `_controls.npy`
- `_history.json`
- with `MUJOCO_GL=osmesa`, an `.mp4` and `_plots.png`

```sh
python3.12 -m venv venv && venv/bin/pip install mujoco==3.2.3 numpy imageio imageio-ffmpeg matplotlib
cd mjpc/tasks/paddle_crab_swimming/analysis
MUJOCO_GL=osmesa python gait_chain.py --direction 0 -1 0 --out runs/ladder_mY
MUJOCO_GL=osmesa python gait_chain.py --direction 1 0 0 --ctrl_limit 3.14159 \
    --weights tilt=0 --seed_style crab+wide --seeds 1536 --speed_start 0.1 --out runs/free_pX
python export_gaits.py paddles_lead=runs/ladder_mY/step10_max_dt0.001.npz ... \
    --mjpc ../gait_reference.xml --package export_v1
MUJOCO_GL=osmesa python render_gait.py runs/ladder_mY/step10_max_dt0.001.npz
```

MuJoCo 3.2.3 matches the MuJoCo commit the C++ build pins
(`MUJOCO_MPC_MUJOCO_GIT_TAG`). It has no wheel for Python 3.13, hence the
3.12 venv.

## Results

Speeds are at the 1 ms simulation timestep, from a steady state unless noted.
For comparison, the original emergent MPC stroke swims 0.04–0.05 m/s.

| Gait (run) | Direction | Commands | Speed (m/s) | Period (s) | Notes |
|---|---|---|---|---|---|
| **paddles_lead** (`ladder_mY` step 10) | −Y, paddles leading | task ranges | **0.112** | 1.10 | straight from rest; 0.118 in the planner's 20 ms model |
| paddles_trail (`ladder_pY` step 8) | +Y | task ranges | 0.087 | 1.23 | |
| side (`ladder_pX` step 4) | +X, sideways | task ranges | 0.073 | 0.73 | |
| crab_side (`free_pX3` step 3) | +X | ±π, tilt free | 0.087 from rest | 1.18 | paddles held over the back (rear mid joints 2–3.1 rad); heading wanders open loop over 10–20 s |
| `free_mY` | −Y | ±π, tilt free | 0.088 | 2.27 | slow long stroke found by the search |

Findings:
- **Seeds near the zero pose were the speed cap.** One random wide-offset
  seed in −Y swims 0.087 m/s unoptimized. Crab-style seeds give the best
  sideways starts (0.053 m/s raw, against 0.02–0.03 for random seeds).
- With these seeds and LM's local reach, −Y beat sideways. The crab-like
  sideways gait is fast, but its heading is not stable open loop.

### In MJPC

`export_gaits.py --mjpc ../gait_reference.xml` writes the gait library that
`task.xml` includes. The task's **Gait** drop-down (`residual_select_Gait`)
picks a primitive; **Off** is the original emergent stroke.

The selected primitive is fed forward through the servo callback, so the
servo setpoint is `clamp(ctrl + u_gait(t), ±gait limit)`, both in the
simulation and in every planner rollout:
- The planner's `ctrl` is a correction on top of the primitive.
- `Effort` prices the deviation from the primitive.
- Forward and Lateral use the gait's swim direction.
- The emergent-rhythm terms are off while a gait runs.
- The gait clock restarts when the selection changes; `GaitPhase` shifts it.

`cost_sweep` (an MJPC target) runs the task headless:

```sh
build/bin/cost_sweep --seconds 20 Gait=Off Gait=paddles_lead     # closed loop
build/bin/cost_sweep --open_loop --seconds 20 Gait=paddles_lead  # primitive alone
build/bin/cost_sweep --numeric sampling_exploration=0.01 Gait=paddles_lead
```

MJPC's open-loop playback matches the optimizer for all four exported gaits,
to the millimetre along the swim direction.

Closed loop, 20 s from rest (single runs):

| | Off | paddles_lead | side | crab_side |
|---|---|---|---|---|
| exploration 0.3 (task default) | 0.038–0.041 m/s | 0.108 m/s, identical to open loop: the planner never beats doing nothing | 0.069, identical to open loop | 0.055, yaw +1.17 rad |
| exploration 0.01 | | **0.111 m/s, 1 cm lateral, yaw −0.03 rad**, but depth +0.33 m | | 0.061, yaw −1.58 rad |

Gait mode wants small exploration (about 0.01–0.03). The task default of 0.3
is tuned for the emergent stroke. Heading control of `crab_side` is still
open: it may need a higher Heading weight, or a gait made heading-stable
offline.
