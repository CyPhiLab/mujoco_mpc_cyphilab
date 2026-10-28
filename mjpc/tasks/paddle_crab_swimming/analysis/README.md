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

### Round 2: heading, free posture and design levers

- **crab_side v2** (`crab_heading`). The crab gait was re-polished from rest
  over a 14 s window, with yaw drift and heading weighted up. It swims
  0.087 m/s and holds within ±7° of heading for about 15 s from rest (the
  first version wandered from about 8 s). Beyond that it still drifts: −33°
  by 27 s, and over 60 s it slides 1.1 m sideways and climbs 1.7 m open loop.
  The open-loop heading is slowly unstable and needs feedback.
- **paddles_lead with free posture** (`lead_free`), with ±π commands and
  steady tilt free, stays at 0.1126 m/s. It is a local optimum.
- **Design levers** (`design_audit.py`, `crab_model.DESIGN_DEFAULTS`). The
  audit of `paddles_lead` found:
  - Thrust: all of it comes from the paddle legs (+0.49 N) and is spent on
    body drag (−0.50 N).
  - Walking legs: they do much work (mid joints about 5 J per cycle) for no
    net force.
  - Paddle-lift servos: they sit at their torque limit 35–41% of the time.
  - Setpoint filter: it shrinks strokes by 10–30%.
  - Plate: the flat plate is most of the body drag in MuJoCo's ellipsoid
    model. Its slender-drag term scales with the 0.3 × 0.4 m planform even
    when the plate moves edge-on, which may overstate a real plate.

  Each variant was re-optimized from `paddles_lead` (2 ms, same ladder
  settings):

  | Variant (actuators fixed) | Speed (m/s) | vs. baseline |
  |---|---|---|
  | baseline (unchanged design) | 0.116 | — |
  | body drag × 0.5 | **0.160** | **+38%** |
  | paddle area × 1.5 | 0.151 | +30% |
  | setpoint filter tau 0.04 s | 0.113 | none |
  | (paddle servo torque × 2, stopped early; actuators fixed for now) | 0.146 | +26% |

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
is tuned for the emergent stroke.

Tuning (`mjpc_tuning/`, 3 repeats of 20–30 s, plus 60 s runs):
- **The 20 ms planning model mispredicts the stiff servos.** At exploration
  0.03 it steers `paddles_lead` about 1 rad off heading. With a 5 ms model
  (`--agent_dt 0.005`) that drops to 0.11 rad.
- **Steering weights ×4** (Heading, Lateral, Depth) change little.
- **Over 60 s, closed loop does not reliably beat open loop** for crab_side
  v2 (3.74 m forward, 1.06 m lateral, +1.69 m depth open loop). With the
  20 ms planner, one run did better (4.43 m, 0.28 m, +1.13 m) and one failed
  (+2.97 m depth). With the 5 ms planner it is roughly open loop, and depth
  is not corrected.

Small random corrections to 18 raw servo commands cannot steer, and large
ones break the stroke. A low-dimensional steering interface, such as
left/right paddle amplitude for yaw and paddle mean angle for depth, is the
likely next step.

## Kinematic swimming reference (`kinematic_ref.py`, `crab_reference.json`)

This is a pure kinematic animation keyframed by eye against blue crab
footage. There are two gaits, each with its own paddle stroke:

- **`sideways`:** two clips, 000106 (front, slightly below) and 000014
  (behind and below). The leading paddle strokes up and over the back, and
  the trailing one, held up and out, strokes out of phase with it.
- **`paddles_first`:** two clips, 000094 (side) and 000054 (above). The crab
  swims toward its own back, like the optimized `paddles_lead` gait. Both
  paddles point back and run the same stroke, the far one about 0.6 of a
  stroke behind the near one.

Each gait is therefore checked from two viewpoints.

The paddles slice through the water continuously rather than flapping in
one plane:

- **Tip loop:** the base (proximal) swing is timed against the raise (mid),
  so each tip carves a loop. In paddles first the power stroke sweeps over
  the back and in toward the midline, and the return sweeps out to the side
  and down.
- **Blade angle:** the blade angle is given relative to the tip's own motion
  (`slice`: 0 broadside, 90 edge-first). The feather joint is solved as the
  smoothest periodic path that holds that angle, weighted by blade speed.
  Through a reversal the blade turns like an oar instead of spinning.

`paddle_paths.png` plots the loops and the blade angles.

Footage is never mirrored. When the crab swims toward its left, the robot is
mirrored instead.

```
python kinematic_ref.py crab_reference.json --out kinematic/v9 \
    --video 000106=PATH_000106.mp4 --video 000014=PATH_000014.mp4 \
    --video 000094=PATH_000094.mp4 --video 000054=PATH_000054.mp4
```

The command writes the following to the output directory:

- **`robot_<mode>.mp4`:** one per mode.
- **`compare_<clip>.mp4`:** raw footage, then the robot from the clip's
  camera, then the robot from above and behind, synced by stroke phase.
- **`sheet_<clip>.png`:** a single stroke shown at 8 phases.
- **`mode_<mode>.png`:** every clip of a mode, each over the robot seen from
  that clip's camera, at the same 8 stroke phases.
- **`paddle_paths.png`:** each gait's paddle paths in the body frame, and
  the blade's angle to its own motion.
- **`reference.npz`:** qpos for each mode over one stroke.

In the renders, orange legs are the robot's right side and blue legs its
left; the darker shade marks the paddles.

## Trajopt against the kinematic references

`gait_chain.py --reference crab_reference.json --mode MODE` tracks a gait
from `kinematic_ref.py`. It adds a residual for the joint angles over the
last cycle minus the reference's at the same phase (`--weights track=W`) and
starts from a Fourier fit of the reference, led by the servo lag. An
imitate step first solves for tracking only, then the speed step pulls
toward a target speed with tracking kept. `--fix_period T` holds the stroke
period.

With the period free, tracking-only solves drift to 2-3 s strokes, because
slow strokes are the ones the servos can follow. So the runs below hold the
period fixed.

| gait, actuators | T (s) | imitate v (m/s) | tracking + speed v (m/s) | joint RMS from reference (rad) |
|---|---|---|---|---|
| paddles_first, XW430 | 0.4 / 0.6 / 0.8 / 1.0 / 1.2 | 0.040 / 0.050 / 0.051 / 0.044 / 0.044 | 0.087 / 0.085 / 0.081 / 0.073 / 0.070 | 0.30 / 0.27 / 0.23 / 0.20 / 0.18 |
| sideways, XW430 | 0.4 / 0.6 / 0.8 / 1.0 / 1.2 | 0.005 / 0.005 / 0.002 / 0.007 / 0.004 | 0.038 / 0.048 / 0.056 / 0.056 / 0.051 | 0.21 / 0.18 / 0.17 / 0.15 / 0.14 |
| paddles_first, paddle actuators 2x torque, 12x speed | 0.4 / 0.5 / 0.6 | 0.120 / 0.070 / 0.055 | **0.184** / 0.176 / 0.176 | 0.06 / 0.06 / 0.06 |
| sideways, same | 0.4 / 0.5 / 0.6 | 0.007 / -0.018 / 0.007 | 0.159 / 0.156 / 0.150 | 0.08 / 0.08 / 0.08 |

(`runs/sweep_*`, `runs/design05_*`, `runs/sweep_summary.png`,
`runs/design05_summary.png`.) For comparison, the best gait found without
a reference is `paddles_lead` at 0.1125 m/s with a 1.1 s stroke.

**Why the robot can't stroke like the crab.** `stroke_dynamics.py` runs
inverse dynamics on the reference stroke, and the result is that the robot
is speed and power limited, not inertia limited.

- **The servo's speed limit dominates.** The robots use DYNAMIXEL
  XW430-T333-R servos: 3.1 N m stall and 3.25 rad/s no load at 12 V. That
  torque-speed line is exactly the model's 1 N m s/rad joint damping, and it
  makes up about 90% of the torque needed at every stroke period.
- **The crab-rate stroke needs far more power.** At 0.3 s each paddle joint
  needs 10-80 W mean and 11-63 rad/s. The servo's peak output is about
  2.5 W.
- **Lighter paddle legs hardly help.** A 4x lighter leg changes the load by
  under 1%.
- **Added mass is not modelled.** MuJoCo's fluid model has none; a paddle
  blade's added mass in water, about 1 kg, would add more inertia than the
  motors do.

**What a 0.5 s stroke takes.** Paddle actuators of 6.2 N m stall and about
370 rpm no load (a roughly 60 W, small brushless class) leave the 0.5 s
reference stroke short of torque only 2% of the time (`paddle_torque=2,
paddle_damping=0.1667`).

**With those actuators:**

- **Paddles first tracks the reference to about 2 deg** and swims
  0.07-0.12 m/s on imitation alone.
- **The speed step reaches 0.18 m/s** while staying about 3.6 deg from the
  reference.
- **The sideways reference swims backward when tracked closely.** Most of
  the fix is in the leading paddle's raise (`runs/design05_changes.png`): its
  power stroke peaks at phase about 0.25, not the reference's 0.36. That
  matches clip 000014.
