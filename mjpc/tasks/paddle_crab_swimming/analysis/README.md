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

## Round 3: a primitive library for every direction and turning

The same pipeline now produces one primitive per motion a controller needs:
8 horizontal swim directions, turning on the spot both ways, and swimming up
and down. Everything uses the robot as built: XW430-T333-R servos at 12 V and
the task's command ranges. `library.txt` lists the library, and
`../gait_reference.xml` and `export_v3/` hold its export.

### What changed in the pipeline

- **Turning primitives.**
  - `gait_opt.Problem(yaw_rate=W)` targets a yaw rate. While turning, the
    mean-heading residual is off, and the tilt is measured in the heading
    frame that turns with the body.
  - `gait_chain.py --yaw_ladder W1 W2 ...` climbs a ladder of yaw-rate
    targets, holding translation at `--turn_speed` (default 0). Rungs are
    named `w<W>`. Each step's start state has its yaw removed.
  - `--yaw_rate W` instead holds a yaw target through a speed ladder (arcs).
- **Vertical primitives.** `--direction 0 0 ±1` is supported. Drift is then
  measured along world X ("lateral") and world ±Y ("depth" slot).
- **Mirror images.** The robot is mirror-symmetric to within its CAD, so
  `mirror_primitives.py` makes −X, the −X diagonals and turning right from
  their counterparts. The mirrors swim the same speed with the drift flipped
  (checked by simulation; mirrored states agree to 0.4 mm over 3 s). Only one
  side of each pair needs an optimization.
- **Library seeds.** `--seed_style library --seed_files A.npz ...` adds known
  gaits and their mirror images to the search batch, and `splice` adds one
  gait's right legs with another's left. Both diagonal searches started from
  a library gait.
- **Restart-safe solvers.** `es`/`lm` save a checkpoint after every iteration,
  and every search stage and ladder step has its own. Reruns continue
  exactly where they stopped; interrupted-and-resumed runs give the same
  result as uninterrupted ones. The container restarted twice during these
  runs.
- **`primitive_report.py`** scores a library the way MJPC runs it:
  - the steady limit cycle (12 cycles from rest);
  - cycles 5–6 from rest;
  - the 20 ms planner model;
  - a 20 s open-loop path.

  It writes `runs/primitives_report.{md,json}` and `_paths.png`.
- **`run_primitives.sh`** holds the exact commands.

### The library

Steady limit cycle at 1 ms (`runs/primitives_report.md` has every column).
The heading change is over 20 s open loop from rest.

| primitive | swims | stroke (s) | speed | drift (lateral) | yaw | sat. | 20 ms planner | 20 s open loop | run |
|---|---|---|---|---|---|---|---|---|---|
| paddles_lead (K = 5) | −Y | 1.10 | 0.116 m/s | +1 mm/s | 0.000 rad/s | 0.07 | 0.120 | 2.23 m, +0.03 rad | `prim_k5_mY` |
| paddles_cruise (walking legs held) | −Y | 1.09 | 0.091 | +1 (sinks 18 mm/s) | +0.001 | 0.01 | 0.093 | 1.75 m, +0.01 rad | `prim_hold_mY` |
| paddles_trail | +Y | 1.23 | 0.086 | +2 | −0.010 | 0.09 | 0.093 | 1.66 m, −0.19 rad | `ladder_pY` |
| side / side_left | ±X | 0.73 | 0.073 | ∓2 | ±0.003 | 0.06 | 0.081 | 1.37 m, ±0.12 rad | `ladder_pX`, `prim_mX` |
| diag_right_lead / diag_left_lead | ±X−Y | 0.74 | 0.080 | ±4 | ∓0.012 | 0.06 | 0.083 | 1.53 m, ∓0.09 rad | `prim_pXmY`, `prim_mXmY` |
| diag_right_trail / diag_left_trail | ±X+Y | 1.22 | 0.070 | ±2 | ±0.004 | 0.04 | 0.076 | 1.35 m, ±0.08 rad | `prim_pXpY`, `prim_mXpY` |
| turn_left / turn_right | on the spot | 2.26 | ±0.373 rad/s (21°/s) | ∓4 | — | 0.03 | ±0.374 rad/s | ±6.97 rad, 2 cm | `prim_turnL`, `prim_turnR` |
| up | +Z | 1.71 | 0.071 | 23 (−Y) | −0.001 | 0.03 | 0.077 | climbs 1.35 m, 0.47 m −Y | `prim_up` |
| down | −Z | 2.15 | 0.072 | 21 (−Y) | −0.004 | 0.02 | 0.067 | sinks 1.13 m, 0.75 m −Y | `prim_down` |
| crab_side (round 2) | +X, ±π, tilt free | 1.21 | 0.084 | 23 | −0.034 | 0.03 | 0.088 | 1.46 m, −0.20 rad | `crab_heading` |

Notes:

- **Diagonals.** The paddles-leading diagonals (0.080 m/s) are faster than
  pure ±X (0.073), and the paddles-trailing ones (0.070) about as fast.
  Leading with the paddles wins on the diagonal as it does on ±Y. From rest the leading
  diagonals start out heading more −Y and bend onto the diagonal within a
  few strokes (`_paths.png`).
- **Turning.**
  - With the default yaw weight (1, sized for drift), the turning ladder
    stalls short of 0.1 rad/s (`prim_turnL_w1`). With `--weights yaw=8` (a
    yaw target weighted like a speed target) it climbs to 0.38 rad/s at
    2 ms. The 1 ms polish settles at 0.373.
  - The turn uses a slow stroke (2.3 s).
- **Vertical.**
  - Up and down hold their attitude only slowly: their tilt keeps settling
    for tens of seconds. They slide about 2 cm/s in −Y open loop, which the
    steady-start ladder (scored on cycles 3–4) does not see.
  - A re-polish of down scored from rest after 10 cycles (`prim_down_rest`,
    stopped) was worse over 30 s. This is an attitude-stability issue, not a
    scoring-window one.
  - **In MJPC closed loop the drift goes away** (below), so they are usable
    as primitives under feedback.
- **+X.** The library re-search (`prim_pX`, 0.069 m/s) did not beat
  `ladder_pX` (0.073), which stays the side primitive.

### In MJPC

The gait header gained a yaw rate (`period, K, speed, direction(3),
yaw_rate, c, a, b, limit`), and the old layout is still read.

With a gait running, the residuals change as follows:

- **Turning primitives:** the Heading target turns at the gait's yaw rate.
- **Turning on the spot:** the Forward reward is dropped.
- **Vertical primitives:** Lateral holds world X and Depth holds world Y
  (for horizontal ones Depth is unchanged).

`cost_sweep` builds with clang. GCC's LTO objects don't link with lld.
`mjpc/tasks/CMakeLists.txt` also copies two models from sibling repos
(`../ptero_sim`, `../spirob_mujoco`) that a build needs present.

- **Open loop (`--open_loop`, 20 s):** every primitive matches the Python
  rollout to the millimetre (paddles_lead, turn_left/right, down,
  diag_right_lead, side_left checked).
- **Closed loop (`sampling_exploration=0.01`, single runs):**

  | gait | open loop | closed loop |
  |---|---|---|
  | paddles_lead, 20 s | 0.108 m/s | 0.111 m/s, 1 cm lateral (as in round 2) |
  | diag_right_lead, 20 s | 0.077 m/s | 0.078 m/s |
  | turn_left, 20 s | 6.97 rad | 6.87 rad, 11 cm translation |
  | turn_right, 30 s | | −11.1 rad |
  | down, 30 s | sinks 1.84 m, 0.98 m −Y | **sinks 2.04 m, 3 cm horizontal** |
  | up, 30 s | climbs 2.06 m, 0.69 m −Y | climbs 2.13 m, 0.28 m −Y |

### Supply voltage: 14.8 V (realistic, same servos)

At 14.8 V the XW430 gives 3.6 N m stall and 39 rpm
(`--design torque=1.16,damping=0.88`):

| run | gait | speed (1 ms) | stroke | 20 ms planner |
|---|---|---|---|---|
| `v148_mY` | paddles_lead re-optimized at 14.8 V | **0.150 m/s (+34%)** | 1.05 s | 0.157 |
| `v12_from148_mY` | 12 V re-optimized from that gait (control) | 0.1135 | 1.16 s | 0.117 |

The control shows that the 12 V gait was already at its optimum (+1%), so
the gain comes from the supply.

- **Power scaling alone would predict about +13%.** That's 1.46× peak power
  with drag going as v².
- **The rest comes from the higher no-load speed.** The paddle servos sit at
  their speed limit (`stroke_dynamics.py`), and raising that limit is what
  pays.
- **14.8 V gaits are not exported.** MJPC simulates 12 V servos;
  `export_gaits.py` now warns when a gait was made for a design variant.

### More harmonics (K = 5)

`prim_k5_mY` warm-starts paddles first from the K = 3 gait with `--k 5`
(199 parameters instead of 127). It swims 0.1161 m/s at 1 ms against
0.1125 (+3%), with a 0.120 m/s prediction from the 20 ms planner model. It
is also straighter: lateral drift is 1.3 mm/s against 8.1, and over 20 s
open loop it ends 2 cm off line with a 0.03 rad heading change (12 cm and
0.11 rad for K = 3). It replaces the K = 3 gait as `paddles_lead` in the
library. MJPC plays it back exactly (`cost_sweep --open_loop`: 2.23 m,
2 cm lateral).


### Walking legs: what they cost, and a cruise gait

`leg_audit.py STEP.npz` splits a gait's steady cycle by body part:

- fluid force and moment from the body, each walking leg and each paddle;
- passive drag at the mean posture (tow test);
- servo work per leg;
- the gait rerun with the walking legs held at their mean pose.

For paddles_lead (K = 3):

- **Drag.** The walking legs are 13% of the passive drag. The body is 72%
  and the paddles 16%, so posture alone could gain at most about 7% speed.
- **Work.** The walking legs do 62% of the servo work (38 of 61 J per
  stroke), for almost no net thrust: the front pair +0.06 N, the centre
  pair −0.05 N.
- **What their motion is for.** Holding them still drops the speed 21% and
  the body tilts 0.20 rad. They mostly keep the plate level.

The objective lets them move because speed (weight 4 at 5 mm/s) far
outweighs effort, and effort is RMS torque, not energy. Two experiments
followed:

- **`hold_mY`.** `--hold_legs R1,R2,L1,L2` holds the walking legs still
  with their posture free, and the paddles re-optimize around them.
- **`energy_mY`.** The `energy` weight puts cost of transport (positive
  servo work / m g distance) in the objective (weight 4). It ran down a
  ladder of speed targets from paddles_lead.

Steady state at 1 ms, with the energy points optimized at 2 ms:

| gait | speed (m/s) | cost of transport | servo work (J/stroke) | walking-leg share | tilt (rad) |
|---|---|---|---|---|---|
| paddles_lead K = 3 (sprint) | 0.112 | 4.52 | 61 | 62% | 0.05 |
| paddles_lead K = 5 (sprint) | 0.116 | 4.07 | 57 | 58% | 0.06 |
| energy-weighted, 0.11 target | 0.106 | 3.86 | 49 | 57% | 0.18 |
| energy-weighted, 0.10 target | 0.097 | 3.15 | 37 | 48% | 0.18 |
| energy-weighted, 0.09 target | 0.092 | 2.76 | 31 | 42% | 0.06 |
| energy-weighted, 0.08 target | 0.086 | 2.75 | 29 | 41% | 0.03 |
| **walking legs held (cruise)** | **0.091** | **1.48** | **16** | 0% | 0.14 |

Findings:

- **There are two regimes, like an animal's sprint and cruise gaits.**
  - Paddling with the walking legs still costs about a third of the
    sprint's energy per metre, at 79% of its speed.
  - At equal speed (0.091 vs 0.092 m/s), the cruise gait uses 54% of the
    energy-weighted all-legs gait's energy.
- **They are separate local optima.** Even with energy in the objective,
  descent from the sprint gait only shrinks the walking-leg motion (to 41%
  of the work). It never finds the paddle-only gait, which needs the legs
  held still to reach.
- **The cruise gait couldn't regain speed.** Its ladder stalled at 0.093
  m/s (target 0.099).
- **The cruise gait sinks open loop.** It descends 1.8 cm/s.
  - In MJPC closed loop (exploration 0.01, 20 s, one run) the sinking goes
    away: −0.04 m against −0.28 m open loop, at 0.093 m/s.
  - That run did take 0.29 rad of heading and drifted 16 cm sideways.
- **It's in the library** as `paddles_cruise`.

### Reproduce

The exact `ladder_mY` command (`runs/repro_mY`) reruns its search step
bit-identically: same theta and limit cycle. The saved gaits re-evaluate
bit-exactly too.

```sh
./run_primitives.sh chain1        # +X-Y, +X+Y, +X
./run_primitives.sh turnL_w1      # turning search (then:)
./run_primitives.sh chain2        # turning ladder (yaw weight 8), up
./run_primitives.sh chain3        # down
python mirror_primitives.py runs/prim_pXmY/step8_max_dt0.001.npz runs/prim_mXmY   # and mX, mXpY, turnR
python export_gaits.py $(grep -v '^#' library.txt) --mjpc ../gait_reference.xml --package export_v3
python primitive_report.py $(grep -v '^#' library.txt) --out runs/primitives_report
build/bin/cost_sweep --open_loop --seconds 20 Gait=turn_left Gait=down
```

## Robot v2 (the updated robot)

The collaborators' updated model is in `../robot_v2/` (`Robot.xml` plus its
meshes, committed verbatim). It is the robot from here on; v1
(`robot_xdomain.xml`) stays for MJPC until MJPC moves to MuJoCo 4.

What changed (see `crab_model.ROBOTS`):

| | v1 | v2 |
|---|---|---|
| body | flat plate, cylinder, ballast | chassis mesh, buoyancy foam |
| mass | 11.1 kg | 8.9 kg |
| drag at 0.1 m/s (−Y / +X / +Z) | 0.57 / 0.58 / 0.88 N | 0.70 / 0.79 / 1.01 N |
| servo housings | 0.34 kg | 0.096 kg |
| walking-leg feet | box | foot mesh |
| servos | kp 50 position servo + 0.08 s setpoint filter, joint damping 1 as the speed limit | MuJoCo `dcmotor`: XW430 at 12 V (3.1 N m stall, 3.25 rad/s no load) with its own PID position control |
| MuJoCo | 3.2.3 | ≥ 4 (`dcmotor`) |

How the pipeline loads v2 (`crab_model.load_v2`):

- **Joint damping is set to 0.** The file keeps damping 1 on every joint as
  well as the `dcmotor`, which already models the back-EMF. That counts the
  speed limit twice: the servos then reach 1.58 rad/s instead of the
  datasheet's 3.25 (measured with a step command). This should go back to
  the collaborators.
- **The task's command ranges, not the file's.** The file's ranges are
  ±1.6 / ±1.0 / ±0.5 rad, and the paddle feather would be ±0.5. The task's
  are ±1.2 / ±1.57 / ±1.05, with the paddle feather ±1.57.
- **No floor, contacts off, no setpoint filter.**
- **`input="position"` is rewritten to `"pos"`** when the installed MuJoCo
  needs it. The file is written for mjlab's newer MuJoCo; 4.0.1 spells it
  `pos`.
- **Actuator-force sensors are added.**

**Effort, saturation and work on v2.** The `dcmotor` reports the net torque
on the joint, which is small (0.1–0.5 N m: the fluid and inertial load). Its
speed limit is the supply voltage. On v2 these measures therefore use the
supply fraction u = torque / 3.1 N m + speed / 3.25 rad/s (1 at the voltage
limit; checked on step responses):

- effort is RMS(u);
- "saturated" is the time at |u| > 0.99;
- work is (u × 3.1 N m) × speed.

This is what v1's damping-inclusive servo torque measured, so v1 and v2
numbers compare.

Checks:

- **Timestep.** At 2 ms, speeds are within about 1% of 0.5 and 1 ms, but
  5 ms is off (drift). The schedule stays 2 ms, then 1 ms.
- **The 20 ms planner check is NaN on v2.** The `dcmotor` is unstable at a
  20 ms step; MJPC will need its own planner step for v2.
- **Mirror images** swim within about 2% of their source on v2 (0.1% on
  v1). `run_primitives_v2.sh mirrors` re-polishes them with 4 LM iterations.
- **v1 results** still re-score bit-identically (`CRAB_ROBOT=v1`, MuJoCo
  3.2.3).

The robot is chosen per run with `gait_chain.py --robot v1|v2` or
`CRAB_ROBOT`; the default is v2. Every step npz records its robot in a
`robot` field (files without it are v1), and the analysis tools follow it.
`--init` from a v1 gait works on v2: the commands are radians, and the first
step starts from rest. Speed rungs now count as met when the gait swims at
least the target (an overshoot used to count as a miss and pull the gait
back), and a missed rung's `max` is its best straight attempt, not its last.

The v1 primitives on v2, unchanged (2 ms, from rest):

| primitive | v1 speed | on v2 | note |
|---|---|---|---|
| paddles_lead (K = 5) | 0.116 | 0.129 | |
| paddles_cruise | 0.093 | 0.090 | |
| paddles_trail | 0.087 | 0.096 | yaw −0.02 rad/s |
| side | 0.073 | 0.072 | 49 mm/s lateral drift, yaw −0.05 rad/s |
| diag_right_lead | 0.080 | 0.094 | yaw −0.045 rad/s |
| diag_right_trail | 0.070 | 0.084 | |
| turn_left | 0.382 rad/s | 0.359 rad/s | |
| up | 0.074 | 0.052 | |
| down | 0.075 | 0.020 | |

### Running the v2 library (any machine)

```sh
git clone -b updated_paddle https://github.com/CyPhiLab/mujoco_mpc_cyphilab
cd mujoco_mpc_cyphilab/mjpc/tasks/paddle_crab_swimming/analysis
python3.12 -m venv ~/venv_mj4 && ~/venv_mj4/bin/pip install -r requirements_v2.txt
PROCS=8 MUJOCO_GL=egl nohup ./run_primitives_v2.sh all > v2_all.log 2>&1 &
```

- **`all`** runs two chains side by side, then the mirror images:
  - `chainA`: −Y, +X−Y, turn, up, cruise −Y;
  - `chainB`: +Y, +X, +X+Y, down.
- **`PROCS`** is the worker count per run, so about half the cores. Batches
  are capped at 32 rollouts, so beyond about 16 per run extra cores help
  less. For more parallelism, start the single runs (`./run_primitives_v2.sh
  NAME`) yourself.
- **`MUJOCO_GL`**: `egl` on a GPU machine, `osmesa` with libosmesa, or empty
  to skip the per-step videos.
- **Restarts.** A restarted run resumes from its last iteration; rerun the
  same command.
- **Outputs** go to `runs/v2_<name>/` (step npz, mp4, plots, summary.json)
  and `runs/primitives_queue.log`.

### More behaviours: rotations, attitude, arcs, a repertoire map

New targets (all off by default; existing gaits re-score bit-identically):

- **Rotation about any body axis.** `gait_chain.py --rate_axis x|y|z
  --rate_ladder W1 W2 ... --from_rest` ladders the mean body-frame angular
  rate about one axis (`gait_opt.Problem(body_rate=...)`). The other two
  rates are held at 0 (`--off_axis_tol`). Heading, tilt and the world
  attitude in the cycle residual are off while rotating; translation is held
  at `--turn_speed`. Reflecting x keeps a rate about body x and flips body y
  and z, so `mirror_primitives.py` turns a +y rotation into a −y one.
- **Attitude-held swims.** `--attitude AX AY` holds the body z axis at its
  direction after rotating about world x by AX, then world y by AY, instead
  of level. `--start_attitude` starts from rest at that attitude, e.g.
  `--attitude 3.14159 0 --start_attitude` swims upside down.
- **Arcs and helices.** `--yaw_rate W` with a speed ladder now measures speed
  and drift in the heading frame (turned by the mean heading change over the
  scored cycles), and each step starts with its yaw removed.
- **Attitude creep.** `--weights attitude_rate=W` adds the mean pitch and
  roll rate to the objective; rungs then also need both within
  `--yaw_drift_tol`. Held-leg gaits need it (`runs/v2_cruise_report_notes.md`).
- New metrics in every step: `body_rate`, `body_velocity` (body frame) and
  `velocity` (world). `primitive_report.py` adds the body rotation over the
  20 s path (`path.body_angle`).
- **`qd_map.py`**: MAP-Elites over gaits. It bins each gait by mean
  body-frame linear and angular velocity and keeps the most periodic gait per
  cell. `run` (resumable), `report` (projections, `cells.json`) and `export`
  (cells as step npz for `gait_chain.py --init` and `render_gait.py`).

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
