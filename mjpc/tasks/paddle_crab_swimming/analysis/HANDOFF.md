# Paddle crab: reference gaits and optimized gaits (handoff)

This covers how to regenerate three things:

1. the kinematic crab references, rendered next to the real crab footage;
2. the best gaits optimized against those references on the robot's
   XW430 servos;
3. the same with faster, stronger paddle actuators.

Everything lives in `mjpc/tasks/paddle_crab_swimming/analysis/` on branch
`updated_paddle`. `README.md` in the same folder has the longer history,
including the earlier gait search without references.

## Setup

- **Run from `analysis/`.** The scripts import each other as siblings
  (`import crab_model`, `gait_opt`, ...).
- **Python environment.** Python 3.12 with `mujoco==3.2.3`, `numpy`,
  `matplotlib`, `imageio` with the `pyav` plugin and `Pillow`. The session
  used a venv at `/home/user/venv312`.
- **Headless rendering.** Use `MUJOCO_GL=osmesa`; EGL failed in the
  container.
- **The model.** `crab_model.load_model()` runs
  `../patch_paddle_crab_robot.py` on `../robot_xdomain.xml`, exactly as the
  MJPC build does. It swaps the MJPC servo callback for MuJoCo's built-in
  first-order `filter` dynamics (tau 0.08 s, identical behaviour) and drops
  the floor. Contacts are off and gravity is 0 (the robot floats).
- **The crab footage is not in the repo.** These are 200 fps FASTEC clips
  the lab shared, and you need the files themselves to make the comparison
  renders:

  | clip | file | gait |
  |---|---|---|
  | `000106` | `FASTEC-TS5-67E_2026-07-15_000106_10bit_small.mp4` | sideways |
  | `000014` | `FASTEC-TS5-67E_2026-07-16_000014_10bit_small.mp4` | sideways |
  | `000094` | `FASTEC-TS5-67E_2026-07-15_000094_10bit_small.mp4` | paddles first |
  | `000054` | `FASTEC-TS5-67E_2026-07-16_000054_10bit_small.mp4` | paddles first |

  The clip views:
  - **000106:** front view, slightly below. The crab swims toward its own
    left, which is screen right.
  - **000014:** from behind and below. The crab swims toward its own right,
    which is screen right.
  - **000094:** side view, anterior on screen left. The crab swims toward
    its own back, which is screen right.
  - **000054:** from above, anterior on screen left. The crab swims toward
    its own back.

## 1. Kinematic references next to the crab

- **Files:**
  - **`crab_reference.json`:** the reference (current version v9).
  - **`kinematic_ref.py`:** the animation, comparison renders and paddle-path
    plots.
- **The two gaits.** The JSON has one entry per gait, `sideways` and
  `paddles_first`. Each has its own paddle `stroke` (keyframed channels), a
  `joints` table and a `base`. The base is the swim direction, speed, tilt
  and heave.
  - **Joint entries.** A joint entry is one of: its own keys, a channel of
    the stroke, a mirror of another joint, or `slice`. A `slice` entry sets a
    paddle blade by its angle to its own motion (0 broadside, 90
    edge-first). The feather angle is then solved as the smoothest periodic
    path that holds it. `kinematic_ref.py`'s docstring has the full format
    and the joint sign conventions, which were checked with forward
    kinematics.
- **Clip settings.** `footage` maps each clip to its gait, frame range,
  stroke period, the frame where stroke phase 0 falls, the camera, crop and
  `mirror`.
  - **Footage is never flipped.** When the crab swims toward its own left
    (000106), the robot is mirrored instead (`"mirror": true`).
- **Regenerate:**

  ```
  MUJOCO_GL=osmesa python kinematic_ref.py crab_reference.json --out kinematic/v9 \
      --video 000106=PATH/..._000106_10bit_small.mp4 --video 000014=PATH/..._000014_10bit_small.mp4 \
      --video 000094=PATH/..._000094_10bit_small.mp4 --video 000054=PATH/..._000054_10bit_small.mp4
  ```

  This takes about 4 minutes and writes the following to `kinematic/v9/`
  (already committed there):
  - **`compare_<clip>.mp4`:** raw footage, then the robot from the clip's
    camera, then the robot from above and behind.
  - **`mode_<gait>.png`:** both clips of a gait at the same 8 stroke phases.
  - **`sheet_<clip>.png`:** one stroke at 8 phases for a single clip.
  - **`paddle_paths.png`:** the paddle-tip loops and blade angles.
  - **`robot_<gait>.mp4`:** the robot alone.
  - **`reference.npz`:** `qpos_<gait>` at 200 phases.
- **Colours.** Orange is the robot's right legs, blue its left, and the
  darker shade is the paddles.
- **Without footage.** Omit `--video` to get just the robot videos, paths
  and npz.

## 2. and 3. Best optimized gaits

These were optimized against the references with a fixed stroke period
(`gait_chain.py --reference ... --fix_period T`). Each run has an
`imitate` step (track the reference only) and then a `v0.2` step (track
the reference plus a pull toward speed). The best gaits are the `v0.2`
steps.

| what | file | stroke | speed (m/s) | joint RMS from reference |
|---|---|---|---|---|
| XW430, paddles first | `runs/sweep_paddles_first_T0.4/step2_v0.2_dt0.002.npz` | 0.4 s | 0.087 | 0.30 rad |
| XW430, sideways | `runs/sweep_sideways_T0.8/step2_v0.2_dt0.002.npz` | 0.8 s | 0.056 (0.061 from rest) | 0.17 rad |
| upgraded paddle actuators, paddles first | `runs/design05_paddles_first_T0.4/step2_v0.2_dt0.002.npz` | 0.4 s | 0.184 | 0.06 rad |
| upgraded paddle actuators, sideways | `runs/design05_sideways_T0.4/step2_v0.2_dt0.002.npz` | 0.4 s | 0.159 | 0.08 rad |

All four were re-checked at the 1 ms simulation timestep, both from rest
and from the steady state; speeds match to within about 1%. The sideways
gaits drift slightly in heading, by 0.01-0.02 rad/s.

- **More data.** Every period that was run is in `runs/sweep_<gait>_T*` and
  `runs/design05_<gait>_T*`, with summaries in `runs/sweep_summary.png/json`
  and `runs/design05_summary.png/json`. Each step has an `.mp4` and
  `_plots.png` next to its npz.
- **Best gait without any reference** (for comparison): the XW430
  `paddles_lead` gait, 0.1125 m/s with a 1.1 s stroke, in
  `runs/ladder_mY/step10_max_dt0.001.npz`. It is also exported to MJPC as
  `gait_reference.xml` (gait `paddles_lead`).

### What "without the speed limitation" means here

- **Servo model.** The servos are DYNAMIXEL XW430-T333-R: 3.1 N m stall and
  31 rpm (3.25 rad/s) no load at 12 V. That torque-speed line is modelled as
  joint damping of 1 N m s/rad (`robot_xdomain.xml`) with a ±3.1 N m force
  limit.
- **The upgraded variant is not unlimited.** It changes only the six paddle
  joints: `--design paddle_torque=2,paddle_damping=0.1667`. That is 6.2 N m
  stall and 12x the no-load speed (about 39 rad/s, roughly 60 W peak, a
  small-BLDC class). It was sized with `stroke_dynamics.py` to follow the
  reference at a 0.5 s stroke. The walking legs keep XW430s.
- **Design variants.** `crab_model.DESIGN_DEFAULTS` lists them all: `torque`,
  `paddle_torque`, `kp`, `body_drag`, `paddle_area`, `damping`,
  `paddle_damping`, `paddle_mass`.
  - **For truly speed-unlimited servos** use `paddle_damping=0` (or
    `damping=0`). Torque is then still capped at `3.1 x torque`.

### Replay, render, extract commands

```
MUJOCO_GL=osmesa python render_gait.py runs/design05_paddles_first_T0.4/step2_v0.2_dt0.002.npz --dt 0.001
```

This writes the `.mp4` and `_plots.png` next to the npz, starting from rest.
It reads the design and command range from the npz.

**What's in a step npz:**

| field | contents |
|---|---|
| `theta` | the gait: `[log T, c(18), a(18,k), b(18,k)]`; `k` = 5 for these |
| `half` | command half-ranges (π here; `--ctrl_limit 3.1416`) |
| `design` | JSON string of the design overrides |
| `tau` | servo filter time constant |
| `direction` | world swim direction |
| `x0` | start state used |
| `metrics` | JSON string of the step's metrics |
| `phase`, `qpos`, `qvel`, `act`, `ctrl` | one limit cycle, 200 samples |
| `state` | full state at phase 0 |

**The commands.** Per actuator:

```
u_i(t) = half_i * clip(c_i + sum_k a_ik cos(2 pi k t/T) + b_ik sin(2 pi k t/T), -1, 1)
```

- **Command time series:** `gait_opt.controls(gait_opt.Layout(k), theta, half, n, dt)`.
- **Command order:** `crab_model.ACTUATORS`, which matches `crab_model.JOINTS`
  (psfr msfr dsfr pscr mscr dscr psbr msbr dsbr psfl ...).
- **Commands are servo setpoints.** They pass through the first-order filter
  (tau 0.08 s) and a position servo with kp 50.
- **Robot frame.** Anterior is +Y, right is +X, and the paddles are the rear
  legs. Paddles first swims −Y; sideways swims +X (right legs leading).

`export_gaits.py` turns step npz files into MJPC's `gait_reference.xml` and
a standalone package. It hasn't been run on these new gaits yet.

### How they were produced (exact commands)

Both sweeps share these flags:

```
common="--reference crab_reference.json --k 5 --ctrl_limit 3.1416 --lead 0.07 --dts 0.002 --speeds 0.2 \
  --patience 0 --es_iters 0 --lm_iters 15 --imitate_iters 20 --processes 2 \
  --weights track=30,saturation=0.002,effort=0.3,speed=0.1"
```

- **XW430 sweep:** for each `mode` in `paddles_first sideways` and each `T` in
  `0.4 0.6 0.8 1.0 1.2`:

  ```
  python gait_chain.py $common --mode $mode --fix_period $T --out runs/sweep_${mode}_T$T
  ```

- **Upgraded actuators:** add `--design paddle_torque=2,paddle_damping=0.1667`,
  use `T` in `0.4 0.5 0.6` and output to `runs/design05_${mode}_T$T`.
- **Run time.** Each run takes 10-30 min on 2 cores.
- **Resuming.** Reruns resume: finished steps are reloaded, not redone.

**Why the weights are set this way:**

- **`speed=0.1` toward an unreachable 0.2 m/s target.** This is a gentle,
  constant pull toward speed that Gauss-Newton handles well. It makes about
  0.01 m/s worth about 0.02 rad of tracking error.
- **`track=30`.** Tracking has to outweigh the other residuals.
- **`saturation=0.002`.** With the XW430s the reference asks for up to 10x
  the servo torque, so the default saturation penalty would swamp everything.
  Saturating is physical anyway: it just shows up as tracking error.
- **The period is fixed** because, left free, tracking-only solves drift to
  2-3 s strokes (slow strokes are easy to track). Those earlier free-period
  runs are kept in `runs/ref_*`.

## Supporting analysis

- **`stroke_dynamics.py crab_reference.json --out kinematic/v9/dynamics`.**
  Inverse dynamics of the reference strokes. It gives the torque, speed and
  power per paddle joint against stroke period and design variant
  (`stroke_dynamics.json/png`).
- **The headline:** the XW430's speed limit (damping) is about 90% of the
  load at every period. Inertia (motor mass) only matters once the servos
  are much faster. A crab-rate (0.3 s) stroke needs 10-80 W per paddle joint
  against about 2.5 W available.
- **`runs/design05_changes.png`.** Reference vs imitation vs optimized
  paddle joint angles for the upgraded design.

## Known issues and caveats

- **The sideways reference is slightly wrong.** Tracked closely, it swims
  backward (−0.018 m/s at 0.5 s with the upgraded actuators). The optimizer
  fixes it with small changes, mostly to the leading paddle's raise (`msbr`):
  its power stroke should peak over the back at phase about 0.25, not the
  keyed 0.36. Clip 000014 agrees: the paddle is about 45 deg over the back
  around frames 260-264. This hasn't been fixed in `crab_reference.json` yet.
- **The sideways trailing paddle's blade twist** (`dsbl`, a `slice` joint)
  is very fast through its reversal, up to about 170 rad/s at a 0.3 s stroke.
  No servo tracks it, and it should be smoothed in the reference.
- **No added mass in the fluid model.** MuJoCo's ellipsoid fluid model has
  none. A paddle blade's added mass is about 1 kg, against 0.34 kg for the
  blade segment, so loads at fast strokes are underestimated. Treat the
  upgraded actuator size as a lower bound.
- **Optimization timestep.** The optimized gaits were optimized at 2 ms. All
  four best ones were re-checked at 1 ms (see the table).
- **Sharp blade twists.** Joint RMS from the reference is dominated by the
  blade (distal) joints wherever the servos can't follow the twists.
