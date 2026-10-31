# TurtleCompliantSwimming

MJPC task for the sea-turtle robot with **compliant front flippers**: the current
`turtle_simulator` model (`mujoco_models/turtle_compliant.xml`, default blade stiffness
G = 0.2 MPa, 8 blade segments) swimming straight and level, starting from the
`turtle_simulator` reference gait.

The older `turtle_3dswimming` task (pre-graft rigid model, velocity actuators, target
tracking) is left untouched.

## Files

| File | What it is |
|---|---|
| `task.xml` | planner settings, residual parameters, cost weights, sensors |
| `turtle_compliant_swimming.{h,cc}` | residual and the reference-gait lookup |
| `turtle_compliant_mjpc.xml`, `assets/` | the robot, **generated** by `prepare_model.py` |
| `reference_gait.xml` | the reference gait table and `home` keyframe, **generated** by `make_reference_gait.py` |
| `turtle_trajopt.cc` | headless runner (`bin/turtle_trajopt`): closed-loop MPC or reference playback, CSV log, summary |
| `analyze_runs.py` | compares runs (table + plots) and extracts a phase-averaged gait from a run |

## Updating the model from turtle_simulator

Both generated files come from a `turtle_simulator` checkout. Re-run after any graft or
compliance change there (after `tools/compliant_flipper.py`), then rebuild:

```bash
cd mjpc/tasks/turtle_compliant_swimming
python prepare_model.py --turtle-sim ../../../../turtle_simulator
python make_reference_gait.py --turtle-sim ../../../../turtle_simulator
```

`prepare_model.py` changes the model in four ways (see its docstring):

* **All 10 actuators are joint-target servos** using `turtle_sim.py --servo`'s placeholder
  gains (kp = 20 N m/rad, kv = 1 N m s/rad, ±10 N m), rear hips included. The plain model's
  actuators are kp = 1 springs that `turtle_sim.py` wraps in an outer PD loop at 100 Hz,
  which the planner can't express through `ctrl`. turtle_simulator's `cad_exports/README.md`
  also recommends `--servo` for compliance studies.
* `ctrlrange` placeholders, because the CAD export has no joint limits: ±1.2 rad (joints 1, 3),
  ±1.6 rad (joint 2), ±1.05 rad (rear hips).
* Contacts disabled, as in `turtle_sim.py`.
* Sites `base_cog`, `head`, `tail` on the shell. Shell frame: −y forward, +z up.

`make_reference_gait.py` samples `turtle_ctrl`'s Cornelia trajectories exactly as
`turtle_sim.py --servo --controls 1 0 0 0` commands them (same `FRONT_JOINT_MAP`,
`SERVO_STROKE_SCALE`, time scaling and start phase): 128 samples over one 1.911 s stroke.
The rear hips are held at 0 (the simulator's ±5°, 1 Hz hip wiggle isn't periodic in the
stroke). Played open loop, the table reproduces `turtle_sim.py`'s motion on this model
(0.084 vs 0.089 m/s net, same path; the small gap is the rear hips).

## Cost

| Term | Residual | Default weight |
|---|---|---|
| Speed | `min(v − v_des, 0)`, COM velocity along the swim direction | 20 |
| Slip | COM velocity off the swim direction (sideways and vertical) | 10 |
| Heading | `forward × swim direction` | 2 |
| Upright | `up × world z` | 2 |
| AngVel | shell angular velocity | 0.2 |
| Gait | `ctrl − reference gait(phase)`, 10-dim | 1 |
| Power | `sqrt(Σ max(τ·q̇, 0))`, so the cost is 0.5·w·(positive mechanical power) | 0.01 |

Parameters: `Speed` (desired speed, m/s), `Heading` (swim direction yaw, 0 = world −y),
`Cadence` (stroke-rate multiplier on the reference) and `Phase` (offset, fraction of a stroke).
Gravity is off in the model (neutrally buoyant), so nothing but the controller keeps the shell level.

**The Gait weight is the knob between the reference and free optimization.** High (≈10):
MPC follows the reference gait and only stabilizes. Low (≈0.1): the reference only
regularizes. The reference is also the starting point of the plan: the headless runner extends
the sampling planner's nominal plan with the reference gait each iteration
(`--seed_reference`, on by default). Without that, the sampling planner fills new spline
nodes by holding the last value and only reaches the gait through the Gait cost. The GUI has
no such hook, so it relies on the Gait cost alone.

## Running

GUI: `./bin/mjpc --task=TurtleCompliantSwimming`. Planning is ~20× slower than real time on
4 cores (32 rollouts × 1 s horizon on a 32-DoF model with 54 fluid ellipsoids), so in the GUI
expect slow motion, or use the runner:

```bash
cd build/bin
./turtle_trajopt --mode=reference --duration=20 --out=reference.csv
./turtle_trajopt --mode=mpc --duration=20 --weight=Gait=1 --out=mpc.csv
python ../../mjpc/tasks/turtle_compliant_swimming/analyze_runs.py \
    reference=reference.csv mpc=mpc.csv --out-dir figs --extract mpc
./turtle_trajopt --mode=reference --gait_csv=figs/gait_mpc.csv --duration=20   # open loop
./turtle_trajopt --mode=mpc --gait_csv=figs/gait_mpc.csv ...                   # next starting point
```

Runner flags: `--weight=Name=value,...`, `--param=Name=value,...`,
`--numeric=agent_planner=2,agent_horizon=1.5,...` (any `task.xml` numeric), `--replan` (sim
seconds between planning steps, default 0.02), `--iterations` per step, `--warmup` iterations
at t = 0, `--threads`. Planning is synchronous, so results don't depend on machine speed.
The summary skips the first two strokes and averages over whole strokes after that.

## Results

See [RESULTS.md](RESULTS.md).
