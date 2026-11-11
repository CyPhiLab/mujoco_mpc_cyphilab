# Paddle crab gait primitives: how to use them

These are open-loop periodic swimming gaits for the PaddleCrabSwimming robot
(18 servos, DYNAMIXEL XW430 at 12 V), made by offline trajectory optimization.
They live in repo `CyPhiLab/mujoco_mpc_cyphilab`, branch `updated_paddle`,
folder `mjpc/tasks/paddle_crab_swimming/analysis/`.

## The library

| name | motion | speed | stroke T |
|---|---|---|---|
| paddles_lead | swim −Y (toward the paddles) | 0.116 m/s | 1.10 s |
| paddles_cruise | swim −Y, walking legs still: ~3× less energy per metre | 0.091 m/s | 1.09 s |
| paddles_trail | swim +Y (anterior) | 0.086 m/s | 1.23 s |
| side / side_left | swim +X / −X (robot right / left) | 0.073 m/s | 0.73 s |
| diag_right_lead / diag_left_lead | swim (±X, −Y)/√2 | 0.080 m/s | 0.74 s |
| diag_right_trail / diag_left_trail | swim (±X, +Y)/√2 | 0.070 m/s | 1.22 s |
| turn_left / turn_right | turn on the spot | ±0.373 rad/s (CCW + from above) | 2.26 s |
| up / down | swim +Z / −Z | 0.071 / 0.072 m/s | 1.71 / 2.15 s |
| crab_side | +X, alternative (±π commands, tilts) | 0.084 m/s | 1.21 s |

The robot frame is anterior = +Y, right = +X, up = +Z. The paddles are the
rear legs (actuators R3_*, L3_*). Directions are in the frame of the start
heading: a translation gait keeps the body's heading and moves in that
direction.

`paddles_lead` is the fast gait. `paddles_cruise` is the efficient one, with
cost of transport 1.5 against 4.1. It sinks about 1.8 cm/s open loop, which
feedback corrects.

Speeds are steady state. From rest it takes about 2–4 strokes to get up to
speed. Up and down slide about 2 cm/s toward −Y open loop and need feedback
to hold position. The others drift under about 1 cm/s and turn under about
0.2 rad in 20 s. `runs/primitives_report.md` has every metric.

## Where the data is

**`export_v3/gaits.npz`** is the main source. Gait `i` has:

- `names[i]`, `period[i]` (s), `k[i]` (harmonics)
- `speed[i]`, `direction[i]` (unit 3-vector), `yaw_rate[i]` (rad/s)
- `c[i]` (18), `a[i]` and `b[i]` (18 × K, padded; use the first `k[i]`
  columns: paddles_lead has K = 5, the rest K = 3)
- `half[i]` (18): the per-servo command limit, rad
- `table_time[i]`, `table_ctrl[i]`: one cycle of commands at 100 Hz
- `joint_phase[i]`, `joint_qpos[i]`: the resulting joint-angle limit cycle
  (200 phases × 18)

All angles are in radians. Servo order is in `actuators`:

```
R1_1 R1_2 R1_3 R2_1 R2_2 R2_3 R3_1 R3_2 R3_3 L1_1 L1_2 L1_3 L2_1 L2_2 L2_3 L3_1 L3_2 L3_3
```

Legs are R1 R2 R3 L1 L2 L3 (front, centre, rear/paddle). The joints are
proximal, mid, distal; the joint names are in `joints`.

The other files in `export_v3/`:

- **`<name>_commands_100hz.csv`:** one cycle of commands (`time` + 18
  columns). Loop it.
- **`model.xml`:** standalone MuJoCo model, the same physics as MJPC.
  - 1 ms timestep, zero gravity (neutrally buoyant), contacts off.
  - The servo setpoint filter is built in.
- **`replay.py [NAME] [CYCLES]`:** open-loop replay from rest.
- **`../gait_reference.xml`:** the same library for the MJPC task. The
  `Gait` drop-down (`residual_select_Gait`) picks one.

## The command law

```
u_i(t) = clip( c_i + sum_{j=1..k} [ a_ij cos(2πj t/T) + b_ij sin(2πj t/T) ],  -half_i, +half_i )
```

`u` is the servo **setpoint**. The robot's servos filter it first-order
(tau 0.08 s), then track it with a kp 50 position servo limited to ±3.1 N m.
`model.xml` and MJPC do this filtering. On hardware, either send `u` through
the same 0.08 s low-pass, or expect slightly different motion.

```python
import mujoco, numpy as np
g = np.load('export_v3/gaits.npz', allow_pickle=True)
i = list(g['names']).index('turn_left')
T, k = g['period'][i], g['k'][i]
c, a, b, lim = g['c'][i], g['a'][i][:, :k], g['b'][i][:, :k], g['half'][i]
w = 2*np.pi*np.arange(1, k+1)/T
u = lambda t: np.clip(c + a @ np.cos(w*t) + b @ np.sin(w*t), -lim, lim)

m = mujoco.MjModel.from_xml_path('export_v3/model.xml'); d = mujoco.MjData(m)
while d.time < 20:
    d.ctrl[:] = u(d.time); mujoco.mj_step(m, d)
```

## Using them downstream

- **Switching gaits.** Restart the phase clock (t = 0) or keep the phase
  continuous. Strokes differ in period and posture, so expect a transient of
  about one stroke. Smoothing the switch over about 0.5 s helps.
- **Steering a path.** Pick the translation primitive closest to the desired
  direction, and use turn_left/right to change heading. Mirror pairs
  (side/side_left, the diagonals, the turns) are left/right reflections:
  same speed, opposite drift.
- **Feedback.** These are feedforward. MJPC's gait mode adds the planner's
  correction on top (exploration about 0.01), with the Heading, Lateral,
  Level and Depth terms closing the loop. Closed loop, down and up keep
  their position; open loop they slide.
- **Regenerating or extending.** The pipeline is `gait_chain.py`,
  `mirror_primitives.py`, `export_gaits.py` and `primitive_report.py`;
  `run_primitives.sh` has the exact commands. `README.md`, "Round 3", has
  the details.
