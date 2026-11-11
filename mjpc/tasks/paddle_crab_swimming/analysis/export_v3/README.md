# Paddle crab gait primitives (export_v3)

Made by `mjpc/tasks/paddle_crab_swimming/analysis/gait_chain.py` and exported by `export_gaits.py`.

- `model.xml`: the MJPC robot (after `patch_paddle_crab_robot.py`) with the servo setpoint filter as built-in `filter` dynamics (tau = 0.08 s), 1 ms timestep, contacts off, zero gravity.
- `gaits.npz`: per gait the period, Fourier coefficients of the servo command (radians), one-cycle command tables at 100 Hz and the joint-angle limit cycle.
- `<name>_commands_100hz.csv`: one cycle of servo commands (radians), actuator order R1_1 ... L3_3; loop it. On hardware the commands assume a first-order setpoint lag of 0.08 s.
- `replay.py [NAME] [CYCLES]`: open-loop replay from rest.

| gait | direction | yaw rate (rad/s) | period (s) | steady speed (m/s) | replay from rest (m/s) |
|---|---|---|---|---|---|
| paddles_lead | [0.0, -1.0, 0.0] | +0.000 | 1.098 | 0.1161 | 0.0961 |
| paddles_cruise | [0.0, -1.0, 0.0] | +0.000 | 1.094 | 0.0913 | 0.0748 |
| paddles_trail | [0.0, 1.0, 0.0] | +0.000 | 1.227 | 0.0855 | 0.0734 |
| side | [1.0, 0.0, 0.0] | +0.000 | 0.728 | 0.0728 | 0.0485 |
| side_left | [-1.0, 0.0, 0.0] | +0.000 | 0.728 | 0.0728 | 0.0485 |
| diag_right_lead | [0.71, -0.71, 0.0] | +0.000 | 0.740 | 0.0799 | 0.0594 |
| diag_left_lead | [-0.71, -0.71, 0.0] | +0.000 | 0.740 | 0.0801 | 0.0594 |
| diag_right_trail | [0.71, 0.71, 0.0] | +0.000 | 1.223 | 0.0701 | 0.0583 |
| diag_left_trail | [-0.71, 0.71, 0.0] | +0.000 | 1.223 | 0.0702 | 0.0584 |
| turn_left | [0.0, -1.0, 0.0] | +0.373 | 2.258 | 0.0011 | -0.0045 |
| turn_right | [-0.0, -1.0, 0.0] | -0.373 | 2.258 | 0.0009 | -0.0042 |
| down | [0.0, 0.0, -1.0] | +0.000 | 2.146 | 0.0718 | 0.0461 |
| up | [0.0, 0.0, 1.0] | +0.000 | 1.706 | 0.0705 | 0.0625 |
| crab_side | [1.0, 0.0, 0.0] | +0.000 | 1.206 | 0.0840 | 0.0704 |

The replay average includes the start-up transient from rest, so it is lower than the limit-cycle speed.
