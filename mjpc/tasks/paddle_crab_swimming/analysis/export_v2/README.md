# Paddle crab gait primitives (export_v2)

Made by `mjpc/tasks/paddle_crab_swimming/analysis/gait_chain.py` and exported by `export_gaits.py`.

- `model.xml`: the MJPC robot (after `patch_paddle_crab_robot.py`) with the servo setpoint filter as built-in `filter` dynamics (tau = 0.08 s), 1 ms timestep, contacts off, zero gravity.
- `gaits.npz`: per gait the period, Fourier coefficients of the servo command (radians), one-cycle command tables at 100 Hz and the joint-angle limit cycle.
- `<name>_commands_100hz.csv`: one cycle of servo commands (radians), actuator order R1_1 ... L3_3; loop it. On hardware the commands assume a first-order setpoint lag of 0.08 s.
- `replay.py [NAME] [CYCLES]`: open-loop replay from rest.

| gait | direction | period (s) | steady speed (m/s) | replay from rest (m/s) |
|---|---|---|---|---|
| paddles_lead | [0.0, -1.0, 0.0] | 1.103 | 0.1121 | 0.0922 |
| paddles_trail | [0.0, 1.0, 0.0] | 1.227 | 0.0855 | 0.0734 |
| side | [1.0, 0.0, 0.0] | 0.728 | 0.0728 | 0.0485 |
| crab_side | [1.0, 0.0, 0.0] | 1.206 | 0.0840 | 0.0704 |

The replay average includes the start-up transient from rest, so it is lower than the limit-cycle speed.
