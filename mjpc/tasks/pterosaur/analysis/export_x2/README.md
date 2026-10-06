# Exported launch: tracked_X2_arm1.0_faster_smooth/step1_T0.45_v7_m0

Self-contained: needs only `mujoco` (exported with 3.14.0) and `numpy`.

- `model.xml` + `assets/`: the robot and floor as standalone MJCF (motor
  gains, torque-speed lines, shoulder differential tendons, joint limits and
  contact parameters baked in). Time step 0.002 s, mass 50.1 kg.
  Keyframe `launch_start` = the start pose (start at rest).
- `trajectory.npz`: `start_qpos`; `ctrl` (500 x nu, actuator commands in
  [-1, 1], one row per step); recorded `qpos`, `qvel`, `comvel` (CoM
  velocity), `time` and contact flags, each row the state *after* that
  step; `takeoff_step`; `controls_symmetric` (the optimizer's 6 left/right
  symmetric channels, mirrored to the 12 actuators as in `ctrl`).
- `replay.py`: replays `ctrl` from `start_qpos` and checks against the
  recording (export check: max |qpos error| 4.0e-06).
- `info.json`: actuator and joint names, stall torques, the optimizer
  settings that produced the launch.

Takeoff at 0.474 s: 3.89 m/s at 30.4 deg.

Actuator model: force = gain * ctrl - (gain / 20 rad/s) * actuator speed,
clipped to +-gain (DC motor torque-speed line). The two shoulder actuators
per side drive fixed tendons (swing + abduction, swing - abduction).
