# PaddleCrabSwimming Task for MJPC

This is an 18-control paddle crab swimming task for MJPC.

- The robot is a 6-legged (18 DOF) paddle crab, swimming in fluid.
- The task uses a directional target-tracking MPC objective. The robot's forward direction is defined by the vector from base_cog to front_point, with the rear paddle legs behind the body.
- The cost structure is general and does not use CPG, DMP, phase, amplitude, or hand-coded gait timing.
- The objective is emergent MPC swimming, with actuator limits and morphology-aware control regularization that bias the optimizer toward using the rear paddles when that is dynamically useful.

## Files
- `paddle_crab_swimming.h` / `paddle_crab_swimming.cc`: Task implementation
- `task.xml`: Task configuration and cost weights
- `model.xml`: Includes the cross-domain robot
- `patch_paddle_crab_robot.py`: Patches robot.xml for MJPC compatibility

## Task Parameters
- `agent_horizon = 1.5` (1.5 second planning window)
- `agent_timestep = 0.02`
- `sampling_trajectories = 128` (128 trajectory samples per iteration)
- `sampling_spline_points = 8` (8-point spline for trajectory generation)
- `sampling_exploration = 0.12` (exploration noise level)
- `gradient_spline_points = 20` (gradient computation spline points)
- `residual_DesiredSpeed = 0.7` (target forward speed in m/s)
- `residual_SlowRadius = 0.20` (slow-down region radius)
- `LocAxis = body +Y` (body-frame forward direction, fallback for alignment)
- `UpAxis = body +Z` (body-frame up direction)
- `EnvNormal = world +Z` (environment gravity direction)
- `ControlSmoothingEnabled = 1` (enable first-order control filtering)
- `ControlSmoothingTau = 0.18` (smoothing time constant in seconds)
- Default target position: (0, 1.0, 3.0) in front of the robot

## Residual Weights Summary

| Residual | Dimension | Weight | Notes |
|----------|-----------|--------|-------|
| Position | 3 | 4.0 | Track target location |
| Progress | 1 | 6.0 | Penalize speeds below 0.25 m/s (reduced for smooth debugging) |
| Slip | 3 | 2.0 9.0 | Penalize speeds below 0.7 m/s; one-sided penalty |
| Slip | 3 | 2.0 | Penalize lateral/perpendicular velocity w.r.t. target direction |
| Align | 3 | 0.5 | Heading alignment using front_point direction from base_cog |
| Trim | 3 | 0.2 | Stabilize body upright via cross(body_up, env_normal) |
| AngVel | 3 | 0.5 | Damp angular velocity perpendicular to environment normal |
| Control | 1 | 0.04 | Morphology-aware scalar effort with per-actuator weights |
| ControlRate | 18 | 0.4 | Per-actuator rate penalty; rear paddles at 0.8, mid-legs at 1.5, front at 2.5
## Implementation Details

**Residual Computation**
- Total residuals: 17 fixed + 18 control-rate = 35 total
  - [0-2]: 3D position error to target
  - [3]: One-sided progress floor (penalizes speeds below desired)
  - [4-6]: Slip velocity (perpendicular component)
  - [7-9]: Alignment error (heading toward target)
  - [10-12]: Trim (body-up stabilization)
  - [13-15]: Angular velocity damping
  - [16]: Control effort (weighted L2 norm)
  - [17-34]: Control-rate regularization (per-actuator penalties)

**Sensor Setup**
- `base_pos_task`: Body center-of-mass position (base_cog site)
- `front_pos_task`: Front reference point (front_point site) for heading direction
- `target_pos_task`: 3D target position
- `base_vel_world_task`: Body velocity in world frame
- `base_angvel_world_task`: Body angular velocity in world frame

## Morphology-Aware Regularization

**Rear-Paddle-Biased Control**
- The scalar `Control` residual computes `sqrt(sum_i w_i * ctrl[i]^2)` with morphology-aware weights:
  - **Front legs (R1, L1)**: control weight 3.0, rate weight 2.5 — most expensive
  - **CenterBehavior
- **Rear paddles** dominate propulsion (cheapest control cost) and enable steering via asymmetric motion.
- **Center legs** provide secondary support and fine motion control (medium cost).
- **Front legs** remain relatively quiet, used mainly for alignment and fine corrections (highest cost).
- **Smooth motion** via control-rate regularization and first-order smoothing filter (tau=0.18s).
- **Emergent gait**: No CPG, DMP, phase variables, or hand-coded trajectories—all motion emerges from MPC optimization.

## Task Running
```bash
cd build
./bin/mjpc --task PaddleCrabSwimming
```
The GUI allows real-time target repositioning (mocap body manipulation) to test tracking performancets.

**Actuator Configuration**
- Position actuator `kp = 35` (soft control tracking for smooth motion)
- Actuator contControl (Alignment)
- The `Align` residual uses the front_point direction: `front_point - base_cog` relative to the target direction.
- This computes `cross(body_forward_world, target_direction)` to penalize heading misalignment.
- The robot is free to use asymmetric rear paddle motion for steering and maneuvering.
- Weak `Align` weight (0.5) allows emergent steering behaviors while maintaining speed/efficiency focus
  - Rear paddle (distal): ±1.5 (wider range for aggressive paddling)

## Directional Align
- RearPairSymmetry was removed, so the robot can use asymmetric rear paddle motion for steering.
- Directionality is handled through a weak `Align` term using only `front_point - base_cog`.
- This encourages the crab to face the target without forcing left-right rear paddle symmetry.

## Expected Motion
- Rear paddles should provide most of the propulsion, while remaining free to move asymmetrically for steering.
- Center legs may contribute stabilization or secondary support.
- Front legs should remain relatively quiet.
- No CPG, DMP, phase variables, sine waves, amplitude references, or hand-coded trajectories are used.

## Usage
- Register this task in `tasks.cc` and `CMakeLists.txt` to appear after CrabSwimming in the task selection bar.

---

*Note: This task was formerly prototyped as CrabSwimming2. All code, config, and scripts are now unified under PaddleCrabSwimming.*