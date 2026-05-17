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
- `agent_timestep = 0.02` (20 millisecond control timestep)
- `sampling_trajectories = 128` (128 trajectory samples per iteration)
- `sampling_spline_points = 8` (8-point spline for trajectory generation)
- `sampling_exploration = 0.12` (exploration noise level for sampling)
- `gradient_spline_points = 20` (gradient computation spline points)
- `residual_DesiredSpeed = 0.7` (target forward speed in m/s)
- `residual_SlowRadius = 0.20` (slow-down region radius in meters)
- `LocAxis = body +Y` (body-frame forward direction)
- `UpAxis = body +Z` (body-frame up direction)
- `EnvNormal = world +Z` (environment gravity direction)
- `ControlSmoothingEnabled = 1` (enable first-order control filtering)
- `ControlSmoothingTau = 0.18` (smoothing time constant in seconds)
- Default target position: (0, 1.0, 3.0) in front of the robot

## Residual Weights Summary

| Residual | Dimension | Weight | Notes |
|----------|-----------|--------|-------|
| Position | 3 | 4.0 | Track target location (3D error) |
| Progress | 1 | 9.0 | One-sided penalty for speeds below 0.7 m/s |
| Slip | 3 | 2.0 | Penalize lateral/perpendicular velocity w.r.t. target direction |
| Align | 3 | 0.5 | Heading alignment using front_point direction from base_cog |
| Trim | 3 | 0.2 | Stabilize body upright via cross(body_up, env_normal) |
| AngVel | 3 | 0.5 | Damp angular velocity perpendicular to environment normal |
| Control | 1 | 0.04 | Morphology-aware scalar effort with per-actuator weights |
| ControlRate | 18 | 0.4 | Per-actuator rate penalty with morphology weights |

## Implementation Details

**Residual Computation (35 total residuals for 18-control hexapod)**
- [0-2]: 3D position error to target
- [3]: One-sided progress floor (penalizes speeds below desired speed)
- [4-6]: Slip velocity (perpendicular component to target direction)
- [7-9]: Alignment error (heading toward target using front_point direction)
- [10-12]: Trim (body-up stabilization)
- [13-15]: Angular velocity damping (perpendicular to environment normal)
- [16]: Control effort (weighted L2 norm with morphology awareness)
- [17-34]: Control-rate regularization (per-actuator penalties)

**Sensor Setup**
- `base_pos_task`: Body center-of-mass position (base_cog site)
- `front_pos_task`: Front reference point (front_point site) for heading direction
- `target_pos_task`: 3D target position
- `base_vel_world_task`: Body velocity in world frame
- `base_angvel_world_task`: Body angular velocity in world frame

## Morphology-Aware Control Regularization

**Per-Actuator Control Weights**

The `Control` residual computes `sqrt(sum_i w_i * ctrl[i]^2)` with morphology-aware weights:
- **Front legs (R1, L1)**: control weight 3.0 — most expensive, used for fine steering
- **Center legs (R2, L2)**: control weight 1.8 — moderate cost, provide stabilization
- **Rear support (R3_{1,2}, L3_{1,2})**: control weight 0.7 — cheaper, enable propulsion
- **Rear distal paddles (R3_3, L3_3)**: control weight 0.4 — cheapest, primary swimming propulsion

**Per-Actuator Rate Weights**

The `ControlRate` residual applies: `0.4 * sqrt(rate_weight[i]) * (ctrl[i] - prev_ctrl[i])`
- **Front legs (R1, L1)**: rate weight 2.5 — smooth, precise steering
- **Center legs (R2, L2)**: rate weight 1.5 — moderate rate penalty
- **Rear support (R3_{1,2}, L3_{1,2})**: rate weight 0.8 — smoother propulsion
- **Rear distal paddles (R3_3, L3_3)**: rate weight 0.8 — efficient paddle motion

This biasing encourages:
- Rear paddles for efficient propulsion (low control cost)
- Smooth control changes via rate regularization
- Front legs reserved for steering corrections (high control cost discourages overuse)

**Actuator Configuration**
- Position actuator `kp = 35` (soft control tracking for smooth motion)
- Actuator control ranges by leg segment:
  - Front legs (R1_*, L1_*): ±0.9
  - Center legs (R2_*, L2_*): ±1.0
  - Rear support (R3_{1,2}, L3_{1,2}): ±1.2
  - Rear distal paddles (R3_3, L3_3): ±1.5 (wider range for aggressive paddling)

## Expected Behavior

- **Rear paddles** dominate propulsion due to low control cost (0.4) and enable steering via asymmetric motion
- **Center legs** provide secondary support and stabilization (control cost 1.8)
- **Front legs** remain relatively quiet for directional control (highest control cost 3.0)
- **Smooth motion** achieved via control-rate regularization and first-order smoothing filter (tau=0.18s)
- **Emergent gait**: No CPG, DMP, phase variables, or hand-coded trajectories—all motion emerges from MPC optimization
- **Directional control** via asymmetric rear paddle motion; `Align` weight (0.5) provides gentle guidance toward target

## Task Running

```bash
cd build
./bin/mjpc --task PaddleCrabSwimming
```

The GUI allows real-time target repositioning (mocap body manipulation) to test tracking performance.

---

*Note: This task was formerly prototyped as CrabSwimming2. All code, config, and scripts are now unified under PaddleCrabSwimming.*
