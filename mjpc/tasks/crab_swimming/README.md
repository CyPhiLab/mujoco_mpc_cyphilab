# Crab Swimming Task for MJPC

Generic body-motion locomotion controller for 3D target tracking with a 12-DOF position-controlled crab robot.

## Overview

**CrabSwimming** is an MJPC task for a 12-joint crab robot swimming toward a 3D target in fluid. The current controller tracks the target from the body-center reference site `base_cog`, uses a short planning horizon for reactive motion, and adds amplitude and control-rate regularization to encourage smaller, smoother flapping motions.

### Current Configuration
- **Planning horizon:** 0.5 s
- **Timestep:** 0.02 s
- **Desired speed:** 0.70 m/s
- **Slow radius:** 0.20 m
- **Locomotion axis:** `0 0 1` in the body frame
- **Body up axis:** `0 1 0`
- **Environment normal:** `0 0 1`
- **Target default position:** `(0, 0, 2.0)`
- **Control smoothing:** enabled, `tau = 0.08 s`
- **Amplitude regularization weight:** 0.05
- **Control-rate regularization weight:** 0.05
- **Align weight:** 0.0

### Key Features
- **Body-center targeting:** Position, velocity, and angular velocity are measured from `base_cog`
- **Position actuators:** The planner commands desired joint angles, not torques
- **One-sided progress floor:** Being too slow is penalized; overspeed is not
- **Slip suppression:** Penalizes velocity perpendicular to the target direction
- **Soft trim control:** Keeps body-up near the environment normal without hard orientation locking
- **Angular-rate damping:** Reduces tumbling orthogonal to the environment normal
- **Joint regularization:** Penalizes large joint commands and abrupt command changes
- **Control smoothing:** Applies a first-order low-pass filter before controls reach the servos

---

## Files

**`crab_swimming.h`**
- Declares `CrabSwimming` and its nested `ResidualFn`
- Stores filtered controls and previous controls for smoothing and rate penalties

**`crab_swimming.cc`**
- Implements a **41-residual** controller:
  - `[0-2]` position error from `base_cog` to target
  - `[3]` one-sided progress floor
  - `[4-6]` slip velocity
  - `[7-9]` locomotion-axis alignment residual
  - `[10-12]` trim residual
  - `[13-15]` angular-rate damping
  - `[16]` scalar control effort
  - `[17-28]` amplitude regularization for all 12 joints
  - `[29-40]` control-rate regularization for all 12 joints
- Filters `data->ctrl` in `TransitionLocked()` using a first-order low-pass filter

**Actuator Mapping (ctrl indices 0-11)**
```text
0-1:   Front right leg (mid, distal)
2-3:   Middle right leg (mid, distal)
4-5:   Back right leg (mid, distal)
6-7:   Front left leg (mid, distal)
8-9:   Middle left leg (mid, distal)
10-11: Back left leg (mid, distal)
```

**`task.xml`**
- Sets planner configuration and cost weights
- Defines the body-center sensors:
  - `base_pos_task` from `base_cog`
  - `base_vel_world_task` from `base_cog`
  - `base_angvel_world_task` from `base_cog`
  - `target_pos_task` from the mocap target body
- Places the default target at `(0, 0, 2.0)`

**`crab_swimming_model.xml`**
- Includes `Robot_modified.xml`

**`patch_crab_robot.py`**
- Adds the `base_cog` site to the floating base
- Reduces position-servo `kp` from 200 to 30 for stability
- Removes pre-existing sensors so MJPC task sensors can be declared in `task.xml`
- No longer adds a separate forward-point site

---

## Cost Weights

| Residual | Name | Weight | Description |
|----------|------|--------|-------------|
| [0-2] | Position | 2.0 | 3D vector error (`base_cog` → target) |
| [3] | Progress | 8.0 | One-sided speed floor toward target |
| [4-6] | Slip | 3.0 | Velocity perpendicular to target direction |
| [7-9] | Align | 0.0 | Locomotion-axis alignment residual, currently disabled |
| [10-12] | Trim | 0.3 | Body-up relative to environment normal |
| [13-15] | AngVel | 0.5 | Angular-rate damping orthogonal to environment normal |
| [16] | Control | 0.05 | Scalar control effort |
| [17-28] | Amplitude | 0.05 | Joint-angle magnitude regularization |
| [29-40] | ControlRate | 0.05 | Joint-command rate regularization |

---

## Notes on the Current Design

### Body-center sensing
The controller now computes target displacement, progress, slip, and angular-rate terms using `base_cog`. This avoids relying on an offset forward-point site and makes the objective purely body-center based.

### Locomotion axis vs Align cost
The locomotion axis is still configured in the task as `0 0 1`, but the explicit Align cost weight is set to `0.0`. That means the controller still knows what the nominal swim axis is, but it is not currently paying a direct penalty for locomotion-axis misalignment.

### Smoothing and regularization
The current controller uses three separate mechanisms to tame motion:
- first-order control smoothing in `TransitionLocked()`
- amplitude regularization on all 12 commanded joint angles
- control-rate regularization against the previous filtered command

Together these are meant to reduce large flapping and abrupt command changes while preserving forward progress.

---

## Quick Start

### Build
```bash
cd /path/to/mujoco_mpc_cyphilab
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target mjpc -j$(nproc)
```

### Run
```bash
./build/bin/mjpc --task="CrabSwimming"
```

### Interact
- Move the mocap target in the simulator to retask the crab
- Observe the body-center target tracking and smoothed joint commands
- Edit `task.xml` to retune horizon, speed, axes, or cost weights

---

## Tuning Guide

### If the crab is too sluggish
```xml
<numeric name="agent_horizon" data="0.6" />
<numeric name="residual_DesiredSpeed" data="0.80" />
```
A slightly longer horizon or higher desired speed can make the controller less conservative.

### If the leg motion is too large
```xml
<numeric name="residual_AmplitudeWeight" data="0.10" />
```
This increases the cost on large joint-angle commands.

### If the leg motion is too jerky
```xml
<numeric name="residual_ControlRateWeight" data="0.10" />
<numeric name="residual_ControlSmoothingTau" data="0.12" />
```
Increase either the explicit rate penalty, the filter time constant, or both.

### If the crab drifts sideways
```xml
<user name="Slip" dim="3" user="0 5.0 0 10.0" />
```
The Align term is currently disabled, so sideways drift is mostly handled by the Slip cost.

### If you want explicit heading enforcement again
```xml
<user name="Align" dim="3" user="0 2.5 0 10.0" />
```
This re-enables the locomotion-axis alignment penalty.

---

## Dependencies and Assets

- MuJoCo 3.2+
- C++17
- CMake-based MJPC build
- Crab model assets under the crab task resources used by `Robot.xml` and `Scene.xml`

---

## Known Issues

| Issue | Solution |
|-------|----------|
| Build fails because `base_cog` is missing | Rebuild so `patch_crab_robot.py` regenerates `Robot_modified.xml` |
| Simulation diverges | Check that the patched model reduced actuator `kp` to 30 |
| Motion is too twitchy | Raise `residual_ControlRateWeight` or `residual_ControlSmoothingTau` |
| Motion is too floppy | Raise `residual_AmplitudeWeight` |
| Task does not appear in MJPC | Rebuild `mjpc` and verify the task is registered in the task list |


---

## Related Tasks

- **Turtle3DSwimming** — Generic body-motion controller with velocity actuators (10 DOF)
- **Other locomotion tasks** — See `mjpc/tasks/` for walker, quadruped, and bimanual manipulation examples

---

## References

- MuJoCo documentation: https://mujoco.readthedocs.io/
- MJPC repository: https://github.com/deepmind/mujoco_mpc
- CyPhiLab MJPC fork: https://github.com/CyPhiLab/mujoco_mpc_cyphilab
