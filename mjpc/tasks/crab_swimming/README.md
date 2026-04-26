# Crab Swimming Task for MJPC

Generic body-motion locomotion controller for 3D target-tracking with a 12-DOF position-controlled crab robot.

## Overview

**CrabSwimming** is a fully-tuned MPC task implementing a generic body-motion controller for a 12-joint crab robot swimming toward a 3D target. Like Turtle3DSwimming, this controller is morphology-agnostic and reusable—configurable locomotion axes and environmental parameters allow adaptation to other swimmers or walkers by changing only XML parameters.

### Key Features
- **Morphology-agnostic:** Body-frame locomotion axis, body-up axis, and environment normal are all configurable parameters
- **Position actuators:** Controls desired joint angles (not forces), making it suitable for robots with joint-level servo control (e.g., position-controlled legs)
- **Base-body targeting:** Tracks distance and direction from the plate (floating base) center to target
- **One-sided progress floor:** Penalizes only being too slow; does not penalize overspeed
- **Soft attitude control:** Cross-product alignment and trim constraints avoid over-constraining orientation
- **Angular rate damping:** Reduces body tumbling perpendicular to the environment normal
- **Control smoothing:** First-order low-pass filtering on joint targets reduces abrupt angle changes without affecting locomotion objective
- **Real crab geometry:** 12 DOF with 6 legs (front, middle, back on each side), mid and distal segments per leg

### Performance
- **Planning Horizon:** 2.0 seconds
- **Sampling:** 64 trajectories, 5 spline points
- **Approach Speed:** 0.60 m/s (configurable via `residual_DesiredSpeed`)
- **Actuator Gains:** Position servo with Kp=30 (stabilized during build to prevent numerical divergence)

---

## Files

**`crab_swimming.h`** — Task class header
- Defines `CrabSwimming` inheriting from MJPC `Task`
- Declares `ResidualFn` with 17 residuals
- Includes filtered control state for joint-target smoothing

**`crab_swimming.cc`** — Generic body-motion controller implementation
- `[0-2]` **3D position error:** Vector from base to target (not scalar distance)
- `[3]` **One-sided forward progress floor:** Penalizes only being slower than desired; ignores overspeed
- `[4-6]` **Cross-track / slip velocity:** Perpendicular component of base velocity relative to target direction
- `[7-9]` **Locomotion-axis alignment:** `cross(locomotion_axis_world, target_direction)` — zero when locomotion axis points at target
- `[10-12]` **Trim relative to environment normal:** `cross(body_up_world, environment_normal)` — zero when body-up aligns with environment normal
- `[13-15]` **Angular-rate damping orthogonal to environment:** Penalizes rolling/tumbling perpendicular to environment normal
- `[16]` **Scalar control effort:** Equal weighting across all 12 actuators (no per-leg discounts)

**Actuator Mapping (ctrl indices 0-11):**
```
0-1:   Front right leg (mid segment, distal segment)
2-3:   Middle right leg (mid segment, distal segment)
4-5:   Back right leg (mid segment, distal segment)
6-7:   Front left leg (mid segment, distal segment)
8-9:   Middle left leg (mid segment, distal segment)
10-11: Back left leg (mid segment, distal segment)
```

**`task.xml`** — MJPC configuration and configurable parameters
- **Residual parameters (configurable):**
  - `residual_DesiredSpeed` (default 0.60 m/s): target cruise speed
  - `residual_SlowRadius` (default 0.35 m): distance at which to start tapering speed toward goal
  - `residual_LocomotionAxisBody` (default `1 0 0`): forward direction in body frame (along crab's length)
  - `residual_BodyUpAxisBody` (default `0 0 1`): "up" direction in body frame (used for trim)
  - `residual_EnvNormalWorld` (default `0 0 1`): environment normal in world frame (gravity direction for swimming)
  - `residual_ControlSmoothingEnabled` (default `1`): enables first-order joint-target smoothing
  - `residual_ControlSmoothingTau` (default `0.08` s): low-pass filter time constant for joint targets
- Includes `crab_swimming_model.xml`

**`crab_swimming_model.xml`** — Include wrapper
- References `Robot_modified.xml` (patched during build with base_cog site added and Kp reduced)

**`patch_crab_robot.py`** — Build-time model patcher
- Copies Robot.xml from `../../Crab/mujoco_models/`
- Adds `base_cog` reference site to plate body for sensor attachment
- Reduces position servo gains from Kp=200 to Kp=30 to prevent numerical instability
- Removes pre-existing sensors (MJPC requires user cost sensors to be declared first in task.xml)

---

## Cost Weights (task.xml)

| Residual | Name | Weight | Description |
|----------|------|--------|-------------|
| [0-2] | Position | 2.0 | 3D vector error (base → target) |
| [3] | Progress | 8.0 | One-sided forward progress floor |
| [4-6] | Slip | 3.0 | Cross-track / slip velocity vector |
| [7-9] | Align | 2.5 | Locomotion-axis alignment cross-product |
| [10-12] | Trim | 1.0 | Trim relative to environment normal |
| [13-15] | AngVel | 0.5 | Angular-rate damping orthogonal to environment |
| [16] | Control | 0.01 | Scalar control effort (all 12 joints equal) |

---

## Key Differences from Turtle3DSwimming

| Aspect | Turtle3DSwimming | CrabSwimming |
|--------|------------------|--------------|
| Actuators | 10 velocity-controlled | 12 position-controlled |
| Locomotion Axis | [0 -1 0] (tail-to-head) | [1 0 0] (body length) |
| Control Signal | Motor command (force/torque) | Joint angle target |
| Servo Gains | Built-in (XW540 actuators) | Kp=30 (position servo) |
| Control Smoothing | Applied to forces | Applied to joint targets |
| Leg Configuration | 2 flippers (front & rear) | 6 legs (3 pairs: front/mid/back) |

---

## Customization for Other Morphologies

To adapt this controller to a different robot, change only the three axis parameters in `task.xml`:

```xml
<!-- For crab (default) — moves along body length -->
<numeric name="residual_LocomotionAxisBody" data="1 0 0" />
<numeric name="residual_BodyUpAxisBody"     data="0 0 1" />

<!-- For biped walking on ground, Z is up -->
<numeric name="residual_LocomotionAxisBody" data="0 1 0" />
<numeric name="residual_BodyUpAxisBody"     data="0 0 1" />
<numeric name="residual_EnvNormalWorld"     data="0 0 1" />

<!-- For swimmer with head pointing forward -->
<numeric name="residual_LocomotionAxisBody" data="0 -1 0" />
<numeric name="residual_BodyUpAxisBody"     data="0 0 1" />
<numeric name="residual_EnvNormalWorld"     data="0 0 1" />
```

The cost function logic automatically transforms these axes into world frame and computes alignment/trim relative to them, regardless of robot morphology or actuator count.

---

## Quick Start

### 1. Build in MJPC Workspace
```bash
cd /path/to/mujoco_mpc_cyphilab
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --target mjpc -j$(nproc)
```

The build system automatically:
- Copies `Robot.xml` from `../../Crab/mujoco_models/` (if available)
- Patches it with `patch_crab_robot.py` (adds `base_cog` site, reduces Kp)
- Registers the CrabSwimming task in the UI

### 2. Run
```bash
./build/bin/mjpc  # Select "CrabSwimming" from the task dropdown
```

### 3. Interact
- **Click on simulator** to place target (blue sphere)
- **Observe** the crab's MPC planner compute and execute smooth leg motions toward the target
- **Inspect parameters** via the UI; modify and recompile `task.xml` to re-tune

### 4. Tune Parameters (if needed)
Edit `crab_swimming/task.xml` to adjust:
- `residual_DesiredSpeed`: Increase for faster swimming, decrease for slower
- `residual_SlowRadius`: Distance at which to start decelerating near goal
- `residual_ControlSmoothingTau`: 
  - `0.05` for quick, responsive joint tracking
  - `0.08` (default) for balanced smoothness and responsiveness
  - `0.12` for very smooth but slower joint motions
- `residual_ControlSmoothingEnabled`: Set to `0` to bypass smoothing entirely
- Cost weights in `<user>` elements to balance different objectives

---

## Implementation Notes

### Position Control vs. Velocity Control
- **Turtle3DSwimming** uses velocity actuators: `data->ctrl` represents desired motor speeds, and the planner commands forces
- **CrabSwimming** uses position actuators: `data->ctrl` represents desired joint angles, and the planner commands angle targets
- Position smoothing in `TransitionLocked()` filters joint angles before applying them to the servo controllers

### Build-Time Model Patching
The `patch_crab_robot.py` script runs during CMake configuration:
1. Copies the original `Robot.xml` (which may have high Kp gains for terrestrial locomotion)
2. Adds a `base_cog` reference site to the `plate` body for sensor attachment
3. Reduces position servo Kp from 200 to 30 to ensure numerical stability during MPC trajectory rollouts
4. Removes pre-existing sensors to comply with MJPC's requirement that cost sensors be declared first

### Body-frame Axes
- `residual_LocomotionAxisBody` and `residual_BodyUpAxisBody` are defined in the robot's body frame
- They are transformed to world frame using the root body (`plate`) rotation matrix at runtime
- This allows seamless reuse across different orientations and morphologies

### No Hard Quaternion Constraint
Unlike older versions, this controller does not enforce full quaternion matching. Instead:
- Soft cross-product residuals (`Align`, `Trim`) allow natural attitude variations
- Angular velocity damping (`AngVel`) prevents unwanted tumbling
- This approach is more physically natural and robust to model discrepancy

---

## Tuning Guide

### If the crab feels sluggish
```xml
<!-- Lower control smoothing time constant -->
<numeric name="residual_ControlSmoothingTau" data="0.05" />
```
This makes joint angles respond faster to MPC commands (at the cost of more abrupt motions).

### If the crab's leg motions look jerky
```xml
<!-- Increase control smoothing time constant -->
<numeric name="residual_ControlSmoothingTau" data="0.12" />
```
This smooths the joint angle trajectory but may reduce responsiveness.

### If the crab is not reaching targets quickly enough
```xml
<!-- Increase desired speed and/or progress weight -->
<numeric name="residual_DesiredSpeed" data="0.80" />  <!-- up from 0.60 -->

<!-- In task.xml, increase Progress weight -->
<user name="Progress" dim="1" user="0 12.0 0 1.0" />  <!-- up from 8.0 -->
```

### If the crab is drifting sideways
```xml
<!-- Increase Slip or Align weight -->
<user name="Slip"  dim="3" user="0 5.0  0 10.0" />  <!-- up from 3.0 -->
<user name="Align" dim="3" user="0 3.5  0 10.0" />  <!-- up from 2.5 -->
```

### If the crab is rolling excessively
```xml
<!-- Increase Trim weight to enforce stricter upright posture -->
<user name="Trim" dim="3" user="0 2.0 0 10.0" />  <!-- up from 1.0 -->
```

---

## Dependencies & Assets

- **MuJoCo 3.2+**, **C++17**, **CMake 3.15+**
- **Crab model files:** Expected at `../../Crab/mujoco_models/`
  - `Robot.xml` (floating base, 12 DOF position-controlled legs)
  - `Scene.xml` (water environment)
  - `asset/` directory (mesh files)
- Water physics and lighting inherited from `common.xml` in the parent MJPC project

---

## Known Issues & Solutions

| Issue | Solution |
|-------|----------|
| Build fails: "Cannot find Robot.xml" | Ensure `../../Crab/mujoco_models/Robot.xml` exists relative to build directory |
| Build fails: "Unrecognized name 'base_cog'" | The patch script may have failed; check that `patch_crab_robot.py` ran successfully |
| Simulation diverges (NaN in QACC) | Position servo Kp is too high; should be ~30 (patch script should handle this) |
| Crab feels sluggish | Lower `residual_ControlSmoothingTau` from 0.08 to 0.05 |
| Crab's legs twitch | Raise `residual_ControlSmoothingTau` from 0.08 to 0.12 |
| Task not appearing in UI | Ensure `tasks.cc` includes `crab_swimming.h` and registers `std::make_shared<CrabSwimming>()` |

---

## Related Tasks

- **Turtle3DSwimming** — Generic body-motion controller with velocity actuators (10 DOF)
- **Other locomotion tasks** — See `mjpc/tasks/` for walker, quadruped, and bimanual manipulation examples

---

## References

- MuJoCo documentation: https://mujoco.readthedocs.io/
- MJPC repository: https://github.com/deepmind/mujoco_mpc
- CyPhiLab MJPC fork: https://github.com/CyPhiLab/mujoco_mpc_cyphilab
