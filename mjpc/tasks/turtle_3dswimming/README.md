# Turtle 3D Swimming Task for MJPC

Generic body-motion locomotion controller for 3D target-tracking with morphology-agnostic design.

## Overview

**Turtle3DSwimming** is a fully-tuned MPC task implementing a generic body-motion controller for autonomous vehicles (swimming or walking). The robot reaches a 3D target using configurable locomotion axes and environmental parameters, making it reusable across different morphologies (turtle, crab, etc.) by changing a few axis parameters.

### Key Features
- **Morphology-agnostic:** Body-frame locomotion axis, body-up axis, and environment normal are all configurable parameters
- **Head-based targeting:** Tracks distance and direction from head position (end-effector) to target
- **One-sided progress floor:** Penalizes only being too slow; does not penalize overspeed
- **Soft attitude control:** Cross-product alignment and trim constraints avoid over-constraining orientation
- **Angular rate damping:** Reduces body tumbling perpendicular to the environment normal
- **Real robot geometry:** 6.33 kg Onshape CAD model with velocity-controlled XW540-T260 actuators

### Performance
- **Planning Horizon:** 2.0 seconds
- **Sampling:** 64 trajectories, 5 spline points
- **Approach Speed:** 0.60 m/s (configurable via `residual_DesiredSpeed`)

---

## Files

**`turtle_3dswimming.h`** — Task class header
- Defines `Turtle3DSwimming` inheriting from MJPC `Task`
- Declares `ResidualFn` with 17 residuals

**`turtle_3dswimming.cc`** — Generic body-motion controller implementation
- `[0-2]` **3D position error:** Vector from head to target (not scalar distance)
- `[3]` **One-sided forward progress floor:** Penalizes only being slower than desired; ignores overspeed
- `[4-6]` **Cross-track / slip velocity:** Perpendicular component of base velocity relative to target direction
- `[7-9]` **Locomotion-axis alignment:** `cross(locomotion_axis_world, target_direction)` — zero when locomotion axis points at target
- `[10-12]` **Trim relative to environment normal:** `cross(body_up_world, environment_normal)` — zero when body-up aligns with environment normal
- `[13-15]` **Angular-rate damping orthogonal to environment:** Penalizes rolling/tumbling perpendicular to environment normal
- `[16]` **Scalar control effort:** Equal weighting across all actuators (no morphology-specific discounts)

**`task.xml`** — MJPC configuration and configurable parameters
- **Residual parameters (configurable):**
  - `residual_DesiredSpeed` (default 0.60 m/s): target cruise speed
  - `residual_SlowRadius` (default 0.35 m): distance at which to start tapering speed toward goal
  - `residual_LocomotionAxisBody` (default `0 -1 0`): forward direction in body frame
  - `residual_BodyUpAxisBody` (default `0 0 1`): "up" direction in body frame (used for trim)
  - `residual_EnvNormalWorld` (default `0 0 1`): environment normal in world frame (e.g., gravity direction for swimming)
- Includes `turtle_3dswimming_model.xml`

**`turtle_3dswimming_model.xml`** — Include wrapper
- Combines `turtle_3dswimming_body.xml` + `turtle_3dswimming_actuators.xml`

**`turtle_3dswimming_body.xml`** — Robot kinematics and geometry
- 10 revolute joints (3 per front flipper chain, 2 per rear flipper pair)
- Sites: `base_cog`, `head_pos`, `tail_pos`
- Ellipsoid fluid drag on flippers
- Root body: `fr13_s105k` (free joint, 6.33 kg)

**`turtle_3dswimming_actuators.xml`** — Motor specs
- 10 velocity-controlled actuators, forcerange ±12 N, ctrlrange ±4.3 rad/s, kv=10

---

## Cost Weights (task.xml)

| Residual | Name | Weight | Description |
|----------|------|--------|-------------|
| [0-2] | Position | 2.0 | 3D vector error (head → target) |
| [3] | Progress | 8.0 | One-sided forward progress floor |
| [4-6] | Slip | 3.0 | Cross-track / slip velocity vector |
| [7-9] | Align | 2.5 | Locomotion-axis alignment cross-product |
| [10-12] | Trim | 1.0 | Trim relative to environment normal |
| [13-15] | AngVel | 0.5 | Angular-rate damping orthogonal to environment |
| [16] | Control | 0.01 | Scalar control effort (all actuators equal) |

---

## Customization for Other Morphologies

To adapt this controller to a different robot (e.g., crab, quadruped, biped), only change the three axis parameters in `task.xml`:

```xml
<!-- For turtle (default) -->
<numeric name="residual_LocomotionAxisBody" data="0 -1 0" />
<numeric name="residual_BodyUpAxisBody"     data="0 0 1" />

<!-- For crab (side-facing) -->
<numeric name="residual_LocomotionAxisBody" data="1 0 0" />  <!-- Claws point forward -->
<numeric name="residual_BodyUpAxisBody"     data="0 0 1" />

<!-- For biped (walking on ground, Z is up) -->
<numeric name="residual_LocomotionAxisBody" data="0 1 0" />  <!-- Forward walking -->
<numeric name="residual_BodyUpAxisBody"     data="0 0 1" />
<numeric name="residual_EnvNormalWorld"     data="0 0 1" />
```

The cost function logic automatically transforms these axes into world frame and computes alignment/trim relative to them, regardless of robot morphology.

---

## Quick Start

### 1. Copy Files to MJPC
```bash
mkdir -p mujoco_mpc/mjpc/tasks/turtle_3dswimming
cp turtle_3dswimming.{h,cc} task.xml turtle_3dswimming_*.xml \
   mujoco_mpc/mjpc/tasks/turtle_3dswimming/
```

### 2. Copy Mesh Assets
```bash
# Copy from the original TurtleReal3D task
cp -r /path/to/turtle_real3d/assets/ mujoco_mpc/mjpc/tasks/turtle_3dswimming/
```

### 3. Register Task
Edit `mjpc/tasks/task_factory.cc`:
```cpp
#include "mjpc/tasks/turtle_3dswimming/turtle_3dswimming.h"
// Add to factory: std::make_unique<Turtle3DSwimming>()
```
Edit `mjpc/tasks/CMakeLists.txt` to add `turtle_3dswimming/turtle_3dswimming.cc`.

### 4. Build and Run
```bash
cd build-clean
cmake --build . --target mjpc -j$(sysctl -n hw.logicalcpu)
cmake --build . --target copy_resources
./bin/mjpc  # Select "Turtle3DSwimming" from UI
```

### 5. Tune Parameters (if needed)
Edit `turtle_3dswimming/task.xml` to adjust:
- `residual_DesiredSpeed`: Increase for faster swimming, decrease for slower
- `residual_SlowRadius`: Distance at which to start decelerating near goal
- Cost weights in `<user>` elements to balance different objectives

---

## Implementation Notes

- **Head-based targeting:** The controller uses `head_pos_task` sensor (not `base_pos_task`) to compute target distance and direction, allowing the "nose" of the robot to aim at the goal
- **Body-frame axes:** `residual_LocomotionAxisBody` and `residual_BodyUpAxisBody` are defined in the robot's body frame and transformed to world frame using the root body's rotation matrix
- **No hard quaternion constraint:** Unlike older versions, this does not enforce full quaternion matching; soft cross-product residuals allow natural attitude variations
- **Angular velocity feedback:** Uses `base_angvel_world_task` sensor to damp unwanted tumbling perpendicular to the environment normal


## Tuning

### Speed (distance-adaptive)
```cpp
// turtle_3dswimming.cc
const double vmax = 0.55;  // max approach speed (m/s)
const double d0   = 0.6;   // distance at which speed = 0.55 * tanh(1) ≈ 0.46 m/s
```

### Cost Weights
```xml
<!-- task.xml — increase Velocity weight for more aggressive chasing -->
<user name="Velocity" dim="1" user="0 6.0 0 1.0" />

<!-- Increase Yaw weight for tighter heading hold -->
<user name="Yaw" dim="3" user="0 2.5 0 10.0" />

<!-- Roll trim: 0.0 disables, 0.8 is strong trim -->
<user name="Height" dim="1" user="0 0.5 0 100.0" />
```

---

## Dependencies & Assets

- **MuJoCo 3.x**, **C++17**, **CMake 3.15+**
- Mesh files referenced in `turtle_3dswimming_body.xml` must be provided separately (copy `assets/` from the original TurtleReal3D project)
- Water physics and lighting inherited from `common.xml` in the parent MJPC project

---

## Known Issues

| Issue | Solution |
|-------|----------|
| Robot floating motionless | Check `forcerange` in actuators.xml — must be ±12 N |
| Asymmetric flipper motion | Verify `shoulderR1` has `axis="0 0 -1"` in body.xml |
| Drifting sideways | Increase Yaw weight (2.5 → 3.5) |
| Rolls excessively | Increase Height weight (0.5 → 0.8) |
