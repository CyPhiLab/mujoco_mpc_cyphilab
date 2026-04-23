# Turtle 3D Swimming Task for MJPC

Real-world turtle robot 3D target-tracking task using MuJoCo Predictive Control.

## Overview

**Turtle3DSwimming** is a fully-tuned MPC task for an autonomous underwater vehicle (turtle-inspired) built from a real Onshape CAD export. The robot swims toward a 3D target with symmetric flipper motion and a naturally level body attitude.

### Key Features
- **Smooth target-chasing:** Distance-adaptive speed via tanh — pushes hard when far, decelerates naturally near the goal
- **Symmetric locomotion:** Heading alignment via cross-product residuals using head-to-target direction
- **Weak roll trim:** Bank stabilizer keeps body attitude natural without locking orientation
- **Real robot geometry:** 6.33 kg Onshape CAD model with velocity-controlled XW540-T260 actuators

### Performance
- **Planning Horizon:** 2.0 seconds
- **Sampling:** 64 trajectories, 5 spline points
- **Approach Speed:** up to 0.55 m/s (tanh-scaled with distance)

---

## Files

**`turtle_3dswimming.h`** — Task class header
- Defines `Turtle3DSwimming` inheriting from MJPC `Task`
- Declares `ResidualFn` with 17 residuals
- `XmlPath()` points to `turtle_3dswimming/task.xml`

**`turtle_3dswimming.cc`** — Cost function implementation
- `[0-9]` Control effort: joints 2, 5 at 0.5× (front flippers); joints 7, 9 at 0.33× (rear flippers)
- `[10]` 3D Euclidean distance from base to target
- `[11]` Distance-adaptive progress: `desired_speed = 0.55 * tanh(dist / 0.6)`, residual = `desired_speed - speed_toward_target` (two-sided, no clamp)
- `[12-14]` `cross(body_axis, head_to_target_direction)` — zero when perfectly aligned
- `[15]` Weak roll/bank trim: Z-component of body right axis in world frame (~0 when level)
- `[16]` Reserved / disabled (zero)

**`task.xml`** — MJPC configuration
- Planning, sensors, target position, and cost weights
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
| [0-9] | Control | 0.01 | Joint effort (front stroke 0.5×, rear stroke 0.33×) |
| [10] | Distance | 2.0 | 3D Euclidean distance base→target |
| [11] | Velocity | 6.0 | Distance-adaptive progress (tanh, two-sided) |
| [12-14] | Yaw | 2.5 | Cross-product alignment body→target |
| [15] | Height | 0.5 | Weak roll/bank trim |
| [16] | Pitch | 0.0 | Disabled (reserved) |

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

---

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
