# Tutorial: Implementing a New Task in MuJoCo MPC

This tutorial walks you through creating a new task from scratch in MJPC. By the end you will have a working task that appears in the GUI and can be optimized by the planner. We use a simple **ball reaching** task as the running example — a sphere that must reach a target position — because it touches every required file without extra complexity.

> **Reference implementations:** `mjpc/tasks/cartpole/` (bundled model) and `mjpc/tasks/spirob/` (external model with patch).

---

## 1. Background: How Tasks Work

MJPC tasks are defined by a **cost function** expressed as a sum of weighted, normed residuals:

$$
\text{cost} = \sum_i w_i \cdot \text{norm}_i(\mathbf{r}_i)
$$

Each residual $\mathbf{r}_i$ is a chunk of a `double* residual` array that your C++ code fills in. The weights and norm types are declared in the XML. The planner never sees your task logic directly — it only sees MuJoCo sensor values and the cost that comes out of your `Residual()` function.

**Data flow:**

```
task.xml  ──► MuJoCo model  ──► mjData (sensors, qpos, qvel, ctrl)
                                      │
                              ResidualFn::Residual()
                                      │
                              residual[] array
                                      │
                              BaseResidualFn::CostValue()  ──► planner
```

---

## 2. File Overview

You need to create or modify exactly **five** things:

| What | File |
|------|------|
| Task header | `mjpc/tasks/<name>/<name>.h` |
| Task source | `mjpc/tasks/<name>/<name>.cc` |
| MuJoCo XML | `mjpc/tasks/<name>/task.xml` |
| Library sources | `mjpc/CMakeLists.txt` |
| Task registry | `mjpc/tasks/tasks.cc` |

If your robot model comes from an external repository, you also need:

| What | File |
|------|------|
| Asset copy/patch commands | `mjpc/tasks/CMakeLists.txt` |
| Patch file(s) | `mjpc/tasks/<name>/<model>.xml.patch` |

---

## 3. Step 1 — Write the Header (`<name>.h`)

Create `mjpc/tasks/ball/ball.h`:

```cpp
// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ... (standard Apache 2.0 header) ...

#ifndef MJPC_TASKS_BALL_BALL_H_
#define MJPC_TASKS_BALL_BALL_H_

#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {

class Ball : public Task {
 public:
  std::string Name() const override;
  std::string XmlPath() const override;

  class ResidualFn : public BaseResidualFn {
   public:
    explicit ResidualFn(const Ball* task) : BaseResidualFn(task) {}
    // ------- Residuals for Ball task ------
    //   Number of residuals: 4
    //     Residual (0-2): ball position error (3D vector)
    //     Residual (3):   control effort (scalar)
    // --------------------------------------
    void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override;
  };

  Ball() : residual_(this) {}

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(this);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  ResidualFn residual_;
};

}  // namespace mjpc

#endif  // MJPC_TASKS_BALL_BALL_H_
```

**Key rules:**
- Header guard format: `MJPC_TASKS_<NAME>_<NAME>_H_`  (all caps, underscores)
- Always inherit from `Task` and declare a nested `ResidualFn : public BaseResidualFn`
- The boilerplate `ResidualLocked()` / `InternalResidual()` / `residual_` are required for thread safety — copy them verbatim, changing only the class name
- Document the number of residuals and what each index means in the comment block

---

## 4. Step 2 — Write the Source (`<name>.cc`)

Create `mjpc/tasks/ball/ball.cc`:

```cpp
// Copyright 2022 DeepMind Technologies Limited
// ... (standard Apache 2.0 header) ...

#include "mjpc/tasks/ball/ball.h"

#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"   // provides SensorByName(), mju_*

namespace mjpc {

std::string Ball::XmlPath() const {
  return GetModelPath("ball/task.xml");
}

std::string Ball::Name() const { return "Ball"; }

// ------- Residuals for Ball task ------
//     Position: ball should reach the target site
//     Control:  keep actuator effort small
// ------------------------------------------
void Ball::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                double* residual) const {
  // ---------- Position (residual indices 0, 1, 2) ----------
  // SensorByName returns a pointer into data->sensordata for the named sensor.
  double* ball_pos   = SensorByName(model, data, "ball_pos");
  double* target_pos = SensorByName(model, data, "target_pos");
  mju_sub3(residual, ball_pos, target_pos);   // residual[0..2] = ball - target

  // ---------- Control (residual index 3) ----------
  // Sum of squared controls, then square-root → L2 norm of ctrl vector
  double effort = 0.0;
  for (int i = 0; i < model->nu; i++) {
    effort += data->ctrl[i] * data->ctrl[i];
  }
  residual[3] = mju_sqrt(effort);
}

}  // namespace mjpc
```

**Key rules:**
- `XmlPath()` must return `GetModelPath("<taskname>/task.xml")` — the path is relative to the build-time binary directory
- `Name()` is the string shown in the GUI task selector
- `SensorByName(model, data, "name")` looks up a named sensor and returns a pointer into `data->sensordata`. The sensor must exist in `task.xml`
- Index into `residual[]` sequentially. The total number of entries written must match the sum of `dim` values across all `<user>` cost sensors in the XML
- Never write beyond the end of `residual[]`

---

## 5. Step 3 — Write the XML (`task.xml`)

Create `mjpc/tasks/ball/task.xml`:

```xml
<mujoco model="Ball Reaching">
  <include file="../common.xml"/>   <!-- Required: shared compiler/visual defaults -->

  <size memory="4K"/>

  <worldbody>
    <!-- the ball: a free body (6 DoF) with a sphere geom -->
    <body name="ball" pos="0 0 0.5">
      <freejoint name="ball_joint"/>
      <geom type="sphere" size="0.05" rgba="0.2 0.5 1 1" mass="0.1"/>
      <site name="ball_site" size="0.01"/>
    </body>

    <!-- target: a visual-only marker the ball must reach -->
    <body name="target" pos="1 0 0.5">
      <geom type="sphere" size="0.05" rgba="1 0.3 0.3 0.4"
            contype="0" conaffinity="0"/>
      <site name="target_site" size="0.01"/>
    </body>
  </worldbody>

  <actuator>
    <!-- three force actuators along world axes -->
    <general name="fx" site="ball_site" gear="1 0 0" forcerange="-5 5"/>
    <general name="fy" site="ball_site" gear="0 1 0" forcerange="-5 5"/>
    <general name="fz" site="ball_site" gear="0 0 1" forcerange="-5 5"/>
  </actuator>

  <custom>
    <!-- ---- Agent / planner configuration ---- -->
    <numeric name="agent_planner"          data="1"   />  <!-- 1 = sampling -->
    <numeric name="agent_horizon"          data="1.5" />
    <numeric name="agent_timestep"         data="0.02"/>
    <numeric name="sampling_trajectories"  data="64"  />
    <numeric name="sampling_spline_points" data="5"   />
    <numeric name="sampling_exploration"   data="0.3" />

    <!-- ---- Estimator ---- -->
    <numeric name="estimator" data="0"/>
  </custom>

  <sensor>
    <!-- ============================================================
         COST SENSORS — must come first, must be sequential.
         The order here must match the order residual[] is written in Residual().

         Format of the `user` attribute (5 values):
           norm_type  weight  cutoff  lower_bound  upper_bound
         Use norm_type=0 for quadratic, norm_type=6 for L2 (Euclidean).
         ============================================================ -->

    <!-- Residuals 0-2: 3D position error  (dim="3") -->
    <user name="Position" dim="3" user="6 10.0 0 100.0"/>

    <!-- Residual 3: control effort  (dim="1") -->
    <user name="Control" dim="1" user="0 0.1 0.0 1.0"/>

    <!-- ============================================================
         MONITORING SENSORS — used by Residual() via SensorByName().
         These do NOT contribute to the cost array directly.
         ============================================================ -->
    <framepos name="ball_pos"   objtype="site" objname="ball_site"/>
    <framepos name="target_pos" objtype="site" objname="target_site"/>
  </sensor>

  <keyframe>
    <!-- Initial state: ball at origin, all velocities zero, all controls zero -->
    <key name="home" qpos="0 0 0.5 1 0 0 0" ctrl="0 0 0"/>
  </keyframe>
</mujoco>
```

**Key rules:**

| Rule | Why |
|------|-----|
| `<include file="../common.xml"/>` must be first | Provides shared `<compiler>`, `<visual>`, and `<default>` settings |
| `<user>` sensors must appear before all other sensors | MJPC reads cost terms by position, not name |
| `dim` on a `<user>` sensor = number of `residual[]` slots it consumes | Must match what `Residual()` writes |
| `user` attribute = `norm_type weight cutoff lower upper` | Missing values default to 0 |
| Monitoring sensors referenced by `SensorByName()` must exist in the XML | The function returns `nullptr` if the sensor is not found — this will crash |

**Planner index reference:**

| `agent_planner` value | Algorithm |
|-----------------------|-----------|
| 0 | Gradient descent |
| 1 | Sampling (CEM) |
| 2 | iLQG |
| 3 | Sample-gradient hybrid |

---

## 6. Step 4 — Register the Task

### `mjpc/tasks/tasks.cc`

Add an include and an entry in the returned vector:

```cpp
// Near the other includes:
#include "mjpc/tasks/ball/ball.h"

// Inside GetTasks():
std::make_shared<Ball>(),
```

### `mjpc/CMakeLists.txt`

Find the `add_library(libmjpc ...)` block and add your source files alongside the other tasks:

```cmake
tasks/ball/ball.cc
tasks/ball/ball.h
```

---

## 7. Step 5 — Handle Model Assets

### Option A: Self-contained model (no external files)

If your robot/scene is entirely defined inside `task.xml` (like the ball example above), you are done — `copy_resources` in `tasks/CMakeLists.txt` copies the whole `tasks/` source tree to the binary directory automatically.

### Option B: External model (copy + patch)

If your robot comes from another repository (like SpiRob), add `COMMAND` lines inside the `add_custom_target(copy_model_resources ...)` block in `mjpc/tasks/CMakeLists.txt`, **before** the closing `COMMENT` line:

```cmake
# Copy your robot's base XML and mesh assets
COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/../my_robot_repo/robot/myrobot.xml
        ${CMAKE_CURRENT_BINARY_DIR}/ball/myrobot.xml
COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/../my_robot_repo/robot/assets
        ${CMAKE_CURRENT_BINARY_DIR}/ball/assets
# Apply patch to add MJPC-specific changes
COMMAND patch -o ${CMAKE_CURRENT_BINARY_DIR}/ball/myrobot_modified.xml
        ${CMAKE_CURRENT_BINARY_DIR}/ball/myrobot.xml
        <${CMAKE_CURRENT_SOURCE_DIR}/ball/myrobot.xml.patch
```

**Generating a patch file:**

```bash
# 1. Make a copy of the original XML
cp ../my_robot_repo/robot/myrobot.xml /tmp/myrobot_original.xml

# 2. Create your modified version with MJPC-specific changes
cp /tmp/myrobot_original.xml /tmp/myrobot_modified.xml
# ... edit /tmp/myrobot_modified.xml (e.g., change model name, add autolimits) ...

# 3. Generate the patch
diff -u /tmp/myrobot_original.xml /tmp/myrobot_modified.xml \
     > mjpc/tasks/ball/myrobot.xml.patch
```

Keep patches minimal — only change what MJPC strictly requires (e.g., model name, `autolimits="true"`, compiler flags).

---

## 8. Step 6 — Build and Run

```bash
cd /path/to/mujoco_mpc_cyphilab

# Configure (only needed once, or when CMakeLists changes)
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build

# Copy model assets (runs automatically as part of build)
# If assets are missing, force the target:
cmake --build build --target copy_model_resources

# Launch the GUI
./build/bin/mjpc
```

Select **"Ball"** from the task dropdown to verify it appears and runs.

---

## 9. Common Errors and Fixes

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Task not in GUI dropdown | Missing from `GetTasks()` in `tasks.cc` | Add `std::make_shared<Ball>()` |
| Build error: undefined `Ball` | Missing from `libmjpc` in `mjpc/CMakeLists.txt` | Add `tasks/ball/ball.cc` and `tasks/ball/ball.h` |
| MuJoCo XML error at load | `task.xml` path wrong in `XmlPath()` | Must match `GetModelPath("ball/task.xml")` exactly |
| Crash / assertion in `Residual()` | `SensorByName()` returned `nullptr` | Sensor name in C++ must match `name=` attribute in XML exactly |
| Cost is always 0 or garbage | `dim` mismatch between XML and `residual[]` writes | Count entries written; they must equal sum of all `dim` attributes on `<user>` sensors |
| Mesh/texture not found | Asset copy command missing or runs after model load | Add `COMMAND` block to `copy_model_resources` in `tasks/CMakeLists.txt` |
| XML include not found | `<include>` path is wrong | Paths in XML are relative to the binary directory, not source directory |

---

## 10. Adding Tunable Parameters

Parameters that should be adjustable from the GUI (e.g., a target position) are declared in `<custom>` and read in `Residual()` via `parameters_[]`:

**In `task.xml`:**
```xml
<custom>
  <!-- 3-element target position for the ball -->
  <numeric name="residual_Target" data="1.0 0.0 0.5"/>
</custom>
```

Any `<numeric>` whose name starts with `residual_` is automatically exposed in the GUI and loaded into the `parameters_` vector (in declaration order).

**In `ball.cc`:**
```cpp
void Ball::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                double* residual) const {
  // parameters_[0..2] = target position from XML / GUI slider
  const double* target = parameters_.data();

  double* ball_pos = SensorByName(model, data, "ball_pos");
  mju_sub3(residual, ball_pos, target);

  // ... control residual ...
}
```

---

## 11. Checklist Before Committing

- [ ] Copyright header in `.cc` and `.h`
- [ ] Header guard follows `MJPC_TASKS_<NAME>_<NAME>_H_`
- [ ] `XmlPath()` returns `GetModelPath("<name>/task.xml")`
- [ ] `task.xml` starts with `<include file="../common.xml"/>`
- [ ] `<user>` cost sensors are listed **before** all monitoring sensors
- [ ] Sum of `dim` values across `<user>` sensors equals total entries written in `Residual()`
- [ ] Every sensor name passed to `SensorByName()` exists in XML
- [ ] `tasks.cc` includes header and has `std::make_shared<Ball>()`
- [ ] `mjpc/CMakeLists.txt` lists `.cc` and `.h` in `libmjpc`
- [ ] External model assets copied in `tasks/CMakeLists.txt` (if applicable)
- [ ] Task appears in GUI and planner reduces cost when run
