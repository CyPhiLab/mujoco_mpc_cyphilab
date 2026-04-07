# AI Coding Agent Instructions for MuJoCo MPC

## Architecture Overview

**MuJoCo MPC (MJPC)** is a predictive control framework with a multi-layered architecture:

- **Core**: `mjpc/agent.{cc,h}` - central orchestrator that manages planners, estimators, and tasks
- **Planners**: `mjpc/planners/` - modular optimization algorithms (iLQG, gradient descent, sampling)
- **Tasks**: `mjpc/tasks/` - robotics scenarios with cost functions defined as weighted sensor residuals
- **Estimators**: `mjpc/estimators/` - state estimation algorithms
- **gRPC Services**: `mjpc/grpc/` - remote control and Python bindings

## Project Objective

**Current Goal:** Incorporate the SpiRob robot (a multi-segment continuum robot with tendon actuation) into an MJPC task.

- **Robot Model**: Located in `/Users/zach/git-repos/spirob_mujoco/robot/spirob.xml`
- **Robot Features**: 24-segment continuum robot with 3 tendon actuators and spatial tendon routing
- **Integration Target**: Create a new task in `mjpc/tasks/spirob/` following the standard task development pattern
- **Key Challenge**: Design appropriate cost functions and sensors for continuum robot locomotion/manipulation

**Implementation Plan:**
1. **Task Setup**: Create `mjpc/tasks/spirob/` directory with `spirob.{cc,h}` and `task.xml`
2. **Model Integration**: Copy SpiRob model from external repo using CMake (following external dependency pattern)
3. **Basic Task Goal**: Implement tip-to-target reaching task where robot tip touches a specified target position
4. **Cost Function Design**: Define residuals for tip position error, tendon usage, and stability
5. **Sensor Requirements**: Add sensors for tip position, tendon states, and target tracking
6. **Registration**: Update `mjpc/tasks/tasks.cc` and `mjpc/tasks/CMakeLists.txt`

## Build System & Workflow

**Essential Commands:**
```bash
# Build release (Linux requires clang-12)
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release -DMJPC_BUILD_GRPC_SERVICE=ON
cmake --build . --config=Release

# Run tests
cd build/mjpc/test && ctest --output-on-failure .

# GUI application
cd build/bin && ./mjpc
```

**Key CMake Options:**
- `MJPC_BUILD_GRPC_SERVICE=ON` - enables Python bindings (large gRPC dependency)
- Platform-specific compiler requirements in CI workflows

## Task Development Pattern

**Core Abstraction:** Tasks inherit from `mjpc::Task` and implement cost functions as weighted residual sums.

**Task Implementation Template:**
```cpp
class MyTask : public Task {
  std::string Name() const override { return "MyTask"; }
  std::string XmlPath() const override { return GetModelPath("mytask.xml"); }
  
  class ResidualFn : public BaseResidualFn {
    void Residual(const mjModel*, const mjData*, double* residual) const override;
  };
};
```

**Critical Pattern:** Residuals map to MuJoCo sensors - costs are `sum(weight_i * norm(sensor_i))`.

**File Structure:**
- `mjpc/tasks/mytask/mytask.{cc,h}` - task implementation
- `mjpc/tasks/mytask/task.xml` - MuJoCo model definition
- Register in `mjpc/tasks/tasks.cc` and `mjpc/tasks/CMakeLists.txt`

## Planner Integration

**Key Interfaces:**
- Base class: `mjpc/planners/planner.h`
- Policy representation: `mjpc/planners/policy.h`
- Cost computation: `mjpc/planners/cost_derivatives.{cc,h}`

**Adding New Planners:**
1. Inherit from `Planner` base class
2. Implement `OptimizePolicy()` method
3. Register in `mjpc/planners/include.{cc,h}`

## Code Conventions

**Google Style Requirements:**
- Use `clang-format` with Google style
- Run `cpplint` for additional C++ checks  
- Python: use `pyink --pyink-indentation 2 --line-length 80`
- Include copyright header in all files

**Naming Patterns:**
- Classes: `PascalCase` 
- Functions/methods: `PascalCase`
- Files: lowercase with underscores
- Headers: `#ifndef MJPC_PATH_FILE_H_`

## Testing Strategy

**Test Organization:**
- Unit tests: `mjpc/test/` directory structure mirrors source
- Use GoogleTest framework
- Run via `ctest` in build directory
- Tests are automatically built unless disabled

**Common Test Patterns:**
- Gradient verification for planners
- Residual function validation for tasks
- Service API testing for gRPC components

## External Dependencies & Assets

**For Third-party Models (MuJoCo Menagerie, dm_control):**
- DO NOT include XML/assets directly in task directories
- Use CMake to copy models at build time  
- Apply patches for modifications: `diff -u original.xml modified.xml > model.xml.patch`
- Example: `mjpc/tasks/op3/op3.xml.patch`

## Adding a New Task/Simulator — Required Changes Checklist

When integrating a new robot or simulator, every step below is mandatory. Missing any will prevent the task from building or appearing in the GUI.

### 1. Source Files — `mjpc/tasks/<taskname>/`

| File | Purpose |
|---|---|
| `<taskname>.h` | Class declaration inheriting `mjpc::Task`; declares `ResidualFn` inner class |
| `<taskname>.cc` | Implements `Name()`, `XmlPath()`, and `ResidualFn::Residual()` |
| `task.xml` | MuJoCo model with `<custom>` agent params, `<sensor>` cost sensors, and optional `<keyframe>` |
| `<model>.xml.patch` *(if external model)* | Minimal patch applied to the upstream XML at build time |

**Header guard pattern:** `#ifndef MJPC_TASKS_<TASKNAME>_<TASKNAME>_H_`

**`XmlPath()` pattern:**
```cpp
std::string MyTask::XmlPath() const {
  return GetModelPath("<taskname>/task.xml");
}
```

### 2. `task.xml` Required Elements

```xml
<mujoco model="...">
  <include file="../common.xml"/>          <!-- required for shared defaults -->

  <custom>
    <!-- agent config -->
    <numeric name="agent_planner"    data="1"  />
    <numeric name="agent_horizon"    data="2.0"/>
    <numeric name="agent_timestep"   data="0.02"/>
    <numeric name="sampling_trajectories"  data="64"/>
    <numeric name="sampling_spline_points" data="5" />
    <numeric name="sampling_exploration"   data="0.3"/>
    <!-- target / residual params go here too -->
    <numeric name="residual_Target"  data="x y z"/>
    <!-- estimator -->
    <numeric name="estimator" data="0"/>
  </custom>

  <sensor>
    <!-- COST sensors first and sequential — order must match Residual() indexing -->
    <user name="My Residual" dim="N" user="0 weight cutoff norm"/>
    <!-- monitoring sensors follow -->
    <framepos name="tip_position" objtype="site" objname="..."/>
  </sensor>
</mujoco>
```

- `user` sensor attribute format: `"cutoff_index weight_value norm_lower norm_upper"`
- Commented-out cost sensors are fine for iteration, but sensor count must match `residual` array size in `ResidualFn::Residual()`.

### 3. Register the Task — `mjpc/tasks/tasks.cc`

```cpp
// Add include at top
#include "mjpc/tasks/<taskname>/<taskname>.h"

// Add entry in GetTasks() vector
std::make_shared<MyTask>(),
```

### 4. Add to Library Sources — `mjpc/CMakeLists.txt`

In the `add_library(libmjpc ...)` block, add:
```cmake
tasks/<taskname>/<taskname>.cc
tasks/<taskname>/<taskname>.h
```

### 5. Copy/Patch Assets — `mjpc/tasks/CMakeLists.txt`

Inside the `add_custom_target(copy_task_resources ...)` block, add `COMMAND` entries **before** the closing `COMMENT` line:

```cmake
# Copy robot model from external source
COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/../<external_repo>/robot/<model>.xml
        ${CMAKE_CURRENT_BINARY_DIR}/<taskname>/<model>.xml
COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/../<external_repo>/robot/assets
        ${CMAKE_CURRENT_BINARY_DIR}/<taskname>/assets
# Apply patch to produce modified XML
COMMAND patch -o ${CMAKE_CURRENT_BINARY_DIR}/<taskname>/<model>_modified.xml
        ${CMAKE_CURRENT_BINARY_DIR}/<taskname>/<model>.xml
        <${CMAKE_CURRENT_SOURCE_DIR}/<taskname>/<model>.xml.patch
```

If the model is bundled (not external), use `configure_file` or a direct `copy` instead of `patch`.

### 6. Generating a Patch File

```bash
diff -u original.xml modified.xml > mjpc/tasks/<taskname>/<model>.xml.patch
```

Only patch what MJPC needs to change (e.g., model name, compiler flags). Keep patches minimal.

### 7. Residual / Sensor Alignment Rules

- The number and order of `<user>` sensors in `task.xml` must exactly match the indices written in `ResidualFn::Residual()`.
- `SensorByName(model, data, "name")` returns a pointer into `data->sensordata` — the named sensor must exist in the XML.
- Cost sensors use `dim` to declare how many `double` values they consume in the `residual[]` array.

### Quick Validation Checklist

- [ ] `tasks.cc` includes the header and adds `std::make_shared<MyTask>()`
- [ ] `mjpc/CMakeLists.txt` lists `.cc` and `.h` in `libmjpc`
- [ ] `tasks/CMakeLists.txt` copies/patches all model files before the `COMMENT` line
- [ ] `task.xml` includes `../common.xml`
- [ ] Sensor count in XML matches residual array writes in `ResidualFn::Residual()`
- [ ] Header guard follows `MJPC_TASKS_<TASKNAME>_<TASKNAME>_H_` pattern
- [ ] Copyright header present in `.cc` and `.h`

## Key Development Entry Points

- **New tasks**: Start with `mjpc/tasks/cartpole/` as reference
- **New planners**: Study `mjpc/planners/sampling/` for simplest example
- **Python integration**: Examine `mjpc/grpc/agent_service.cc`
- **GUI features**: Check `mjpc/app.cc` and `mjpc/simulate.cc`

## Critical Implementation Notes

- **Thread Safety**: Agent runs planners in background threads - use proper synchronization
- **Memory Management**: MuJoCo objects require careful lifetime management
- **Sensor Integration**: Task residuals must correspond to XML sensor definitions
- **Risk-sensitive Costs**: Support exponential cost transformations via parameter `R`