# PaddleCrabSwimming Task for MJPC — Rear-Paddle-Dominant Diagnostic Version

This is an 18-control paddle crab swimming task configured for **rear-paddle-dominant diagnostic testing**.

## Diagnostic Purpose

This configuration is designed to test whether the rear paddle mechanism can produce useful forward propulsion when front and center legs are heavily discouraged.

**Note**: This is not the final controller. It is a diagnostic test to answer the question: "Can the rear paddles alone generate forward swimming?"

- The robot is a 6-legged (18 DOF) paddle crab, swimming in fluid.
- The task uses a directional target-tracking MPC objective. The robot's forward direction is defined by the vector from base_cog to front_point.
- The cost structure is general and does not use CPG, DMP, phase, amplitude, or hand-coded gait timing.
- The objective is emergent MPC swimming using an unbiased MPC controller with diagnostic morphology-aware regularization.

## Files
- `paddle_crab_swimming.h` / `paddle_crab_swimming.cc`: Task implementation
- `task.xml`: Task configuration and cost weights
- `model.xml`: Includes the cross-domain robot
- `patch_paddle_crab_robot.py`: Patches robot.xml for MJPC compatibility

## Diagnostic Task Parameters

| Parameter | Value | Reason |
|-----------|-------|--------|
| `agent_horizon` | 1.0 s | Medium lookahead for planning |
| `agent_timestep` | 0.02 s | Standard integration step |
| `sampling_trajectories` | 256 | High-quality sampling for diagnostics |
| `sampling_spline_points` | 8 | Standard trajectory spline resolution |
| `sampling_exploration` | 0.06 | Reduced for smoother, more focused motion |
| `gradient_spline_points` | 20 | Standard gradient computation resolution |
| `residual_DesiredSpeed` | 0.45 m/s | Moderate target speed for diagnostic observation |
| `residual_SlowRadius` | 0.20 m | Distance threshold for speed scaling |
| `LocAxis` | (0, 1, 0) | Fallback only; body +Y |
| `UpAxis` | (0, 0, 1) | Body +Z (upright) |
| `EnvNormal` | (0, 0, 1) | World +Z (height) |
| `ControlSmoothingEnabled` | 1 | Enable control rate smoothing |
| `ControlSmoothingTau` | 0.18 s | Time constant for smoothing |

## Diagnostic Residual Weights

| Residual | Dimension | Weight | Purpose |
|----------|-----------|--------|---------|
| Position | 3 | 4.0 | Track target location |
| Progress | 1 | 7.0 | Penalize speeds below 0.45 m/s (moderate urgency) |
| Slip | 3 | 2.0 | Penalize lateral velocity w.r.t. target |
| Align | 3 | 0.7 | Weak heading alignment using front_point |
| Trim | 3 | 0.2 | Stabilize body upright |
| AngVel | 3 | 0.5 | Damp rotation |
| Control | 1 | 0.08 | Morphology-aware scalar effort |
| ControlRate | 18 | 1.0 | Per-actuator rate penalty (uniform) |

**Total residuals: 35 (17 fixed + 18 ControlRate for 18-control hexapod)**

## Diagnostic Morphology-Aware Regularization

This diagnostic configuration intentionally makes front and center legs very expensive while keeping rear paddles permitted but not free.

### ControlRegularizationWeight (control effort penalty)
- **Front legs R1_*, L1_***: 5.0 (very expensive)
- **Center legs R2_*, L2_***: 3.0 (expensive)
- **Rear support joints R3_1/2, L3_1/2**: 1.0 (moderate)
- **Rear distal paddle joints R3_3, L3_3**: 0.7 (cheapest, but not free)

### ControlRateRegularizationWeight (acceleration penalty)
- **Front legs R1_*, L1_***: 5.0 (very expensive)
- **Center legs R2_*, L2_***: 3.0 (expensive)
- **Rear support joints R3_1/2, L3_1/2**: 1.0 (moderate)
- **Rear distal paddle joints R3_3, L3_3**: 1.0 (uniform penalty to discourage twitching)

**Residual equations**:
- `residual[16] = sqrt(sum_i w_i * ctrl[i]^2)` where w_i is ControlRegularizationWeight(i)
- `residual[17+i] = sqrt(rate_weight[i]) * (ctrl[i] - prev_ctrl[i])`

## Actuator Configuration

### Actuator Stiffness
- **Position actuator kp = 50**: Middle ground between responsiveness and smoothness for diagnostic testing

### Actuator Range Limits
- Front legs R1/L1: ±0.9
- Center legs R2/L2: ±1.0
- Rear support joints R3_1/2, L3_1/2: ±1.2
- Rear distal paddle joints R3_3, L3_3: ±1.5

## Diagnostic Sites and Sensors

The patched robot includes diagnostic visualization sites:
- **base_cog**: Hidden sensor site at body origin (red, invisible)
- **front_point**: Visible green sphere at body +Y (0, 0.20, 0), defines forward direction

The task keeps only the core body sensors needed for control. No paddle-tip diagnostic sensors are used.

## What This Tests

If the rear paddles produce forward motion:
- The optimizer finds an efficient way to use only rear distal paddle actuators (indices 8, 17) for propulsion
- Rear support joints (indices 6, 7, 15, 16) may provide stability
- Front and center legs remain nearly inactive

If the rear paddles fail to produce motion:
- The robot will remain stationary or move poorly
- This indicates the rear paddle geometry or fluid dynamics require higher-level assistance (gaits, symmetry, or other constraints)

## Important Constraints

This diagnostic configuration maintains the original emergent MPC paradigm:
- **No CPG** (central pattern generator)
- **No DMP** (dynamical movement primitives)
- **No phase variables**
- **No sine-wave references**
- **No amplitude references**
- **No hand-coded trajectories**
- **No RearPairSymmetry** (rear paddles can move asymmetrically)

The MPC controller is **completely unstructured** aside from the morphology-aware regularization weights. If rear paddles work, it is because of emergent coordination, not prescribed motion.

## Expected Observations

### Success Indicators
- Robot moves forward smoothly in the direction of the target
- Rear paddle tips show coordinated oscillation (visible via diagnostic sites)
- Front and center leg actuators remain near zero
- Desired speed (0.45 m/s) is approximately achieved

### Failure Indicators
- Robot remains stationary
- Robot moves erratically or tumbles
- Rear paddles produce no useful motion
- All legs activate equally (optimization rejects rear-paddle constraint)

## Next Steps (If Diagnostic Succeeds)

If rear paddles prove capable of forward swimming:

1. **Reduce front/center penalties** to allow steering and stabilization
2. **Increase desired speed** to test faster swimming
3. **Adjust Align weight** to test turning capability
4. **Remove diagnostic sites** from the final controller

If rear paddles cannot produce forward swimming alone:

1. **Reconsider morphology**: Check paddle geometry, attachment points, or fluid parameters
2. **Add constraints**: Reintroduce rear-paddle symmetry or other structure (as separate diagnostic)
3. **Modify task**: Test with different desired speeds, exploration rates, or controller horizons

## Usage

Register this task in `tasks.cc` and `CMakeLists.txt` to appear in the task selection bar:

```cpp
// In tasks.cc
tasks.push_back(std::make_unique<PaddleCrabSwimming>());

// In CMakeLists.txt
list(APPEND MJPC_TASK_NAMES "PaddleCrabSwimming")
```

Run the simulator with:

```bash
./bin/mjpc --task PaddleCrabSwimming
```

---

*This diagnostic version was created to test rear-paddle-dominant emergent swimming without gait generators. All code is MPC-based with morphology-aware regularization only.*
