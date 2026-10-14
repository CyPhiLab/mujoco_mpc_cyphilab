# PaddleCrabSwimming

Straight-line swimming task for the cross-domain hexapod (18 controls: 6 legs x
3 joints) whose two rear legs end in flat paddles. The robot swims in still
water with gravity disabled.

**There is no scripted gait.** The cost describes only where the robot should
end up — moving forward, on the start line, level and on depth — and the MJPC
planner discovers a stroke that gets it there. The legs are idle at reset and
start stroking once planning is enabled.

Forward is world **-Y**, the side the rear paddles trail from, marked by the
green `front_point` site.

## Getting it out of the do-nothing minimum

At rest every residual is zero, so the total cost is exactly zero and any
perturbation looks like a loss. Two things decide whether the planner ever
leaves that minimum, and both were measured rather than guessed:

- **Forward must be a linear reward, not a target.** It uses the Null norm on
  `-(v . forward)`, so its gradient never vanishes and any sample that drifts
  forward scores better. A quadratic term around a desired speed flattens out
  and gives random search nothing to climb.
- **The lead terms must stay small.** Each steering residual is
  `error + lead * d(error)/dt`. A lead multiplies angular velocity, and a
  swimming stroke unavoidably rocks and yaws inside every cycle. With
  `LeadPath = 1.5` the planner reported *zero* of 60 samples better than doing
  nothing (`nominal=0.0000 best=0.0000 worst=+3.04`) and the robot never moved.
  At `LeadPath = 0.3` it was 26 of 60, and swimming appeared immediately.

Raising `Lateral`/`Heading` above ~5 buys no extra straightness and costs about
a third of the speed, for the same reason.

## Files

- `paddle_crab_swimming.h` / `.cc` — task, residual, control smoothing law
- `task.xml` — planner settings, cost weights, lead times, smoothing tau
- `model.xml` — includes the patched robot
- `robot_xdomain.xml` — source robot model
- `patch_paddle_crab_robot.py` — swimming physics, reference sites, actuator
  ranges, `dyntype="user"` servos
- `plot_position.py` / `plot_cost.py` — plot the CSV logs written each run

## Cost terms

| Term | Dim | Norm | Weight | Meaning |
|---|---|---|---|---|
| Forward | 1 | Null | 30 | `-(v . forward)`, a linear speed reward |
| Lateral | 1 | quadratic | 5 | offset from the start line, led by sideways velocity |
| Heading | 1 | quadratic | 5 | yaw of the body x-axis, led by yaw rate |
| Level | 2 | quadratic | 6 | tilt of the body z-axis, led by angular velocity |
| Depth | 1 | quadratic | 8 | height deviation from spawn, led by vertical velocity |
| Effort | 18 | quadratic | 0.02 | actuator commands |

`LeadPath = 0.3 s`, `LeadAttitude = 0.2 s`, both exposed as sliders.

## Planner settings

Sampling planner, horizon 1.2 s, planning timestep 0.02 s, 60 trajectories,
9 spline knots, exploration 0.3. The horizon has to span a full stroke cycle,
otherwise an oscillation looks like wasted effort: the planner sees the leg move
away and never sees the thrust it earns coming back.

Benchmarked at an equal wall-clock budget (planner running asynchronously while
physics runs in real time), 30 and 120 trajectories perform the same as 60 —
more samples per iteration simply buys proportionally fewer iterations. The
Cross Entropy planner is much worse here (0.022 m/s); its elite averaging
cancels out the oscillation that makes the stroke work.

## Control smoothing

Each servo setpoint passes through a first-order lag. The law lives in
`paddle_crab_swimming.cc`, which installs `mjcb_act_dyn`:

```
da/dt = (ctrl - a) / tau        force = kp * (a - joint_angle)
```

The model only declares `dyntype="user"` on the actuators; the equation and its
time constant come from the task, with `tau` read from
`residual_ControlSmoothingTau` in `task.xml` so it is a live GUI slider. Because
MuJoCo integrates it inside every `mj_step` and keeps `a` in `mjData`, the
filter is identical in the simulation and in each planner rollout, and each
rollout thread carries its own filter state. `tau` is clamped to at least one
timestep, since `act` is integrated explicitly.

Swept over 30 s runs. `qacc_rms` is the RMS joint acceleration, i.e. how harsh
the motion is:

| tau (s) | speed (m/s) | qacc_rms | setpoint rate (rad/s) | mid stroke (rad) |
|---|---|---|---|---|
| 0.005 (off) | 0.051 | 1656 | 5.34 | 1.51 |
| 0.02 | 0.049 | 964 | 3.78 | 1.55 |
| 0.05 | 0.051 | 669 | 2.98 | 1.52 |
| **0.08** | **0.050-0.067** | **527** | 2.78 | 1.48 |
| 0.12 | 0.054 | 358 | 2.13 | 1.47 |
| 0.15 | 0.051 | 325 | 1.94 | 1.37 |
| 0.30 | 0.050 | 220 | 1.42 | 1.26 |

**tau = 0.08 s** is the working value. It is not a trade-off: joint acceleration
drops **~3x** versus an effectively unfiltered setpoint *and* forward speed is at
its best, because the unfiltered planner output contains high-frequency chatter
that costs energy without producing thrust. Past ~0.15 s the filter starts
eating the stroke itself - amplitude falls from 1.5 to 1.26 rad.

Note the filter smooths the setpoint the joint sees, not `ctrl`; the planner's
own command rate is unchanged at ~5 rad/s regardless of tau.

## Measured behaviour

30 s with the planner running at a realistic asynchronous cadence
(~300 ms/iteration, 0.74x realtime on 11 threads):

- forward speed **0.051 m/s**, 1.51 m travelled
- lateral drift 0.010 m, depth drift 0.053 m
- all six mid joints stroking 1.3-1.56 rad up and down, rear paddles feathering
  1.44 rad

Run-to-run spread over several 30 s runs: speed 0.045-0.067 m/s, lateral drift
0.0001-0.19 m. The discovered stroke is not left/right symmetric, so lateral
drift is a slow random walk that the planner corrects only loosely — this is the
weakest part of the emergent version and the clearest difference from a
symmetric scripted gait.

## Running

```bash
cd build
./bin/mjpc --task PaddleCrabSwimming
```

Each run writes `paddle_crab_swimming_pos_log.csv` and
`paddle_crab_swimming_cost_log.csv` to the working directory.
