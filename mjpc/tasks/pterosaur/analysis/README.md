# Pterosaur launch analysis

Tools for evaluating the **Launch Track** mode of the Pterosaur task: closed-loop
MJPC launches on design variants (actuator torque scale, parallel springs),
with metrics and videos.

## Files

- `launch_driver.cc`: headless MJPC driver. Runs the task in Launch Track with
  synchronous planning on a modified model and writes the trajectory to CSV.
- `run_mpc_launch.py`: builds the design variant (springs, joint limits),
  runs the driver, evaluates takeoff, renders video; `--grid` sweeps
  torque x spring.
- `render_launch.py`: offscreen renderer (CoM-tracking side camera) and
  kinematic playback of `reference/launch.npz`.

## Building the driver

The driver is not a CMake target; link it against a Release build of MJPC.
Build MJPC with **clang**. GCC + `-flto` + `-fuse-ld=lld` (the repo's flags)
produces an empty `libmujoco.so`, because lld cannot read GCC LTO objects.

```sh
CC=clang CXX=clang++ cmake -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DMJPC_BUILD_TESTS=OFF -S . -B build
ninja -C build bin/mjpc
cd build
clang++ -O3 -std=c++20 -flto -mavx -DMJSIMULATE_STATIC \
    -I.. -I. -I_deps/abseil-cpp-src -I_deps/mujoco-src/include \
    -I_deps/json-src/include \
    -c ../mjpc/tasks/pterosaur/analysis/launch_driver.cc -o launch_driver.o
clang++ -O3 -flto -fuse-ld=lld launch_driver.o -o bin/launch_driver \
    -Wl,-rpath,$PWD/lib \
    $(ninja -t commands bin/mjpc | tail -1 | grep -o "lib/libmjpc.a.*" | sed 's/ && :$//')
```

The driver loads the task from the build tree (`build/mjpc/tasks/pterosaur`),
so rebuild (`ninja bin/mjpc`) after editing the task XML or C++.

## Running

```sh
# one variant, with video (MUJOCO_GL=osmesa or egl when headless)
python run_mpc_launch.py --driver build/bin/launch_driver \
    --torque 2 --spring 500 --speed 10 \
    --iters 3 --plan_dt 0.005 --horizon 0.5 \
    --weights RefTakeoff=10,RefJointPos=0.5,RefJointVel=0.01 \
    --out launch.csv --video launch.mp4

# torque {1, 1.5, 2, 3} x spring {0, 250, 500, 1000} J grid, tuned settings
python run_mpc_launch.py --driver build/bin/launch_driver --speed 10 --grid out/
```

## Notes

- **Planning timestep:** the task's 10 ms planning timestep makes planner
  rollouts diverge under launch-level torques; use 5 ms (`--plan_dt 0.005`).
  Physics runs at 2 ms.
- **Cost weights:** the takeoff objective (`RefTakeoff`) has to outweigh joint
  tracking during the push, or the MPC follows the reference's forward
  lunge instead of launching.
- **Target speed:** the target caps the result (5 m/s target gives ~3.3 m/s,
  8 m/s gives ~5.3 m/s on 2x torque + 500 J), so use a high target to
  measure capability.
- **Springs:** parallel springs on shoulder2/hip2, with rest at the limb's
  liftoff angle. They are always on, and the run starts at the deepest crouch
  (`Ref start`), which models a latch released at push onset.
- **Joint limits:** the model has none, so the runner adds hard limits at the
  reference joint range +-0.3 rad.
- **Landing:** Launch Track has no landing phase, so runs end in a crash
  landing.

## Offline launch trajectories

Closed-loop MPC tracking of `reference/launch.npz` did not produce a clean
launch: the reference gets its speed after liftoff (from the wings), and a
short-horizon MPC could not find a coordinated ground push. Offline
trajectory optimization, from the reference's deepest crouch through
takeoff, does.

- `launch_trajopt.py`: sampling-based optimization of symmetric control
  knots (shared physics setup: 2 ms timestep, hard joint limits, damped
  rubber-pad foot contact, torque scale).
- `launch_ilqr.py`: full-horizon iLQR with smooth kinematic contact costs
  (planted limbs, clearance, pitch; takeoff velocity and spin; calm
  flight). Options: `--hand_radius` (rubber hand pad), `--vault_time`
  (single-strike vault schedule), `--foot_timeconst/--foot_dampratio`.
- `hand_load.py`: replays controls and reports hand/foot load per push
  window, to tell planted limbs from hammering ones.
- `render_launch.py npz <trajectory.npz> --slow 4`: slow-motion video.

Findings:
- At 1x torque the legs need 82-93% of their limits just to hold the
  crouch, so these use 2x torque.
- The optimizers readily exploit impacts (hand hammering, foot tapping)
  and soft contacts (MuJoCo's default contact sinks ~26 mm under a hard
  push). The damped pad contact (time constant 0.0067 s, damping ratio 3)
  sinks ~5 mm.
- A 2 cm point hand vaults with several strikes; a 4 cm pad with the
  single-strike schedule vaults with one contact per hand, but with
  6-7 kN impact peaks.

### Candidates (2x torque, no springs, 30 deg launch)

| | takeoff | notes |
|---|---|---|
| `candidates/A_plain_hands` | 4.73 m/s at 29 deg, t = 0.45 s | model hands; vault in several hand strikes; hands and feet on MuJoCo default contact (0.02, 1), hand sink 11.8 mm |
| `candidates/B_pad_vault` | 4.95 m/s at 29 deg, t = 0.43 s | 4 cm rubber hand pads, one vault contact per hand (6-7 kN peak), hand sink 6.9 mm; feet tap during the leg push |

| `candidates/C_t6_s500` | 8.16 m/s at 35 deg, t = 0.43 s | design study: 6x torque + 500 J latched springs (shoulder2/hip2), model hands; hands and feet still hammer, limbs swing in flight |

Each folder has `controls.npy` (symmetric controls, 2 ms steps) and
`trajectory.npz` (time, qpos, qvel, ctrl, contacts, CoM velocity). Replay
and check (push time 0.45 s):

```sh
# A
python hand_load.py candidates/A_plain_hands/controls.npy --foot_timeconst 0.02 --foot_dampratio 1
# B
python hand_load.py candidates/B_pad_vault/controls.npy --hand_radius 0.04
```

Re-optimize, e.g. B from the reference warm start:

```sh
python launch_ilqr.py --push_time 0.45 --hand_radius 0.04 --vault_time 0.08 --iters 150 --out B.npz
```

### Design study toward 10 m/s at 30 deg

`launch_study.py chains` (results: `study_results/chains_10ms_30deg.json`)
raises torque, then spring energy, warm-starting each step from the
previous solution with torque-equivalent controls. Cold-start iLQR per
design stalls and gave non-monotonic, inconclusive results.

| design | best takeoff | CoM energy at takeoff |
|---|---|---|
| 2x torque | 5.3 m/s | 850 J |
| 4x | 6.3 m/s | 1100 J |
| 6x / 8x | 6.5-7.1 m/s | 1170-1380 J |
| 6x + 500 J springs | 8.2 m/s at 35 deg | 1810 J |
| 6x + 1000 J springs | 7.5 m/s at 30 deg | 1510 J |
| 2000-4000 J springs | worse | |

10 m/s needs ~2650 J. Motor work grows with torque but converts to CoM
energy at only ~20-30%, and all these launches still hammer.
