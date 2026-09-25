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
