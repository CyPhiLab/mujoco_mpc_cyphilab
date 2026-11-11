#!/usr/bin/env bash
# The gait primitive library re-optimized for robot v2 (robot_v2/Robot.xml:
# the collaborators' updated robot with dcmotor XW430 servos; crab_model.ROBOTS).
# Needs MuJoCo >= 4 (PY below). Every run warm-starts from the v1 primitive
# for the same motion (commands are servo setpoints in radians, so they carry
# over) and climbs a ladder from the speed that gait already swims on v2.
#
#   ./run_primitives_v2.sh all       # chainA and chainB side by side, then
#                                    # the mirror images
#   ./run_primitives_v2.sh chainA    # -Y, +X-Y, turn, up, cruise -Y
#   ./run_primitives_v2.sh chainB    # +Y, +X, +X+Y, down
#   ./run_primitives_v2.sh mirrors   # -X, -X diagonals, turn right
#   ./run_primitives_v2.sh NAME      # one run
#
# Settings (environment): PY, a Python with mujoco>=4, numpy, matplotlib,
# imageio[pyav], imageio-ffmpeg, Pillow (default: ~/venv_mj4, else python3);
# PROCS, worker processes per run (default 2; each chain runs one run at a
# time, so give each chain about half the cores); MUJOCO_GL=osmesa (or egl)
# renders a video per step, empty skips rendering.
#
# Reruns resume (finished steps reload; the step in progress continues from
# its last iteration checkpoint). Mirror images (-X, -X diagonals, turn
# right) come from mirror_primitives.py afterwards.
set -euo pipefail
cd "$(dirname "$0")"
if [ -z "${PY:-}" ]; then
  if [ -x "$HOME/venv_mj4/bin/python" ]; then PY=$HOME/venv_mj4/bin/python
  else PY=python3; fi
fi
if ! "$PY" -c 'import mujoco, sys; sys.exit(int(mujoco.__version__.split(".")[0]) < 4)' 2>/dev/null; then
  echo "PY=$PY needs mujoco >= 4 (pip install -r requirements_v2.txt)" >&2
  exit 2
fi
PROCS=${PROCS:-2}
export MUJOCO_GL=${MUJOCO_GL-osmesa}

COMMON="--robot v2 --starts 3 --search_iters 6 --es_iters 10 --lm_iters 12 \
  --refine_iters 4 --processes $PROCS"
# Straight gaits weight yaw drift 4 (the v1 gaits start out turning 0.03-0.05
# rad/s on v2; weight 1 let that stand); gait_chain's straight check also
# holds them to 0.015 rad/s. The turn sets its own yaw weight.
STRAIGHT="--weights yaw=4"

final() {  # the 1 ms gait a run ended with (max if any, else its last rung)
  local dir=runs/v2_$1 f
  f=$(ls "$dir"/step*_max_dt0.001.npz 2>/dev/null | tail -1)
  [ -n "$f" ] || f=$(ls -v "$dir"/step*_dt0.001.npz 2>/dev/null | tail -1)
  echo "$f"
}

mirrors() {  # left/right mirror images, re-polished (v2 is not exactly
             # mirror-symmetric: about 2% apart unpolished)
  for pair in pX:mX pXmY:mXmY pXpY:mXpY turnL:turnR; do
    local src=${pair%%:*} dst=${pair##*:} f
    f=$(final "$src")
    if [ -z "$f" ]; then
      echo "skipping v2_$dst: v2_$src has no 1 ms result yet" >&2
      continue
    fi
    echo "$(date -u +%FT%TZ) start v2_$dst (mirror)" >> runs/primitives_queue.log
    $PY mirror_primitives.py "$f" "runs/v2_$dst" --polish 4 \
      --processes "$PROCS" > "runs/v2_$dst.out" 2>&1
    echo "$(date -u +%FT%TZ) done  v2_$dst" >> runs/primitives_queue.log
  done
}

run() {
  local name=$1; shift
  echo "$(date -u +%FT%TZ) start v2_$name" >> runs/primitives_queue.log
  local rc=0
  $PY gait_chain.py "$@" --out "runs/v2_$name" > "runs/v2_$name.out" 2>&1 \
    || rc=$?
  echo "$(date -u +%FT%TZ) done  v2_$name ($rc)" >> runs/primitives_queue.log
}

one() {
  case $1 in
    mY)        # paddles first (K = 5)
      run mY --init runs/prim_k5_mY/step5_max_dt0.001.npz --k 5 \
        --direction 0 -1 0 --speed_start 0.125 --speed_growth 1.08 $STRAIGHT $COMMON ;;
    pY)        # paddles trailing
      run pY --init runs/ladder_pY/step8_max_dt0.001.npz \
        --direction 0 1 0 --speed_start 0.09 --speed_growth 1.1 $STRAIGHT $COMMON ;;
    pX)        # sideways, right legs leading
      run pX --init runs/ladder_pX/step4_max_dt0.001.npz \
        --direction 1 0 0 --speed_start 0.065 --speed_growth 1.1 $STRAIGHT $COMMON ;;
    pXmY)      # diagonal, right side and paddles leading
      run pXmY --init runs/prim_pXmY/step8_max_dt0.001.npz \
        --direction 1 -1 0 --speed_start 0.085 --speed_growth 1.1 $STRAIGHT $COMMON ;;
    pXpY)      # diagonal, right side leading, paddles trailing
      run pXpY --init runs/prim_pXpY/step6_max_dt0.001.npz \
        --direction 1 1 0 --speed_start 0.08 --speed_growth 1.1 $STRAIGHT $COMMON ;;
    turnL)     # turn on the spot, counterclockwise from above
      run turnL --init runs/prim_turnL/step12_max_dt0.001.npz \
        --yaw_ladder 0.35 0.4 0.45 0.5 --weights yaw=8 $COMMON ;;
    up)
      run up --init runs/prim_up/step9_max_dt0.001.npz \
        --direction 0 0 1 --speed_start 0.05 --speed_growth 1.15 $STRAIGHT $COMMON ;;
    down)
      run down --init runs/prim_down/step10_v0.0781_dt0.001.npz \
        --direction 0 0 -1 --speed_start 0.02 --speed_growth 1.25 $STRAIGHT $COMMON ;;
    cruise_mY) # paddles first, walking legs held (the efficient gait)
      run cruise_mY --init runs/prim_hold_mY/step6_max_dt0.001.npz \
        --hold_legs R1,R2,L1,L2 --direction 0 -1 0 --speed_start 0.09 \
        --speed_growth 1.08 $STRAIGHT $COMMON ;;
    *) echo "unknown run $1" >&2; exit 2 ;;
  esac
}

chainA() { for p in mY pXmY turnL up cruise_mY; do one $p; done; }
chainB() { for p in pY pX pXpY down; do one $p; done; }

case ${1:-} in
  all) chainA & chainB & wait; mirrors ;;
  chainA) chainA ;;
  chainB) chainB ;;
  mirrors) mirrors ;;
  "") echo "usage: $0 all|chainA|chainB|mirrors|NAME" >&2; exit 2 ;;
  *) one "$1" ;;
esac
