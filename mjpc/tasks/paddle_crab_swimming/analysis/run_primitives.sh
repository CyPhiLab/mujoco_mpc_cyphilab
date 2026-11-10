#!/usr/bin/env bash
# Gait primitives for every swim direction and for turning, on the robot as
# built (XW430 servos at 12 V, the task's command ranges).
#
#   ./run_primitives.sh chain1   # diagonals and +X
#   ./run_primitives.sh chain2   # turning, up, down
#   ./run_primitives.sh NAME     # one primitive (see the case below)
#
# Each run takes 2-4 h on 2 cores; run the two chains side by side on 4.
# Reruns resume from finished steps (a container restart kills a run: just
# rerun the same command). The left/right mirror images (-X, the -X
# diagonals, turning right) need no runs: mirror_primitives.py makes them.
set -euo pipefail
cd "$(dirname "$0")"
PY=${PY:-/home/user/venv312/bin/python}
export MUJOCO_GL=${MUJOCO_GL:-osmesa}

COMMON="--starts 3 --search_iters 6 --es_iters 10 --lm_iters 12 \
  --refine_iters 4 --processes 2"
# Known gaits (and their mirror images) join the random search seeds.
LIB="runs/ladder_mY/step10_max_dt0.001.npz runs/ladder_pY/step8_max_dt0.001.npz \
  runs/ladder_pX/step4_max_dt0.001.npz"

run() {
  local name=$1; shift
  echo "$(date -u +%FT%TZ) start $name" >> runs/primitives_queue.log
  local rc=0
  $PY gait_chain.py "$@" --out "runs/prim_$name" > "runs/prim_$name.out" 2>&1 \
    || rc=$?
  echo "$(date -u +%FT%TZ) done  $name ($rc)" >> runs/primitives_queue.log
}

one() {
  case $1 in
    pXmY)  # diagonal: right side and paddles leading
      run pXmY --direction 1 -1 0 --speed_start 0.06 \
        --seed_style random+wide+library --seed_files $LIB $COMMON ;;
    pXpY)  # diagonal: right side leading, paddles trailing
      run pXpY --direction 1 1 0 --speed_start 0.06 \
        --seed_style random+wide+library --seed_files $LIB $COMMON ;;
    pX)    # sideways, right legs leading (re-search with the library)
      run pX --direction 1 0 0 --speed_start 0.06 \
        --seed_style random+wide+crab+library --seed_files $LIB $COMMON ;;
    turnL) # turn on the spot, counterclockwise seen from above
      run turnL --yaw_ladder 0.1 0.15 0.2 0.3 0.4 0.5 0.65 0.8 1.0 \
        --seed_style random+wide+library+splice --seed_files $LIB $COMMON ;;
    up)
      run up --direction 0 0 1 --speed_start 0.04 \
        --seed_style random+wide+library --seed_files $LIB $COMMON ;;
    down)
      run down --direction 0 0 -1 --speed_start 0.04 \
        --seed_style random+wide+library --seed_files $LIB $COMMON ;;
    *) echo "unknown primitive $1" >&2; exit 2 ;;
  esac
}

case ${1:-} in
  chain1) for p in pXmY pXpY pX; do one $p; done ;;
  chain2) for p in turnL up down; do one $p; done ;;
  "") echo "usage: $0 chain1|chain2|NAME" >&2; exit 2 ;;
  *) one "$1" ;;
esac
