#!/usr/bin/env bash
# Gait primitives for every swim direction and for turning, on the robot as
# built (XW430 servos at 12 V, the task's command ranges).
#
#   ./run_primitives.sh chain1   # diagonals and +X
#   ./run_primitives.sh chain2   # turning, up
#   ./run_primitives.sh chain3   # down
#   ./run_primitives.sh NAME     # one primitive (see the case below)
#
# Each run takes 2-4 h on 2 cores. Reruns resume (finished steps are
# reloaded, and the step in progress continues from its last LM/ES iteration
# checkpoint), so after a container restart just rerun the same commands. The left/right mirror images (-X, the -X
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
    turnL_w1) # turn on the spot, counterclockwise seen from above: search
      # (with the default yaw weight 1 the ladder stalls short of 0.1 rad/s)
      run turnL_w1 --yaw_ladder 0.1 0.15 0.2 0.3 0.4 0.5 0.65 0.8 1.0 \
        --seed_style random+wide+library+splice --seed_files $LIB $COMMON ;;
    turnL) # the turning ladder from that search, yaw target weighted like
      # a speed target
      run turnL --init runs/prim_turnL_w1/step2_w0.1_dt0.002.npz \
        --yaw_ladder 0.1 0.15 0.2 0.3 0.4 0.5 0.65 0.8 1.0 \
        --weights yaw=8 $COMMON ;;
    up)
      run up --direction 0 0 1 --speed_start 0.04 \
        --seed_style random+wide+library --seed_files $LIB $COMMON ;;
    down)
      run down --direction 0 0 -1 --speed_start 0.04 \
        --seed_style random+wide+library --seed_files $LIB $COMMON ;;
    down_rest) # down re-polished on its true limit cycle: scored from rest
      # after 10 cycles (its attitude settles slowly; the steady-start chain
      # scores too early and the gait drifts -Y at 2 cm/s), drift weighted up
      run down_rest --init runs/prim_down/step10_v0.0781_dt0.001.npz \
        --from_rest --warm 10 --speeds 0.07 --weights depth=3 \
        --patience 1 $COMMON ;;
    k5_mY) # paddles first with 5 harmonics, warm-started from the K=3 gait
      run k5_mY --init runs/ladder_mY/step10_max_dt0.001.npz --k 5 \
        --direction 0 -1 0 --speed_start 0.115 --speed_growth 1.08 \
        $COMMON --processes 4 ;;
    hold_mY) # paddles first with the walking legs held still (posture free):
      # what their motion is worth once the paddles re-optimize around them
      run hold_mY --init runs/ladder_mY/step10_max_dt0.001.npz \
        --hold_legs R1,R2,L1,L2 --direction 0 -1 0 --speed_start 0.09 \
        --speed_growth 1.1 $COMMON ;;
    energy_mY) # paddles first with an energy (cost of transport) term, down
      # a ladder of speeds: the speed-energy trade-off (2 ms; check at 1 ms
      # with primitive_report.py / leg_audit.py)
      run energy_mY --init runs/ladder_mY/step10_max_dt0.001.npz \
        --direction 0 -1 0 --speeds 0.11 0.10 0.09 0.08 --weights energy=4 \
        --speed_rtol 0.1 --patience 0 --dts 0.002 $COMMON ;;
    *) echo "unknown primitive $1" >&2; exit 2 ;;
  esac
}

case ${1:-} in
  chain1) for p in pXmY pXpY pX; do one $p; done ;;
  chain2) for p in turnL up; do one $p; done ;;  # after turnL_w1
  chain3) one down ;;
  "") echo "usage: $0 chain1|chain2|NAME" >&2; exit 2 ;;
  *) one "$1" ;;
esac
