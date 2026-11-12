#!/usr/bin/env bash
# Cruise (efficient) primitives on robot v2: the walking legs held still
# (--hold_legs R1,R2,L1,L2, posture free), paddles only; about 2-3x less
# energy per metre than the sprint gaits (README, "Walking legs"). Each run
# warm-starts from the v1 cruise gait for the same motion (up/down: the v1
# sprint gait, since v1 has no usable cruise up/down) and climbs from the
# speed it already swims on v2 with the legs held. cruise -Y is in
# run_primitives_v2.sh (cruise_mY).
#
#   ./run_cruise_v2.sh all       # two chains side by side, then the mirrors
#   ./run_cruise_v2.sh chainC    # +Y, +X-Y, turn, up
#   ./run_cruise_v2.sh chainD    # +X, +X+Y, down
#   ./run_cruise_v2.sh mirrors   # cruise -X, -X-Y, -X+Y, turn right
#   ./run_cruise_v2.sh NAME      # one run, e.g. cruise_pX
#
# Settings as run_primitives_v2.sh: PY (mujoco >= 4), PROCS, MUJOCO_GL.
# Outputs: runs/v2_cruise_<name>/. Reruns resume.
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

COMMON="--robot v2 --hold_legs R1,R2,L1,L2 --starts 3 --search_iters 6 \
  --es_iters 10 --lm_iters 12 --refine_iters 4 --processes $PROCS"
STRAIGHT="--weights yaw=4"

run() {
  local name=$1; shift
  echo "$(date -u +%FT%TZ) start v2_$name" >> runs/primitives_queue.log
  local rc=0
  $PY gait_chain.py "$@" --out "runs/v2_$name" > "runs/v2_$name.out" 2>&1 \
    || rc=$?
  echo "$(date -u +%FT%TZ) done  v2_$name ($rc)" >> runs/primitives_queue.log
}

final() {  # the 1 ms gait a run ended with (max if any, else its last rung)
  local dir=runs/v2_$1 f
  f=$(ls "$dir"/step*_max_dt0.001.npz 2>/dev/null | tail -1)
  [ -n "$f" ] || f=$(ls -v "$dir"/step*_dt0.001.npz 2>/dev/null | tail -1)
  echo "$f"
}

mirrors() {
  for pair in cruise_pX:cruise_mX cruise_pXmY:cruise_mXmY \
              cruise_pXpY:cruise_mXpY cruise_turnL:cruise_turnR; do
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

one() {
  case $1 in
    cruise_pY)     # +Y, paddles trailing (v1 cruise: 0.056 m/s on v2)
      run cruise_pY --init runs/prim_cruise_pY2/step3_max_dt0.001.npz \
        --direction 0 1 0 --speed_start 0.055 --speed_growth 1.1 \
        $STRAIGHT $COMMON ;;
    cruise_pX)     # +X sideways (0.047 on v2)
      run cruise_pX --init runs/prim_cruise_pX/step5_v0.05_dt0.001.npz \
        --direction 1 0 0 --speed_start 0.045 --speed_growth 1.1 \
        $STRAIGHT $COMMON ;;
    cruise_pXmY)   # +X-Y diagonal, paddles leading (0.060 on v2)
      run cruise_pXmY --init runs/prim_cruise_pXmY/step6_max_dt0.001.npz \
        --direction 1 -1 0 --speed_start 0.058 --speed_growth 1.1 \
        $STRAIGHT $COMMON ;;
    cruise_pXpY)   # +X+Y diagonal, paddles trailing (0.045 on v2)
      run cruise_pXpY --init runs/prim_cruise_pXpY/step6_v0.046_dt0.001.npz \
        --direction 1 1 0 --speed_start 0.043 --speed_growth 1.1 \
        $STRAIGHT $COMMON ;;
    cruise_turnL)  # turn on the spot (0.36 rad/s on v2)
      run cruise_turnL --init runs/prim_cruise_turnL/step7_w0.5_dt0.001.npz \
        --yaw_ladder 0.35 0.4 0.45 0.5 --weights yaw=8 $COMMON ;;
    cruise_up)     # experimental: v1 sprint up with the legs held (0.025)
      run cruise_up --init runs/prim_up/step9_max_dt0.001.npz \
        --direction 0 0 1 --speed_start 0.025 --speed_growth 1.15 \
        $STRAIGHT $COMMON ;;
    cruise_down)   # experimental: v1 sprint down with the legs held (0.039)
      run cruise_down --init runs/prim_down/step10_v0.0781_dt0.001.npz \
        --direction 0 0 -1 --speed_start 0.035 --speed_growth 1.15 \
        $STRAIGHT $COMMON ;;
    *) echo "unknown run $1" >&2; exit 2 ;;
  esac
}

chainC() { for p in cruise_pY cruise_pXmY cruise_turnL cruise_up; do one $p; done; }
chainD() { for p in cruise_pX cruise_pXpY cruise_down; do one $p; done; }

case ${1:-} in
  all) chainC & chainD & wait; mirrors ;;
  chainC) chainC ;;
  chainD) chainD ;;
  mirrors) mirrors ;;
  "") echo "usage: $0 all|chainC|chainD|mirrors|NAME" >&2; exit 2 ;;
  *) one "$1" ;;
esac
