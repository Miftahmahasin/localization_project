#!/usr/bin/env bash
# ============================================================================
# TAHAP B2.1 / 8c — semi-automatic seeded walk-through-centre runs.
#
# Drives ONE run's T2/T3/T4 sequence (teleport -> reliable seed -> eval -> walk)
# and repeats it NRUNS times, then aggregates with landmark_multirun.py.
#
# The LAUNCH (T1) is NOT started here — run it yourself in a separate terminal
# and leave it up (it owns the Webots GUI / YOLO / EKF), e.g.:
#   ros2 launch soccer_object_localization localization_v15_landmark.launch.py \
#       detector:=yolo use_gaze:=true use_line_heading:=false chi2_gate:=16.27
#
# Usage:
#   ./run_seeded_8c.sh [NRUNS] [PREFIX]
#   ./run_seeded_8c.sh 5 s3_run        # 5 runs -> s3_run1.csv .. s3_run5.csv
# Ctrl-C aborts cleanly (stops the gait, kills the background eval).
# ============================================================================
# NOTE: no `set -u` — sourcing the ROS setup.bash references unbound vars and
# would kill the script silently before it prints anything.

# ─── TIMING (jeda) — tune here ──────────────────────────────────────────────
# All in seconds. These are the knobs that matter for a clean run.
SETTLE_AFTER_TELEPORT=3.0   # let the robot physically settle after the teleport
                            #   jump (baseline physics wobbles) BEFORE seeding.
SETTLE_AFTER_SEED=2.0       # let the seed take: EKF reset + mirror ref commit +
                            #   a couple of fixes land, BEFORE eval/walk start.
EVAL_LEAD=1.0               # start eval this long BEFORE the walk, so it captures
                            #   the converged standing baseline (t0 = converged).
INTER_RUN_GAP=4.0           # pause between runs (gait fully stopped, topics quiet)
                            #   — the node is persistent across runs, so give it
                            #   time to settle before the next teleport.
# eval must outlive the whole walk. walk_op3 = ~SETTLE(4)+DURATION+stop; eval is
# started EVAL_LEAD before it, so: EVAL_DUR = EVAL_LEAD + 4 + DURATION + margin.
EVAL_MARGIN=12.0            # extra head-room on the eval window past the walk.

# ─── RUN PARAMS ─────────────────────────────────────────────────────────────
NRUNS="${1:-5}"
PREFIX="${2:-s3_run}"
SEED_X=-2.5 ; SEED_Y=0.0 ; SEED_YAW=0.0     # own half, facing +x
# WALK_DURATION overridable from the env so the TAHAP H degradation sweep can use
# shorter runs (e.g. WALK_DURATION=60) than the 180 s tracking-accuracy eval.
# WALK_X / WALK_ANGLE also env-overridable so the S1 turning confirmatory test can
# spin in place (WALK_X=0 WALK_ANGLE=0.15) to exercise yaw change under blackout —
# the scenario a straight walk (frozen-yaw-optimal) cannot fairly test.
WALK_X="${WALK_X:-0.012}" ; WALK_ANGLE="${WALK_ANGLE:-0.0}" ; WALK_DURATION="${WALK_DURATION:-180}"

# ─── SOURCE THE WORKSPACE (self-contained) ──────────────────────────────────
# resolve workspace root from this script's location (.../src/.../scripts)
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="/home/miftah/basbot"
SCRIPTS="$SELF"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null || true
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash" 2>/dev/null || true

EVAL_DUR="$(python3 -c "print(${EVAL_LEAD}+4.0+${WALK_DURATION}+${EVAL_MARGIN})")"

# ─── PRECHECK: is the stack up? ─────────────────────────────────────────────
echo "== precheck: is the localization stack up? =="
if ! ros2 topic list 2>/dev/null | grep -q "/odometry/filtered"; then
  echo "!! /odometry/filtered not found — launch the stack (T1) first. Aborting."
  exit 1
fi
if ! ros2 topic list 2>/dev/null | grep -q "/landmark_array"; then
  echo "!! /landmark_array not found — detector path not up. Aborting."
  exit 1
fi
echo "   OK. eval window per run = ${EVAL_DUR}s ; runs = ${NRUNS} ; prefix = ${PREFIX}"

# ─── clean shutdown on Ctrl-C ───────────────────────────────────────────────
EVAL_PID=""
cleanup() {
  echo ""
  echo "== aborting: stopping gait + eval =="
  ros2 topic pub --once /robotis/walking/command std_msgs/msg/String "{data: 'stop'}" \
      >/dev/null 2>&1 || true
  [ -n "$EVAL_PID" ] && kill "$EVAL_PID" 2>/dev/null || true
  exit 130
}
trap cleanup INT TERM

# ─── the per-run sequence ───────────────────────────────────────────────────
OUTS=()
for N in $(seq 1 "$NRUNS"); do
  OUT="${PREFIX}${N}.csv"
  OUTS+=("$OUT")
  echo ""
  echo "############################################################"
  echo "# RUN ${N}/${NRUNS}  ->  ${OUT}"
  echo "############################################################"

  echo "[T2] teleport -> (${SEED_X}, ${SEED_Y}, ${SEED_YAW} deg)"
  ros2 topic pub --once /robotis_op3/set_pose geometry_msgs/msg/Pose2D \
      "{x: ${SEED_X}, y: ${SEED_Y}, theta: $(python3 -c "import math;print(math.radians(${SEED_YAW}))")}" \
      >/dev/null 2>&1
  echo "     settle ${SETTLE_AFTER_TELEPORT}s ..."
  sleep "$SETTLE_AFTER_TELEPORT"

  echo "[T2] seed (reliable, x6) ..."
  python3 "$SCRIPTS/seed_side.py" --x "$SEED_X" --y "$SEED_Y" --yaw "$SEED_YAW"
  echo "     settle ${SETTLE_AFTER_SEED}s (EKF reset + ref commit) ..."
  sleep "$SETTLE_AFTER_SEED"

  echo "[T3] eval -> ${OUT} (dur ${EVAL_DUR}s)"
  python3 "$SCRIPTS/landmark_eval.py" --out "$OUT" --dur "$EVAL_DUR" &
  EVAL_PID=$!
  sleep "$EVAL_LEAD"

  echo "[T4] walk (x=${WALK_X}, angle=${WALK_ANGLE}, ${WALK_DURATION}s) ..."
  python3 "$SCRIPTS/walk_op3.py" --x "$WALK_X" --angle "$WALK_ANGLE" \
      --duration "$WALK_DURATION"

  echo "     walk done; waiting for eval to finish ..."
  wait "$EVAL_PID"
  EVAL_PID=""

  if [ "$N" -lt "$NRUNS" ]; then
    echo "     inter-run gap ${INTER_RUN_GAP}s ..."
    sleep "$INTER_RUN_GAP"
  fi
done

# ─── aggregate ──────────────────────────────────────────────────────────────
echo ""
echo "############################################################"
echo "# AGGREGATE"
echo "############################################################"
python3 "$SCRIPTS/landmark_multirun.py" --label "seeded_B2" "${OUTS[@]}"
echo ""
echo "done. CSVs: ${OUTS[*]}"
