#!/usr/bin/env bash
# ============================================================================
# TAHAP 8b — announced RE-ENTRY recovery (deployable path), measured WALKING.
#
# A standing robot in this stack sits in a knee-bent CROUCH (the walking module
# leaves that stance after 'stop', and a bare ini_pose does not reliably take),
# which drops the camera and breaks the projector -> a frozen wrong fix. The
# stance that is PROVEN to localize well (8c: 0.208 m) is the WALKING stance.
# So 8b measures while WALKING, which is also what a match does after re-entry:
#
#   Phase A (unmeasured): teleport START -> seed -> brief hold (a prior at START;
#            may be crouched, does not matter — the re-seed overrides it).
#   Phase B: teleport KIDNAP (referee re-places the robot) -> RESEED at that pose
#            (GameController announces the re-entry).
#   Phase C (measured): eval + WALK (resume play, proper stance) -> must track TRUE.
#
# RESEED=false instead leaves out the re-seed = AUTONOMOUS recovery (hard case).
#
# The LAUNCH (T1) is NOT started here — run it separately and leave it up:
#   ros2 launch soccer_object_localization localization_v15_landmark.launch.py \
#       detector:=yolo use_gaze:=true use_line_heading:=false
#
# Usage:  ./run_kidnap_8b.sh [NRUNS] [PREFIX]     # default 5, kr_run
# Ctrl-C aborts cleanly (stops the gait, kills eval).
# ============================================================================
# NOTE: no `set -u` — sourcing ROS setup.bash references unbound vars.

# ─── TIMING (jeda) — tune here ──────────────────────────────────────────────
SETTLE_AFTER_TELEPORT=3.0   # robot settles after the START teleport before seed
SETTLE_AFTER_SEED=2.0       # a seed takes (EKF reset + mirror ref commit)
CONVERGE_HOLD=3.0           # brief hold at START (unmeasured prior; crouch is ok)
# CRITICAL: the robot lands the KIDNAP teleport CROUCHED, and a crouched robot
# spews GARBAGE fixes (wrong camera height -> yaw ~-160 deg) that DRAG the mirror
# ref across the field before the re-seed (mdiag root-cause of kr_run4). So Phase C
# starts the WALK FIRST (walk_op3 does ini_pose->walking_module = proper stance),
# waits for a CLEAN fix flow, and ONLY THEN re-seeds — so the seed lands against
# good fixes and ref is never corrupted.
RESTAND_WALK=7.0            # after starting the walk: walking_module stands + a couple
                            #   of clean walking-stance fixes BEFORE re-seed.
# WALK_DURATION is sized so walk_op3 ends on its OWN (sending a clean 'stop')
# just after the eval window — NEVER kill it (a SIGTERM raises rclpy
# ExternalShutdownException so walk_op3 dies WITHOUT stopping the gait, leaving the
# walking module wedged -> the next run starts crouched / mis-placed, which is what
# corrupted 2/5 runs). Phase C launch->eval-start is ~ini(4)+module(2)+RESTAND
# remainder+SETTLE; eval runs EVAL_DUR; walk must outlast that.
WALK_DURATION=65            # walk_op3 walking time (ends ~just after eval)
EVAL_DUR=55                 # measured window (starts right after the re-seed)
INTER_RUN_GAP=4.0           # pause between runs (node persistent across runs)

# ─── POSES ──────────────────────────────────────────────────────────────────
START_X=-2.18 ; START_Y=0.0  ; START_YAW=0.0    # own half, facing +x
KID_X=-1.0    ; KID_Y=2.5    ; KID_YAW=0.0       # SAME half (x<0) re-entry point
RESEED=true                                      # true = announced re-entry
                                                 # false = autonomous (hard)
WALK_X=0.012 ; WALK_ANGLE=0.0

# ─── SOURCE THE WORKSPACE ───────────────────────────────────────────────────
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="/home/miftah/basbot"
SCRIPTS="$SELF"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null || true
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash" 2>/dev/null || true

NRUNS="${1:-5}"
PREFIX="${2:-kr_run}"

deg2rad() { python3 -c "import math;print(math.radians($1))"; }
teleport() { ros2 topic pub --once /robotis_op3/set_pose geometry_msgs/msg/Pose2D \
      "{x: $1, y: $2, theta: $(deg2rad "$3")}" >/dev/null 2>&1; }

# ─── PRECHECK ───────────────────────────────────────────────────────────────
echo "== precheck: is the localization stack up? =="
if ! ros2 topic list 2>/dev/null | grep -q "/odometry/filtered"; then
  echo "!! /odometry/filtered not found — launch the stack (T1) first. Aborting."
  exit 1
fi
echo "   OK. RESEED=${RESEED} ; eval/run=${EVAL_DUR}s ; runs=${NRUNS} ; prefix=${PREFIX}"
echo "   START=(${START_X},${START_Y})  RE-ENTRY=(${KID_X},${KID_Y})"

EVAL_PID="" ; WALK_PID=""
stop_gait() { ros2 topic pub --once /robotis/walking/command std_msgs/msg/String \
      "{data: 'stop'}" >/dev/null 2>&1 || true; }
cleanup() {
  echo ""; echo "== aborting: stop gait + kill walk/eval =="
  stop_gait
  [ -n "$WALK_PID" ] && kill "$WALK_PID" 2>/dev/null || true
  [ -n "$EVAL_PID" ] && kill "$EVAL_PID" 2>/dev/null || true
  exit 130
}
trap cleanup INT TERM

# ─── per-run ────────────────────────────────────────────────────────────────
OUTS=()
for N in $(seq 1 "$NRUNS"); do
  OUT="${PREFIX}${N}.csv"
  OUTS+=("$OUT")
  echo ""
  echo "############################################################"
  echo "# RUN ${N}/${NRUNS}  ->  ${OUT}"
  echo "############################################################"

  stop_gait ; sleep 1.0                    # clear any lingering gait from before
  echo "[A] START (${START_X},${START_Y}) teleport + seed (unmeasured prior)"
  teleport "$START_X" "$START_Y" "$START_YAW"
  sleep "$SETTLE_AFTER_TELEPORT"
  python3 "$SCRIPTS/seed_side.py" --x "$START_X" --y "$START_Y" --yaw "$START_YAW"
  sleep "$SETTLE_AFTER_SEED"
  echo "    converge-hold ${CONVERGE_HOLD}s ..." ; sleep "$CONVERGE_HOLD"

  echo "[B] KIDNAP -> RE-ENTRY (${KID_X},${KID_Y}); WALK first (proper stance)"
  teleport "$KID_X" "$KID_Y" "$KID_YAW"
  python3 "$SCRIPTS/walk_op3.py" --x "$WALK_X" --angle "$WALK_ANGLE" \
      --duration "$WALK_DURATION" &
  WALK_PID=$!
  echo "    re-stand+walk ${RESTAND_WALK}s (clean fixes) before re-seed ..."
  sleep "$RESTAND_WALK"
  if [ "$RESEED" = "true" ]; then
    echo "    RESEED at re-entry (announced, robot walking = clean fixes) ..."
    python3 "$SCRIPTS/seed_side.py" --x "$KID_X" --y "$KID_Y" --yaw "$KID_YAW"
    sleep "$SETTLE_AFTER_SEED"
  else
    echo "    AUTONOMOUS (no re-seed): node must self-detect 'lost' -> reloc"
  fi

  echo "[C] eval -> ${OUT} (dur ${EVAL_DUR}s) while walking"
  python3 "$SCRIPTS/landmark_eval.py" --out "$OUT" --dur "$EVAL_DUR" &
  EVAL_PID=$!
  wait "$EVAL_PID" ; EVAL_PID=""
  echo "    eval done; letting walk finish cleanly (sends its own stop) ..."
  wait "$WALK_PID" ; WALK_PID=""          # NO kill — clean 'stop' from walk_op3

  if [ "$N" -lt "$NRUNS" ]; then
    echo "    inter-run gap ${INTER_RUN_GAP}s ..." ; sleep "$INTER_RUN_GAP"
  fi
done

echo ""
echo "############################################################"
echo "# AGGREGATE"
echo "############################################################"
python3 "$SCRIPTS/landmark_multirun.py" --label "reentry_8b" "${OUTS[@]}"
echo ""
echo "done. CSVs: ${OUTS[*]}"
