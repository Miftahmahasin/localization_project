#!/usr/bin/env bash
# ============================================================================
# TAHAP H — run ONE degradation point: set the degrade_relay knob(s) live, then
# run a (short) seeded-8c walk eval. Line-heading ON/OFF is a LAUNCH condition, so
# run this under a T1 started with use_degrade:=true and the desired
# use_line_heading, once per condition; then compare with harness_analyze.py.
#
#   T1 (condition A): ...launch... use_gaze:=true use_line_heading:=true use_degrade:=true
#   T1 (condition B): ...same but use_line_heading:=false
#
# Usage:
#   run_degrade_point.sh <prefix> <nruns> '<param> <value>' ['<param> <value>' ...]
# Example (total junction filter, 2 runs, 60 s walk each):
#   WALK_DURATION=60 run_degrade_point.sh filt_on 2 'filter_classes [0,1,2]'
#   WALK_DURATION=60 run_degrade_point.sh cut2_on 2 'cutoff_range_m 2.0'
# The knob is set via `ros2 param set /degrade_relay ...` (live; no relaunch).
# Pass 'reset' as the only knob arg to clear all degradation back to pass-through.
# ============================================================================
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="/home/miftah/basbot"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null || true
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash" 2>/dev/null || true

PREFIX="${1:-deg}"; NRUNS="${2:-2}"; shift 2 || true

if ! ros2 node list 2>/dev/null | grep -q "/degrade_relay"; then
  echo "!! /degrade_relay not running — launch T1 with use_degrade:=true. Aborting."
  exit 1
fi

if [ "$1" = "reset" ]; then
  echo "[H] reset degrade_relay to pass-through"
  ros2 param set /degrade_relay filter_classes "[-1]"    >/dev/null 2>&1 || true
  ros2 param set /degrade_relay cutoff_range_m 0.0        >/dev/null 2>&1 || true
  ros2 param set /degrade_relay recall 1.0               >/dev/null 2>&1 || true
  ros2 param set /degrade_relay recall_dist_slope 0.0    >/dev/null 2>&1 || true
  ros2 param set /degrade_relay fp_per_frame 0           >/dev/null 2>&1 || true
else
  for kv in "$@"; do
    name="${kv%% *}"; val="${kv#* }"
    echo "[H] set /degrade_relay $name = $val"
    ros2 param set /degrade_relay "$name" "$val" || { echo "  set failed"; exit 1; }
  done
fi
sleep 1.0

echo "[H] eval point '$PREFIX' (${NRUNS} run, walk=${WALK_DURATION:-180}s) ..."
"$SELF/run_seeded_8c.sh" "$NRUNS" "$PREFIX"
