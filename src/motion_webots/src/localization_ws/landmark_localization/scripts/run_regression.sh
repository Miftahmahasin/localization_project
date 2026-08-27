#!/usr/bin/env bash
# ============================================================================
# TAHAP C1 — one-command regression suite for the mature localization results.
#
# Runs the 8c tracking batch + the 8b re-entry batch (the existing orchestrators)
# and gates each against the frozen baseline, so any BAGIAN B change that quietly
# regresses 8c 0.208 m / 8b 0.230 m / mirror 0% / flips 0 is caught immediately.
# Run this after EVERY change in BAGIAN B; if it FAILs outside noise, roll back.
#
# The LAUNCH (T1) must be up first (same as the individual scripts):
#   ros2 launch soccer_object_localization localization_v15_landmark.launch.py \
#       detector:=yolo use_gaze:=true use_line_heading:=false
#
# Usage:
#   ./run_regression.sh                # 5+5 runs, both scenarios
#   ./run_regression.sh 5 8c           # only the 8c batch
#   ./run_regression.sh 5 8b           # only the 8b batch
# Exit code 0 = PASS (all run scenarios pass), 1 = FAIL.
# ============================================================================
# NOTE: no `set -u` — sourcing ROS setup.bash references unbound vars.

NRUNS="${1:-5}"
WHICH="${2:-both}"          # both | 8c | 8b

SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="/home/miftah/basbot"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null || true
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash" 2>/dev/null || true

FAIL=0

# Gate semantics (see regression_gate.py): position = median across runs of each run's
# OWN median error (steady-state, robust to the re-entry transient); mirror/flips =
# COUNT of outlier runs, tolerating --outlier-max=1 (the live baseline itself yields
# ~1/5 vision-death runaway at the kidnap window — a real regression hits 2+ runs).
run_8c(){
  echo ""; echo "################  8c TRACKING  ################"
  "$SELF/run_seeded_8c.sh" "$NRUNS" c_run || true
  python3 "$SELF/regression_gate.py" --scenario "8c tracking" \
      --rmse-th 0.25 --mirror-tol 1.0 --flip-th 1 --outlier-max 1 --conv-min 4 \
      "$WS_ROOT"/c_run*.csv || FAIL=1
}

run_8b(){
  echo ""; echo "################  8b RE-ENTRY  ################"
  "$SELF/run_kidnap_8b.sh" "$NRUNS" kr_run || true
  python3 "$SELF/regression_gate.py" --scenario "8b re-entry" \
      --rmse-th 0.30 --mirror-tol 2.0 --flip-th 1 --outlier-max 1 --conv-min 4 \
      "$WS_ROOT"/kr_run*.csv || FAIL=1
}

case "$WHICH" in
  8c) run_8c ;;
  8b) run_8b ;;
  *)  run_8c ; run_8b ;;
esac

echo ""
echo "############################################################"
if [ "$FAIL" -eq 0 ]; then
  echo "# REGRESSION: PASS — baseline held."
else
  echo "# REGRESSION: FAIL — a scenario regressed. Roll back the last change."
fi
echo "############################################################"
exit "$FAIL"
