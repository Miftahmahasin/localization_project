#!/usr/bin/env bash
# ============================================================================
# Pengumpulan dataset landmark — split TRAIN + VAL (fixed-head, 1080p, DR halus)
# ============================================================================
# PRASYARAT (jalankan & pastikan SEBELUM skrip ini):
#   1. Webots sudah di-RESTART memuat world baru (resolusi 1920x1080 + anti-
#      aliasing + fisika kaku). Reload saja tidak cukup untuk ganti resolusi.
#   2. op3_extern_controller (extern) tersambung, robot berdiri via op3_manager,
#      perilaku head-scan DIMATIKAN (skrip men-set head sendiri).
#   3. Workspace sudah di-build/disinkron (file sudah tersalin ke install).
#
# PEMAKAIAN:
#   ./collect_dataset.sh smoke     # uji 200 frame ke .../_smoke (cek report dulu)
#   ./collect_dataset.sh train     # 8000 frame, seed 1  -> .../train
#   ./collect_dataset.sh val        # 1200 frame, seed 777 -> .../val
#   ./collect_dataset.sh all        # train lalu val berurutan
#
# TOGGLE (env di depan perintah, mis. `SIM_DR=0 ./collect_dataset.sh all`):
#   OUT_ROOT   root output           (default /media/miftah/backup/landmark_dataset)
#   HEAD_MODE  aim|fixed             (default fixed)
#   DR         1|0 DR fotometrik      (default 1)
#   SIM_DR     1|0 DR render lighting (default 1)
# ============================================================================
set -euo pipefail

OUT_ROOT="${OUT_ROOT:-/media/miftah/backup/landmark_dataset}"
HEAD_MODE="${HEAD_MODE:-fixed}"
DR="${DR:-1}"
SIM_DR="${SIM_DR:-1}"

# --- source ROS + workspace ------------------------------------------------
# ROS setup scripts reference unset vars; disable nounset while sourcing.
set +u
source /opt/ros/humble/setup.bash
if [ -f "$HOME/basbot/install/setup.bash" ]; then
  source "$HOME/basbot/install/setup.bash"
fi
set -u

dr_bool() { [ "$1" = "1" ] && echo true || echo false; }

run_split() {   # $1=name  $2=num_samples  $3=seed  $4=resume(true|false, default false)
  local name="$1" n="$2" seed="$3" resume="${4:-false}" out="$OUT_ROOT/$1"
  echo "=============================================================="
  echo " SPLIT: $name  | samples=$n  seed=$seed  head=$HEAD_MODE  resume=$resume"
  echo " output: $out"
  echo " DR(foto)=$(dr_bool "$DR")  SIM_DR(render)=$(dr_bool "$SIM_DR")"
  echo "=============================================================="
  ros2 launch landmark_dataset_gen landmark_sampler.launch.py \
    num_samples:="$n" \
    seed:="$seed" \
    output_dir:="$out" \
    head_mode:="$HEAD_MODE" \
    resume:="$resume" \
    domain_randomize:="$(dr_bool "$DR")" \
    sim_dr:="$(dr_bool "$SIM_DR")"
  echo ">>> selesai split '$name'. Periksa $out/report.txt & report_montage.png"
}

case "${1:-}" in
  smoke) run_split "_smoke" "${2:-200}" 0 ;;   # optional 2nd arg = jumlah sample
  train) run_split "train" 8000 1 ;;
  val)   run_split "val"   1200 777 ;;
  all)
    run_split "train" 8000 1
    echo; echo "### jeda 5 dtk sebelum val ###"; sleep 5
    run_split "val"   1200 777
    ;;
  # resume-train: continue an interrupted TRAIN (same seed 1, 8000 target) from
  # the next contiguous index — renders only the missing tail, no re-render.
  resume-train) run_split "train" 8000 1 true ;;
  # continue: resume the train tail, then collect val fresh (recover from a
  # disk-full that killed the run partway). This is the one to use now.
  continue)
    run_split "train" 8000 1 true
    echo; echo "### jeda 5 dtk sebelum val ###"; sleep 5
    run_split "val"   1200 777
    ;;
  *)
    echo "pakai: $0 {smoke|train|val|all|resume-train|continue}"; exit 1 ;;
esac

echo
echo "Sisa disk:"; df -h "$OUT_ROOT" | grep -vE '^Filesystem' || true
