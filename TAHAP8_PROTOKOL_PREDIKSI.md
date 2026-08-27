# TAHAP 8 — Protokol Evaluasi Formal + PREDIKSI (dikomit sebelum data)

> Aturan main #1 & DoD TAHAP 8: multi-run ≥5, ≥ (durasi skenario), lapor **distribusi
> (mean ± std, min, max)**, dan **komit prediksi SEBELUM melihat data**. Prediksi di
> bawah ditulis 2026-08-22 sebelum run apa pun dijalankan. JANGAN diedit setelah data
> masuk — bandingkan saja hasil vs prediksi.

Baseline pembanding: **v10.17 line-AMCL EKF = 0.336 m** (walking); line-AMCL global-
relocalize = **BEKU (tak konvergen dalam 180 s)**. Stack diuji = no-odom geometris
(TAHAP 4) + mirror-hold (TAHAP 5) + gaze policy (TAHAP 6A). TAHAP 6B (line-scan CV) =
tidak layak di sim (published=0/8500 frame), `use_line_heading:=false`.

Alat: `landmark_eval.py` (per-run CSV) → `landmark_multirun.py` (agregasi distribusi).
`export SCRIPTS=/home/miftah/basbot/src/motion_webots/src/localization_ws/landmark_localization/scripts`

---

## 8a — Global relocalization dari seragam (time-to-converge)

**Tujuan:** dari prior seragam (tanpa seed manual), berapa lama EKF mengunci <0.30 m
(ditahan 3 s). start_x_sign=-1 auto-commit sisi → tak perlu seed.

**Per run (ulang 5×, relaunch fresh tiap run — stokastik: variasi deteksi YOLO + timing):**
```bash
ros2 topic pub --once /robotis_op3/set_pose geometry_msgs/msg/Pose2D "{x: -2.5, y: 0.0, theta: 0.0}"
# relaunch v15 (use_gaze:=true, use_line_heading:=false), TANPA /initialpose seed:
ros2 launch soccer_object_localization localization_v15_landmark.launch.py \
    detector:=yolo single_corner_mode:=partial use_gaze:=true imgsz:=640 conf:=0.25
# begitu stack hidup (nodes t=2 muncul), SEGERA:
python3 $SCRIPTS/landmark_eval.py --out a_run1.csv --dur 60
```
Metrik: time-to-converge (distribusi), pos RMSE pasca-konvergen, sisi TRUE%/flips.

**PREDIKSI 8a (dikomit):**
- time-to-converge: **mean ≈ 18 s, rentang 12–25 s**, **5/5 konvergen**.
- pos RMSE (termasuk fase pull-in): mean ≈ 0.5–0.9 m (didominasi 0–18 s awal); median ≲ 0.15 m.
- sisi: **TRUE 100%, flips 0** (auto-commit sisi seketika).
- vs baseline: line-AMCL **tak pernah konvergen** dari seragam → kita menang telak.

---

## 8b — Kidnap recovery se-sisi (recovery time)

**Tujuan:** setelah terkunci, di-teleport jauh SE-SISI, ukur waktu pulih (self-detect
lost → MHL reloc → EKF reset → re-converge). Kidnap LINTAS-mirror TIDAK diuji (ambigu
fundamental — dilaporkan, bukan dipaksa).

**Per run (ulang 5×):** stack hidup, teleport own-half, seed, TUNGGU `full≈100% committed=True`.
Lalu **kidnap DULU, baru start eval** (supaya t0 eval melihat error besar → converge =
waktu pulih):
```bash
# (sudah terkunci di ~(-2.5,0))
ros2 topic pub --once /robotis_op3/set_pose geometry_msgs/msg/Pose2D "{x: -1.0, y: 2.5, theta: 0.0}"
python3 $SCRIPTS/landmark_eval.py --out b_run1.csv --dur 40
```
Metrik: time-to-converge dari CSV = **waktu pulih**; sisi TRUE% pasca-pulih; flips.

**PREDIKSI 8b (dikomit):**
- recovery time: **mean ≈ 9 s, rentang 5–15 s**, **5/5 pulih** (jarak kidnap ~2.9 m se-sisi).
- sisi pasca-pulih: **TRUE 100%**; flips ≤ 1 (mungkin 1 saat transien reloc).
- pos RMSE jendela ini besar (langkah kidnap ~2.9 m di t0) — bukan ukuran tracking.

---

## 8c — Tracking approach-to-kick DENGAN gaze aktif (error tracking)

**Tujuan:** error tracking saat berjalan, dan worst-case gap tak menyebabkan divergensi.

**Per run (ulang 5×):** stack hidup (use_gaze:=true), teleport own-half hadap +x, seed,
tunggu terkunci. Lalu rekam + jalan:
```bash
python3 $SCRIPTS/landmark_eval.py --out c_run1.csv --dur 150 &
python3 $SCRIPTS/walk_op3.py --x 0.012 --angle 0 --duration 140
```
Metrik: pos RMSE, median, p95, yaw RMSE, longest fix-less gap, sisi TRUE%/flips.

**PREDIKSI 8c (dikomit):**
- pos RMSE: **mean ≈ 0.22 m, rentang 0.15–0.30 m** → **mengalahkan baseline 0.336 m**.
- median ≲ 0.12 m; p95 ≲ 0.55 m; yaw RMSE ≈ 3–6°.
- longest fix-less gap: **< 3 s** (gaze mengangkat kepala sebelum blackout panjang) —
  tak ada divergensi (tak ada flip permanen).
- sisi: TRUE ≥ 98%, flips ≤ 2 (transien band tengah bila lintasan lewat sana).

---

## Agregasi (setelah semua run)
```bash
python3 $SCRIPTS/landmark_multirun.py \
    --label "8a global-reloc" a_run*.csv \
    --label "8b kidnap"       b_run*.csv \
    --label "8c tracking"     c_run*.csv
```
Melaporkan mean±std, min, max, fraksi-konvergen, mirror%, flips per skenario. Bandingkan
langsung dengan blok PREDIKSI di atas (jujur: sebut mana yang meleset).
