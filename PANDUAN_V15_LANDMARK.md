# Panduan Perintah — Stack Lokalisasi Geometris v15 (TAHAP 4, tanpa odom)

Menjalankan & mengevaluasi `localization_v15_landmark.launch.py`: fix pose **geometris
per-frame** (CLAP/ILM → EKF `pose2`), **tanpa odometri badan**. Model YOLO:
`/media/miftah/Project/landmark_deploy/` (`.pt`, `.onnx`, OpenVINO int8 — semua imgsz 640,
kelas `L T X goalpost center_circle`).

Ganti path bila berbeda. **Workspace root = `/home/miftah/basbot`** (di sinilah `install/`
yang Anda `source`). Skrip evaluasi ada di pohon sumber.

```bash
WS=/home/miftah/basbot
SCRIPTS=$WS/src/motion_webots/src/localization_ws/landmark_localization/scripts
MODEL=/media/miftah/Project/landmark_deploy
```

---

## 0. Build & source (sekali)

```bash
source /opt/ros/humble/setup.bash
cd $WS
# paket ini di-install sebagai SALINAN nyata (bukan symlink) — jangan pakai --symlink-install
colcon build --packages-select landmark_localization soccer_object_localization
source $WS/install/setup.bash
# untuk Webots + robot: source juga workspace OP3 sim Anda (op3_manager) di tiap terminal
```

> `soccer_msgs` / `landmark_geometry` / `landmark_detector` sudah ter-*install* & tak berubah —
> tak perlu di-rebuild. Bila `soccer_msgs` perlu rebuild tapi gagal karena symlink lama:
> `rm -rf build/soccer_msgs install/soccer_msgs` lalu
> `colcon build --packages-select soccer_msgs --allow-overriding soccer_msgs`.

---

## 1. Uji plumbing OFFLINE (tanpa Webots) — sanity 30 detik

Membuktikan jalur `fake_detector → landmark_projector → geometric_pose_node → /landmark_pose`
tersambung. Ini **bukan** uji tracking (data sidecar = teleport).

```bash
ros2 launch soccer_object_localization localization_v15_landmark.launch.py \
    detector:=fake sidecar_dir:=/media/miftah/backup/landmark_dataset/val

# terminal lain — harus keluar pose:
ros2 topic hz /landmark_pose
ros2 topic echo /landmark_array --once
```

Cek log `geometric_pose_node`: `fixes: full=.. single=.. none=..`.

---

## 2. Uji LIVE di Webots (jalur nyata, model YOLO)

Butuh 3–4 terminal. Semua di-`source` seperti Bagian 0.

**T1 — Webots + robot OP3 (sim-time, GT):**
```bash
ros2 launch op3_manager op3_simulation.launch.py
# pastikan terbit: /robotis_op3/camera/image_raw, /robotis_op3/camera/camera_info,
#                  /robotis_op3/joint_states, /ground_truth/odom
```

**T2 — stack lokalisasi geometris v15 (YOLO):**
```bash
ros2 launch soccer_object_localization localization_v15_landmark.launch.py \
    detector:=yolo single_corner_mode:=partial use_line_scan:=false \
    model_path:=$MODEL/best_landmark_v8n_int8_openvino_model imgsz:=640 conf:=0.25
```
Launch memuat `landmark_detector` (sub `/robotis_op3/camera/image_rect` → pub
`/robot1/object_bounding_boxes`), `landmark_projector`, dan `geometric_pose_node`. Default
`model_path` sudah menunjuk ke OpenVINO int8 di atas — argumen bisa dihilangkan bila cocok.

> **Model** (`model_path`): OpenVINO int8 = paling ringan, imgsz **wajib 640**. Alternatif
> ringan untuk tes cepat: `model_path:=$MODEL/best_landmark_v8n.pt imgsz:=640` (atau `imgsz:=320`).

**T3 — jalankan robot berjalan** (trajektori nyata — WAJIB untuk uji konvergensi, bukan diam):
gunakan walking/gait Anda yang biasa (mis. demo soccer / perintah `/robotis/...`) agar robot
menempuh lintasan ≥180 s.

**T4 — evaluator konvergensi:**
```bash
python3 $SCRIPTS/landmark_eval.py --out runA_partial.csv --dur 180
```
Saat selesai ia mencetak: EKF pos RMSE, time-to-converge, laju fix (full/single per detik),
dan **gap fix terpanjang** (jendela coasting saat kepala menunduk — yang harus dilewati desain no-odom).

---

## 3. Bandingkan single-corner (a) partial vs (b) coast

Pada **trajektori yang sama**, jalankan dua kali dengan mode berbeda:

```bash
# run A — partial update:
#   T2: ... single_corner_mode:=partial   ; T4: --out runA_partial.csv
# run B — coast:
#   T2: ... single_corner_mode:=coast     ; T4: --out runB_coast.csv

python3 $SCRIPTS/landmark_eval.py \
    --compare runA_partial.csv runB_coast.csv \
    --compare_png ab_compare.png
```
Menghasilkan tabel RMSE/time-to-converge + plot error(t) A vs B → pilih pemenang dari angka.

---

## 4. Checklist verifikasi (GATE 0 sisa + TF)

**Pohon TF (harus `map→odom→base_link`, EKF pemilik `map→odom`):**
```bash
ros2 run tf2_tools view_frames && evince frames.pdf   # atau:
ros2 run tf2_ros tf2_echo map base_link
```
Pastikan **hanya satu** publisher `odom→base_link` (identitas statis dari launch); AMCL harus
`tf_broadcast:=False`.

**GATE 0 — distribusi lag head-sync** (dibaca dari log `landmark_projector`, terbit tiap 5 s):
```
head-sync lag: n=..  median=.. ms  p95=.. ms  max=.. ms
```
Catat median/p95 → itu sisa GATE 0.

**Laju & jenis fix** (log `geometric_pose_node`): `fixes: full=.. single=.. none=..`.
Bandingkan dengan peta gaze 4.4 (`fase_gy1_plots/gaze_fix_map.png`): saat kepala menunduk
melacak bola (`tilt < −35°`) `none` harus melonjak — di situ EKF coasting + (kelak) line-scan.

---

## 5. Varian model (untuk TAHAP 7 nanti)

| Model | Path | Catatan |
|---|---|---|
| OpenVINO int8 | `$MODEL/best_landmark_v8n_int8_openvino_model` | ringan CPU/NUC, imgsz **640** |
| PyTorch | `$MODEL/best_landmark_v8n.pt` | fleksibel, bisa imgsz 320/640 |
| ONNX | `$MODEL/best_landmark_v8n.onnx` | portabel |

Uji degradasi TAHAP 7 (junction hilang di imgsz 320) dilakukan nanti; peta gaze & fix-rate
sudah memprediksi: bila L/T/X hilang, fix penuh runtuh ke ~16% → wajib bersandar line-scan.
