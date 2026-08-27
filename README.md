# PANDUAN — Lokalisasi v15 + Visualisasi Live + Demo Pertandingan

Semua perintah dijalankan dari root workspace `~/basbot`. Tools lokalisasi ada di dalam
workspace (BUKAN `~/basbot/scripts/`), jadi pakai variabel `SCRIPTS` di bawah agar ringkas.

## 0. Prasyarat (sekali per boot)
- Webots **berjalan (play)** dengan world OP3 termuat, robot ada di lapangan.
- Di TIAP terminal baru, source dulu:
```bash
cd ~/basbot
source /opt/ros/humble/setup.bash
source install/setup.bash
export SCRIPTS=~/basbot/src/motion_webots/src/localization_ws/landmark_localization/scripts
```

## 1. Terminal 1 — jalankan stack lokalisasi v15 (T1)
```bash
ros2 launch soccer_object_localization localization_v15_landmark.launch.py \
    detector:=yolo use_gaze:=true
```
Ini profil pertandingan: line-heading ON (default), gaze ON, `/fall` aktif-tapi-inert, tanpa
degradasi. Biarkan terminal ini hidup (memegang YOLO + EKF + line-heading + gaze).
Arg opsional: `use_line_heading:=false`, `use_degrade:=true`, `chi2_gate:=16.27`, `imgsz:=`, `conf:=`.

**Dirikan robot** lewat GUI seperti biasa (Mode `walking_module` → start → stop); pastikan robot
berdiri (GT z ≈ 0.247). Cek pipeline hidup:
```bash
ros2 topic hz /landmark_array        # harus ~6 Hz
ros2 topic hz /odometry/filtered     # harus ~10 Hz  (Ctrl-C untuk berhenti)
```

## 2. Terminal 2 — visualisasi live
```bash
python3 "$SCRIPTS/live_viz.py"
```
Jendela peta lapangan (skala Webots) muncul:
- **panah cyan (isi)** = pose EKF (keyakinan lokalisasi) + jejak
- **panah putih (kosong)** = ground truth (pembanding, sim)
- **titik oranye** = fix geometris mentah (`/landmark_pose`)
- kotak teks: `err pos=… yaw=…` real-time

Opsi: `--no-gt` (hardware, tak ada GT) · `--no-fix` · `--trail 300`
Jika jendela tak muncul (backend headless): `MPLBACKEND=TkAgg python3 "$SCRIPTS/live_viz.py"`
(atau `sudo apt install python3-tk`).

## 3. Terminal 3 — robot "bertanding"
```bash
python3 "$SCRIPTS/match_demo.py"
```
Robot melewati 5 adegan lintas lapangan (KICKOFF → SERANG → LEMPAR-KE-DALAM → BERTAHAN →
SUDUT lawan). Tiap adegan = penempatan (teleport) + re-seed di pose aktual robot + jalan.
Tonton panah cyan mengikuti di Terminal 2.
Opsi: `--loop` (ulang terus) · `--ball-head` (kepala menunduk mengejar bola = kondisi lebih berat)

### Versi 2 — JALAN KONTINU, minim teleport (rekomendasi untuk melihat tracking)
```bash
python3 "$SCRIPTS/match_demo2.py"
```
Sekali penempatan+seed di KICKOFF, lalu robot **berjalan terus-menerus** menyusuri waypoint
acak di seluruh lapangan (belok halus + maju, gait diperbarui live tanpa stop-start). Menguji
tracking BERKELANJUTAN (seperti 8c) lintas lapangan. Arah belok dikalibrasi otomatis di awal.
Opsi: `--duration 240` (lama jalan, detik) · `--no-seed` (lewati kickoff, jika sudah terlokalisasi)
· `--ball-head` (kepala mengejar bola sepanjang jalan)

### Versi 3 — rute AGRESIF (sprint pojok-ke-pojok, belok tajam)
```bash
python3 "$SCRIPTS/match_demo3.py"
```
Seperti v2 tapi menyasar ekstrem lapangan (pojok/ujung) → sprint diagonal panjang + belok tajam +
gait tercepat-yang-stabil. Bagus untuk menegangkan tracking di gerak dinamis.
Opsi: `--speed 0.020` · `--turn 18` · `--period 0.58` (lebih kecil=lebih cepat) · `--duration` · `--ball-head`
**CATATAN:** gait op3 di Webots ini terbatas ~0.02 m/s dan nyaris tak naik dgn amplitudo (diuji
x=0.024/period=0.56 → tetap ~0.02 m/s, stabil tapi tak lebih cepat). "Agresif" v3 dari RUTE (sprint+
belok tajam), bukan kecepatan lokomosi mentah. `--speed` lebih tinggi hanya menambah risiko jatuh.

## Uji manual lain (opsional, dari Terminal terpisah, T1 tetap hidup)
```bash
# teleport robot ke suatu pose (x, y, theta[rad])
ros2 topic pub --once /robotis_op3/set_pose geometry_msgs/msg/Pose2D "{x: -2.5, y: 0.0, theta: 0.0}"
# re-seed pose yang diketahui (SOP penempatan)
python3 "$SCRIPTS/seed_side.py" --x -2.5 --y 0 --yaw 0
# jalan manual
python3 "$SCRIPTS/walk_op3.py" --x 0.012 --angle 0 --duration 30
```

## Catatan hardware
Di robot nyata tak ada `/ground_truth/odom` → jalankan viz dengan `--no-gt`. `/fall` (produser
IMU behavior) belum divalidasi di hardware — lihat `S2_PRAREGISTRASI_GETUP_SUBSTITUSI.md`.
