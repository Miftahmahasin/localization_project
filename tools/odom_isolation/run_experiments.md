# Fase 0 — Panduan Eksperimen Isolasi Odometri

**Tujuan**: Karakterisasi `/odom` mentah vs ground truth tanpa interferensi EKF/AMCL.

---

## Persiapan

```bash
# 1. Source workspace
source /opt/ros/humble/setup.bash
source /home/miftah/basbot/install/setup.bash

# 2. Pastikan kamu ada di root repo
cd /home/miftah/basbot
```

**Stack yang perlu jalan:**
- Webots + op3_extern_controller (robot berjalan)
- `legged_odometry_kf_node` (publish `/odom`)
- `webots_odom_publisher.py` atau `op3_extern_controller` (publish `/ground_truth/odom`)

**Stack yang TIDAK perlu / bisa dimatikan:**
- AMCL, EKF, Cox, scan_gate, dll. — nonaktifkan untuk logging yang lebih bersih
- Alternatif: jalankan stack penuh, tapi logger kita hanya baca `/odom` dan `/ground_truth/odom`

Cek topik tersedia:
```bash
ros2 topic echo /odom --once
ros2 topic echo /ground_truth/odom --once
```

---

## Exp-1: Jalan Lurus ~2m (kecepatan normal)

```bash
python3 tools/odom_isolation/odom_logger.py \
  --output exp1_straight.csv \
  --label  exp1_straight \
  --speed  1.0
```

1. Tunggu pesan "Data diterima — mulai logging."
2. Suruh robot jalan lurus ke depan ±2m
3. Biarkan robot berhenti beberapa detik, lalu tekan **Ctrl+C**
4. Verifikasi: `wc -l exp1_straight.csv` harus > 200 baris

---

## Exp-2a: Kecepatan Rendah

```bash
python3 tools/odom_isolation/odom_logger.py \
  --output exp2_slow.csv \
  --label  exp2_slow \
  --speed  0.5
```

- Jalankan robot pada kecepatan ~setengah normal, jalan lurus ±2m
- **Isi `--speed` sesuai perintah kecepatan yang kamu pakai** (angka ini hanya dicatat sebagai metadata analisis)

---

## Exp-2b: Kecepatan Tinggi

```bash
python3 tools/odom_isolation/odom_logger.py \
  --output exp2_fast.csv \
  --label  exp2_fast \
  --speed  1.5
```

- Jalankan robot pada kecepatan maksimum aman, jalan lurus ±2m

---

## Exp-3: Rotasi Yaw

```bash
python3 tools/odom_isolation/odom_logger.py \
  --output exp3_yaw.csv \
  --label  exp3_yaw \
  --speed  0.0
```

- Suruh robot putar di tempat: 90° kiri, 90° kanan, atau 360° penuh
- Log minimal 30 detik

---

## Analisis (setelah semua eksperimen selesai)

```bash
python3 tools/odom_isolation/odom_analyze.py \
  exp1_straight.csv \
  exp2_slow.csv \
  exp2_fast.csv \
  exp3_yaw.csv
```

Output:
- `fase0_plots/plot1_timeseries.png` — time series odom vs GT
- `fase0_plots/plot2_scatter.png` — scatter cumulative displacement
- `fase0_plots/plot3_k_vs_speed.png` — k vs kecepatan
- `fase0_plots/plot4_yaw.png` — yaw tracking
- `FASE0_HASIL.md` — laporan lengkap + **VERDICT**

---

## Troubleshooting

| Masalah | Solusi |
|---------|--------|
| "Menunggu data..." tidak berubah | Cek `ros2 topic list` apakah `/odom` dan `/ground_truth/odom` ada |
| CSV hanya beberapa baris | Pastikan robot benar-benar bergerak |
| `k_disp = NaN` di analisis | CSV terlalu pendek atau robot tidak bergerak cukup |
| Plot tidak muncul | Normal — disimpan ke file PNG, tidak ditampilkan interaktif |
| Error `matplotlib` tidak ada | `pip3 install matplotlib numpy` |

---

## Catatan Penting

- **JANGAN** analisis dari `/odometry/filtered` — itu output EKF yang sudah terkontaminasi AMCL beku
- **JANGAN** ubah parameter lokalisasi selama Fase 0 — hanya mengukur
- Setelah `FASE0_HASIL.md` selesai, **BERHENTI** dan laporkan verdict ke user untuk keputusan G0
