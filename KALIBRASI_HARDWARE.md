# Prosedur Kalibrasi Kamera — Robot Nyata (draft, TAHAP A1.3)

> Draft prosedur. Wajib dijalankan **per-robot** sebelum lokalisasi landmark dipakai
> di hardware (Orin Nano / NUC). Alasan: di sim, `pitch_bias=−5°` dibekukan karena
> **label & runtime berbagi bias yang sama** (sistem konsisten-diri). Di hardware
> **tidak ada Supervisor**; pose kamera nyata = apa pun yang diberikan mekanik →
> **−5° TIDAK akan transfer.** Harus diukur.

Kalibrasi terbagi dua bagian **independen**: **intrinsik** (K) dan **ekstrinsik**
(tinggi/pitch/pan kamera relatif `base_link`).

---

## Bagian 1 — Intrinsik (K) → `camera.yaml`

- Kalibrasi checkerboard standar (mis. `ros2 run camera_calibration cameracalibrator`
  atau OpenCV offline) pada **resolusi tangkap yang akan dipakai deploy**.
- Simpan hasil sebagai `camera.yaml` (format `camera_matrix`/`distortion_coefficients`).
- Suapkan ke `camera_info_publisher` via param `camera_yaml_path`. **Jangan** andalkan
  fallback FOV — fallback hanya untuk sim/pengujian awal. (Fallback kini menurunkan K
  dari FOV+resolusi, jadi tetap menskala, tapi tak menangkap distorsi lensa nyata.)
- Verifikasi: `ros2 topic echo .../camera_info --once` → K sesuai resolusi tangkap;
  `rectify_node` menghasilkan `image_rect` yang lurus (garis lurus tetap lurus).

---

## Bagian 2 — Ekstrinsik (tinggi, pitch, pan) → file kalibrasi per-robot

Runtime `landmark_projector` SUDAH mengekspos param yang perlu diisi (jangan
hardcode di kode):

| Param projector | Arti | Nilai sim (beku) | Hardware |
|---|---|---|---|
| `base_height_m` | tinggi `base_link` di atas tanah | 0.30 | ukur |
| `base_z_offset_m` | koreksi tinggi kamera | 0.0 | fit |
| `pitch_bias_deg` | koreksi pitch rantai-kepala vs kamera | −5.0 | **fit (kritis)** |
| `pan_bias_deg` | koreksi pan | 0.0 | fit |

### Prosedur fit (overlay reprojeksi — reuse tools yang ada)

1. **Tempatkan robot pada pose yang DIKETAHUI** di lapangan: ukur `(x, y, yaw)`
   terhadap garis lapangan (mis. berdiri tepat di titik penalti menghadap gawang).
   Catat juga `head_pan`, `head_tilt` dari `/robotis_op3/joint_states`.
2. **Tangkap satu frame** `image_rect` pada pose itu (robot diam, kepala diam).
3. **Overlay geometri peta** ke frame dengan kandidat `(base_z_offset, pitch_bias,
   pan_bias, base_height)`, pakai skrip yang sudah ada:
   - `landmark_dataset_gen/scripts/overlay_lines.py` — proyeksikan garis/junction peta
     ke citra; garis terproyeksi harus **menempel** garis nyata **di seluruh frame**
     (bukan hanya di tengah — dekat-horizon paling sensitif ke pitch).
   - `landmark_dataset_gen/scripts/measure_pitch_residual.py` — ukur residu pitch
     terhadap garis ter-deteksi (min-RGB ridge), untuk fit kuantitatif pitch_bias.
4. **Cari nilai** yang meminimalkan residu reprojeksi (grid-search kecil pada
   pitch_bias ±3° langkah 0.5°, lalu base_z_offset ±2 cm, lalu pan_bias ±2°).
   Urutan penting: pitch paling dominan di dekat horizon.
5. **Validasi silang**: ulangi overlay di **pose kedua yang berbeda** (jarak/bearing
   lain). Nilai yang benar harus pas di kedua pose — kalau hanya pas di satu, itu
   kompensasi palsu.

### Simpan & pakai
- Tulis hasil ke **file kalibrasi per-robot** (mis. `config/extrinsics_<robot>.yaml`):
  `base_height_m, base_z_offset_m, pitch_bias_deg, pan_bias_deg`.
- Teruskan ke `landmark_projector` via param launch (`localization_v15_landmark.launch.py`
  sudah meneruskan `pitch_bias_deg`/`base_z_offset_m`/`pan_bias_deg`). **INVARIANT tetap:
  runtime & (jika regenerasi dataset di hardware) sampler harus berbagi nilai yang sama.**

### Kriteria lulus
- Overlay: garis terproyeksi menempel garis nyata **di seluruh frame** di ≥2 pose uji.
- `/landmark_array`: `valid_range=true` untuk junction dekat (≲7 m), `p_base` masuk akal.
- Standing test: seed `/initialpose` di pose diketahui → EKF RMSE ≲ sim (~0.2 m).

> **Kapan diulang:** setiap ganti robot, remount kamera, atau ganti lensa/resolusi.
> Ekstrinsik salah = seluruh fix absolut bias sistematis (persis bug pitch −5° yang
> hilang di sim awal, §5.3 laporan).
