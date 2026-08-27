# Root Cause Analysis — Pengembangan Lokalisasi OP3 Soccer
**Tanggal**: 2026-06-21  
**Versi terakhir**: v10.20  
**Status**: Riset sedang berlangsung, hasil masih suboptimal

---

## 1. RINGKASAN EKSEKUTIF

Sistem lokalisasi gagal tracking posisi robot secara konsisten selama berjalan (x bergerak dari -0.363m ke +2.28m). **Root cause utama bukan satu bug tunggal**, melainkan tiga masalah struktural yang saling berinteraksi:

| # | Masalah | Dampak |
|---|---------|--------|
| **1** | Odometri humanoid sangat tidak akurat (~21% efisiensi displacement) | AMCL particles tidak bergerak mengikuti robot |
| **2** | `center_circle_filter_radius=1.6m` membuang semua observasi center line | Cox blind untuk x ∈ [-0.363, ~1.5m] = mayoritas trajektori |
| **3** | Tidak ada landmark horizontal (ny=0 di seluruh trajektori) | AMCL tidak bisa koreksi y via scan matching |

Sistem ini telah mengandalkan Cox WLS untuk mengkompensasi masalah #1, tetapi masalah #2 membuat Cox hampir selalu blind. Masalah #3 membuat y selalu melayang.

---

## 2. DATA EMPIRIS — SEMUA VERSI

### 2.1 RMSE Historis

| Versi | AMCL RMSE | EKF RMSE | AMCL x-capture | Durasi test | GT_x akhir | Catatan utama |
|-------|-----------|----------|----------------|-------------|------------|---------------|
| v10.10 | 0.947m | 0.736m | **3%** | 130s | 1.44m | Cox tidak aktif (AMCL deadzone salah) |
| v10.11 | 1.194m | 1.148m | **71%** | 195s | 2.48m | Sebelum filter; AMCL overshoot x |
| v10.12 | 0.748m | 0.654m | **51%** | 198s | 2.40m | Sebelum filter; Cox terlambat aktif |
| v10.15 | 1.444m | 1.448m | **99%** | 127s | 1.47m | y spread 0.5m → y false minimum |
| v10.16 | 0.729m | 0.696m | **26%** | 141s | 1.30m | Filter 1.6m aktif; test pendek |
| v10.17 | 0.466m | 0.336m | **64%** | 190s | 2.33m | Test panjang → sampai penalty box |
| v10.18 | 3.821m | 1.134m | **-206%** | 203s | 2.28m | AMCL diverge (GAGAL TOTAL) |
| v10.19 | 0.581m | 0.584m | **21%** | 151s | 1.60m | Filter 1.6m; test pendek; pose BEKU |

**"x-capture %"** = berapa persen perpindahan GT_x yang berhasil ditracking oleh AMCL.

### 2.2 Pola AMCL_y (seharusnya ≈ 0, karena GT_y ≈ 0)

| Versi | AMCL_y range | Penyebab drift y |
|-------|-------------|-----------------|
| v10.15 | [-2.32, +2.40] | cov_y=0.5 → y false minimum ±0.86m |
| v10.16 | [-0.03, +1.73] | Cox δy noise ×1000 (WLS ill-conditioned) |
| v10.17 | [-0.43, +1.08] | cov_y=0.04 → AMCL reconverge ke y false min |
| v10.19 | [-0.05, +0.22] | cov_y=0.001 → y BAIK, tapi x beku |

---

## 3. ROOT CAUSE #1 — ODOMETRI HUMANOID SANGAT TIDAK AKURAT

### 3.1 Bukti Empiris

Selama fase gerak (robot berjalan maju), EKF menggabungkan odom + AMCL + Cox. Jika AMCL dan Cox sama-sama salah, EKF ≈ odom murni. Dari data:

| Versi | Perpindahan odom-dominated (EKF) | Perpindahan GT sebenarnya | Efisiensi |
|-------|----------------------------------|---------------------------|-----------|
| v10.19 | +0.41m dalam 117s | +1.96m dalam 117s | **21%** |
| v10.16 | +0.28m dalam 107s | +1.64m dalam 107s | **17%** |

Robot berjalan ~16-17mm/s secara riil, tetapi EKF (yang mengandalkan odom) hanya mencapai ~3.5mm/s. **Odom mengestimasikan hanya ~21% dari displacement sesungguhnya.**

### 3.2 Kenapa Ini Kritis

AMCL particle filter bekerja dalam dua fase:
1. **Prediction**: particles digerakkan menggunakan odom motion model (robot bergerak → particles bergerak)
2. **Update**: particle weights diperbarui berdasarkan likelihood scan terhadap peta

Jika odom hanya 21% akurat:
- Particles bergerak 0.21× perpindahan sebenarnya
- Scan matching harus mengkompensasi sisanya (79% displacement)
- Scan matching bisa mengoreksi posisi jika:
  - Particles cukup dekat dengan posisi sebenarnya (dalam jangkauan kernel)
  - Ada landmark yang informatif di sekitar robot

Karena odom sangat underestimate, particles "tertinggal" di belakang posisi robot sebenarnya. Scan matching dengan landmark yang lemah (hanya garis vertikal) tidak cukup untuk menarik particles ke posisi benar sejauh 1.5m.

### 3.3 Mengapa Odom Buruk

OP3 menggunakan legged odometry berbasis deteksi stance (kapan kaki menyentuh tanah). Dalam simulasi Webots:
- Walking pattern adalah scripted trajectory (bukan adaptive)
- Slip tidak disimulasikan realistis
- IMU + stance detection → KF untuk estimate displacement
- Humanoid gait memiliki fase double-support dan single-support yang sulit dimodelkan

Untuk lokalisasi yang andal, sistem ini memerlukan odom yang jauh lebih akurat atau mekanisme koreksi yang jauh lebih agresif.

---

## 4. ROOT CAUSE #2 — CENTER CIRCLE FILTER MEMBUANG OBSERVASI PENTING

### 4.1 Sejarah dan Alasan Filter

**FIX J (v10.14)**: Ditambahkan `center_circle_filter_radius_m=1.6` untuk membuang observasi yang ter-project ke dalam r<1.6m dari world origin.

**Alasan aslinya (benar)**: Arc lingkaran tengah (r=0.75m) terdeteksi oleh HoughLinesP sebagai chord lurus. Chord ini jatuh di dalam lingkaran, dan Voronoi LUT di posisi itu memiliki normal yang salah → Cox memberikan δx palsu besar (+0.12-0.27m/step) → cascade failure.

**Yang tidak diperhitungkan**: Filter r<1.6m juga membuang observasi GARIS TENGAH (center line).

### 4.2 Geometri Masalah

Center line: garis lurus di world x=0, dari y=-3m ke y=+3m.

Ketika titik pada center line ter-project ke world coordinates: posisi = (0, y) → jarak dari world origin = |y|.

Dengan filter=1.6m: hanya titik dengan |y|>1.6m yang lolos.

Tapi kamera dari posisi robot di x=-0.363m, melihat ke depan (center line di x=0, jarak 0.363m):
- FOV horizontal: 2×arctan(640/793.3) ≈ 77.8°
- Pada jarak 0.363m, jangkauan y yang terlihat: ±0.363×tan(38.9°) ≈ ±0.29m
- Semua titik center line yang terlihat: |y| ≤ 0.29m → **SEMUA DIFILTER**

Bahkan ketika robot sudah jauh dari center line, kamera (forward-facing, pitch=-20°) melihat lantai dengan rentang y terbatas. Titik center line di |y|>1.6m hanya bisa terlihat dari sudut sangat ekstrim.

### 4.3 Konsekuensi

Untuk trajektori x ∈ [-0.363, ~1.5m] (mayoritas test):
- Center line (x=0) adalah satu-satunya landmark x yang visible
- **Semua observasi center line difilter** → Cox tidak punya data
- Cox tidak publish /initialpose → AMCL tidak mendapat koreksi x
- AMCL hanya bergerak via odom (21% efisiensi) → pose BEKU

Penalty box front (x≈2.48m) berada di r≥2.48m > 1.6m → LOLOS filter. Tapi robot baru mencapai area ini di x≈2.0m ke atas.

### 4.4 Ini Menjelaskan Pola Aneh v10.17

v10.17 mencapai x-capture 64% padahal masih pakai filter=1.6m. Alasannya:
- Test berlangsung 190s, robot mencapai GT_x=2.33m
- Di x≈1.5-2.3m, penalty box front mulai terlihat (r>1.6m)
- Cox mulai aktif di bagian AKHIR test → AMCL catch-up
- Versi lain (v10.16: 141s, v10.19: 151s) robot tidak sampai sejauh itu → Cox blind sepanjang test

**v10.17 bukan lebih baik karena algoritmanya lebih baik — tapi karena testnya berlangsung lebih lama hingga robot mencapai zona yang bisa diobservasi Cox.**

---

## 5. ROOT CAUSE #3 — TIDAK ADA LANDMARK Y DI SELURUH TRAJEKTORI

### 5.1 Hasil LUT Analysis

Voronoi LUT (`voronoi_lut.npz`), shape (400,550), res=0.02m:
- Diperiksa semua cell di x∈[0,3]m, y∈[-2,2]m (zona trajektori robot)
- **ny_nonzero = 0 dari 29,009 cell yang diperiksa**
- Semua garis yang visible hanya memiliki nx≠0, ny≈0 (garis vertikal: tegak lurus sumbu x)

### 5.2 Implikasi

Ini berarti:
1. WLS A[1,1] = ζ = 0.001 (minimal) → δy tidak bisa diestimasi dari observasi
2. AMCL scan matching tidak bisa update y berdasarkan fitur apapun di lapangan
3. AMCL_y hanya bergerak via:
   - Odom y-component (juga tidak akurat)
   - /initialpose dari Cox (tapi Cox tidak punya info y)

### 5.3 Garis Horizontal Terdekat

Garis horizontal (y-direction) yang bisa memberikan info y:
- Garis tepi lapangan: y=±3m (robot berjalan di y≈0, tidak terlihat)
- Garis goal box: x≈4.5m (di luar trajektori test)
- Penalty box samping: y≈±2.5m (juga jauh)

Tidak ada garis horizontal yang bisa dilihat robot selama berjalan dari center ke penalty box.

---

## 6. PERJALANAN FIX DAN TEMUAN DARI SETIAP VERSI

### Kronologi Fix

```
v10.10 → v10.11: Cox pertama kali aktif
v10.11 → v10.12: Tambah center deadzone (position-based, AMCL) → Cox stuck
v10.12 → v10.13: Deadzone pakai odom → threshold 2.0m tidak pernah tercapai → Cox ZERO
v10.13 → v10.14: Geometric filter 1.6m + Cox→EKF direct [FIX J] ← AWAL MASALAH baru
v10.14 → v10.15: Wide AMCL spread (0.5m) + startup delay + prior=/amcl_pose [FIX K]
v10.15 → v10.16: y covariance 0.05 + cooldown 5s + reinstate Cox→EKF [FIX L]
v10.16 → v10.17: Suppress δy when ny=0 [FIX O]
v10.17 → v10.18: Disabled /initialpose [FIX P] ← GAGAL TOTAL
v10.18 → v10.19: cov_y 0.04→0.001 [FIX Q]
v10.19 → v10.20: filter_radius 1.6→0.9 [FIX R]
```

### Temuan Kunci per Versi

**v10.12**: AMCL x-capture 51%. Test terpanjang (198s, GT_x=2.4m). Bukti bahwa kalau Cox bisa aktif di zona penalty box, localization bekerja lebih baik. Tidak ada center_circle_filter — center circle dijaga via position-based deadzone.

**v10.13**: ZERO Cox aktivasi dalam 169s. Pelajaran: **jangan pernah gate Cox berdasarkan estimasi posisi** — estimasi yang salah = Cox tidak pernah aktif.

**v10.14**: Center_circle_filter_radius=1.6m diperkenalkan. Filter ini menyebabkan Cox blind untuk center line, tapi ini belum teridentifikasi karena test yang panjang di v10.12 masih membantu Cox aktif di penalty area.

**v10.17 (terbaik)**: RMSE EKF=0.336m. Bukan karena algoritmanya sempurna, tapi karena test berlangsung lama (190s) dan AMCL y akhirnya lebih tenang (Fix O stop noisy δy). Cox masih blind untuk center line, tapi penalty box visible di akhir test.

**v10.18 (terburuk)**: AMCL_x = -5.4m akhirnya. Pelajaran: Cox /initialpose adalah **SATU-SATUNYA mekanisme koreksi x awal**. Tanpa itu, AMCL konverge ke false minimum x≈-0.04m dan diverge seiring robot bergerak maju.

---

## 7. MASALAH YANG BELUM TERPECAHKAN

### 7.1 AMCL False Initial Convergence

AMCL selalu start dari (0,0) sementara robot benar-benar ada di (-0.363, 0). AMCL secara konsisten konverge ke x ≈ -0.04 to -0.13m (bukan -0.363m). Ini karena:
- Center line (x=0) tampak hampir sama dari x=0 vs x=-0.363m (center line dekat)
- Sebelum Cox aktif (t=26s), AMCL "puas" dengan posisi yang sedikit salah

**Cox /initialpose diperlukan untuk mengoreksi kesalahan awal ini**, tapi Cox sendiri butuh prior yang agak benar untuk bekerja → chicken-and-egg problem.

### 7.2 Circular Dependency Fundamental

```
AMCL prior yang salah
    ↓
Cox prior yang salah (karena pakai /amcl_pose)
    ↓
WLS projection salah → δx salah
    ↓
/initialpose memindahkan AMCL ke posisi salah lainnya
    ↓
AMCL semakin jauh dari kebenaran
```

Cox hanya efektif jika AMCL sudah dalam jangkauan yang wajar. Tapi AMCL butuh Cox untuk sampai ke jangkauan wajar. Ini adalah bootstrap problem yang belum terpecahkan.

### 7.3 AMCL Parameter `num_particles` dan Motion Model

Tidak diketahui berapa jumlah particles yang digunakan. Untuk robot yang bergantung pada scan matching dengan odom yang sangat buruk (21%), diperlukan banyak particles untuk "mempertaruhkan" di berbagai posisi x.

### 7.4 Systematic x Lag

Bahkan di v10.17 (terbaik), AMCL seringkali LEBIH MAJU dari GT_x (overshoot):
- t=73s: AMCL_x=0.796m, GT_x=0.450m → AMCL 0.35m TERLALU MAJU
- t=77s: AMCL_x=0.933m, GT_x=0.516m → AMCL 0.42m TERLALU MAJU

Lalu kemudian tertarik kembali. Ini menunjukkan ada osilasi antara undershoot dan overshoot yang terkait dengan /initialpose yang menggerakkan particles terlalu jauh setiap 5s.

---

## 8. ANALISIS EFEK FIX R (v10.20: filter 1.6→0.9m)

### 8.1 Apa yang Diharapkan

Dengan filter=0.9m, center line di |y|>0.9m bisa diobservasi Cox.

Dari posisi robot x=-0.363m (melihat ke center line di x=0, jarak 0.363m):
- FOV horizontal ≈ ±0.29m di x=0 → titik center line visible: |y| ≤ 0.29m → MASIH DIFILTER

Namun: camera juga melihat pada jarak lebih jauh. Dengan pitch=-20°, camera melihat lantai jauh ke depan. Center line juga terlihat di sudut, dimana y lebih besar.

**Pertanyaan kritis yang belum terjawab**: Apakah kamera dapat melihat center line di |y|>0.9m dari posisi robot x≈-0.363m?

### 8.2 Skenario Optimistis

Jika kamera bisa melihat center line di y∈[0.9, 1.5m]:
- Cox mendapat 10-20 observasi valid
- WLS dapat menghitung δx dari center line
- /initialpose setiap 5s dengan x-correction
- AMCL mulai tracking x dari t≈30s (bukan t≈100s seperti sebelumnya)

### 8.3 Skenario Pesimistis

Jika FOV kamera tidak mencapai |y|>0.9m saat robot di x=-0.363m:
- Filter 0.9m masih membuang semua observasi center line
- Kondisi sama seperti v10.19
- Perbedaan: ketika robot mendekati center line (x≈-0.1m), titik di y>0.9m mulai terlihat

Dalam skenario ini, perbaikan terjadi di awal walk saat melewati center line, bukan dari posisi start.

---

## 9. ARAH RISET ALTERNATIF

### 9.1 Perbaiki Odometri (High Impact, High Effort)

**Problem**: 21% efisiensi odom adalah akar dari semua masalah.

**Opsi**:
- Visual Odometry: estimasi motion dari perubahan frame kamera
- Better KF tuning: naikkan process noise Q untuk membolehkan prediksi lebih agresif
- Wheel/foot contact force estimation yang lebih akurat
- Ganti walking pattern agar odom lebih deterministik

**Expected impact**: Jika odom 70%+, AMCL bisa tracking robot tanpa Cox, dan Cox hanya diperlukan untuk global correction.

### 9.2 Ganti Particle Filter dengan ICP/NDT Langsung (High Impact, Medium Effort)

**Problem**: AMCL particle filter adalah probabilistic approach yang membutuhkan odom yang baik untuk particle prediction.

**Opsi**: Gunakan line segment matching langsung:
- Deteksi line segments di citra
- Project ke world menggunakan estimated pose
- ICP (Iterative Closest Point) pada line segments vs peta
- Output: pose estimate langsung, tanpa particles

**Expected impact**: Tidak bergantung pada odom untuk particle propagation. Lebih robust terhadap odom yang buruk.

### 9.3 Gunakan Lebih Banyak Particles + Dynamic Injection

**Problem**: Dengan odom buruk, particles "left behind" dan scan matching tidak bisa pull mereka ke posisi benar (terlalu jauh).

**Opsi**: 
- Naikkan `num_particles` 10-100× (saat ini tidak diketahui berapa)
- Dynamic particle injection: setiap N detik, inject particles baru di sekitar seluruh lapangan
- Adaptive particle resample: ketika confidence rendah, spread particles lebih luas

**Expected impact**: Lebih banyak particles = lebih banyak peluang untuk scan matching menemukan posisi benar.

### 9.4 Rethink Sensor Selection

**Problem**: Sistem menggunakan kamera untuk scan matching via projected 2D "scan". Ini sangat bergantung pada:
- Kualitas deteksi garis putih
- Akurasi model kamera (proyeksi)
- Kualitas Voronoi LUT

**Pertanyaan**: Apakah ada sensor lain yang bisa digunakan?
- IMU untuk yaw (sudah ada, tapi yaw drift di Webots masih ada)
- Goal post detection untuk triangulasi
- Corner detection untuk absolute localization
- Depth sensor / stereo vision

### 9.5 Pisahkan Phase: Global Localization + Tracking

**Problem**: Sistem mencoba global localization (mencari posisi dari scratch) dan tracking (mengikuti gerakan) sekaligus dengan mekanisme yang sama.

**Opsi**:
- Phase 1 (0-30s): Global localization mode — AMCL dengan banyak particles, tanpa Cox interference
- Phase 2 (setelah konvergen): Tracking mode — AMCL + Cox untuk koreksi incremental

### 9.6 Ganti WLS Solver dengan EKF Direct Pose

**Problem**: Cox WLS menghasilkan δpose yang kemudian dikirim sebagai /initialpose (scatter particles). Ini adalah cara yang tidak efisien — informasi δpose yang tepat dibuang menjadi distribusi particles.

**Opsi**: Cox output δpose → langsung ke EKF sebagai pose measurement (bukan lewat AMCL reinit). Ini sudah ada (/cox_pose → EKF pose1), tapi AMCL masih mendominasi karena menerima /initialpose.

Jika Cox hanya feed EKF (tanpa /initialpose), EKF akan mengintegrasikan odom + Cox corrections langsung. AMCL menjadi secondary check saja.

---

## 10. CONSTRAINT YANG TIDAK BOLEH DIUBAH (PELAJARAN DARI FAILURE)

| Constraint | Alasan |
|-----------|--------|
| Cox prior HARUS /amcl_pose (bukan /odometry/filtered) | v10.14: prior=/odometry/filtered → Cox↔EKF loop → diverge |
| Jangan gate Cox berdasarkan estimated position | v10.13: deadzone berbasis posisi → Cox ZERO aktivasi 169s |
| Cox /initialpose HARUS tetap aktif | v10.18: tanpa /initialpose → AMCL x diverge, RMSE=3.83m |
| startup_delay_s ≥ 20s | AMCL butuh ~20s untuk initial convergence sebelum Cox mengganggu |
| correct_yaw=False | Yaw estimation dari Cox tidak reliable dengan garis vertikal saja |
| cov_y ≤ 0.001 | v10.16-17: cov_y=0.04 → y false minimum |
| min_y_constraint=5.0 | v10.16: tanpa ini, δy noise 1000× saat ny=0 |
| pose1=/cox_pose di EKF | v10.15: tanpa ini, EKF hanya follow AMCL errors |

---

## 11. HIPOTESIS ROOT CAUSE TUNGGAL

Setelah menganalisis semua versi, hipotesis root cause tunggal adalah:

> **Sistem ini menggunakan particle filter (AMCL) yang memerlukan odometri berkualitas baik untuk beroperasi. Odometri humanoid OP3 hanya 21% akurat, menyebabkan particle filter tidak bisa mengikuti gerakan robot. Semua upaya tuning parameter adalah workaround terhadap masalah ini, bukan solusi sebenarnya.**

Cox WLS didesain sebagai koreksi untuk AMCL yang sudah cukup dekat dengan posisi sebenarnya. Tetapi karena AMCL sendiri tidak bisa tracking (odom buruk), Cox tidak pernah bisa efektif karena priornya selalu salah.

---

## 12. REKOMENDASI UNTUK RISET BERIKUTNYA

### Prioritas Tinggi
1. **Ukur akurasi odom secara terisolasi**: Log /odom vs GT_x selama walking. Konfirmasi 21% figure dan karakteristik error (random drift vs systematic bias).
2. **Coba naikkan num_particles AMCL ke 5000-10000**: Lihat apakah tracking membaik dengan lebih banyak particles meskipun odom buruk.
3. **Verifikasi filter_radius=0.9m (v10.20)**: Periksa apakah Cox mendapat observasi valid di awal test.

### Prioritas Menengah
4. **Implementasi direct ICP localization**: Bypass particle filter, gunakan line-to-line ICP langsung.
5. **Explore visual odometry**: OpenCV optical flow atau ORB-SLAM2 untuk odom yang lebih akurat.

### Prioritas Rendah (Architectural Change)
6. **Rethink seluruh pipeline**: Evaluasi apakah AMCL adalah pilihan yang tepat untuk robot humanoid dengan odom buruk dan landmark terbatas.

---

## 13. FILE PENTING

| File | Fungsi |
|------|--------|
| `src/motion_webots/src/localization_ws/soccer_object_localization/launch/localization_v14.launch.py` | Launch file utama, semua parameter |
| `src/motion_webots/src/localization_ws/soccer_object_localization/soccer_object_localization/cox_registration.py` | Cox WLS solver |
| `src/motion_webots/src/localization_ws/soccer_object_localization/config/ekf_soccer.yaml` | EKF fusion parameters |
| `voronoi_lut.npz` | LUT jarak/normal untuk WLS, ny=0 di seluruh trajektori |
| `localization_evaluator.py` | Script evaluasi RMSE |
| `pose_eval_1777514360.csv` | v10.17 (terbaik: EKF=0.336m) |
| `pose_eval_1777542643.csv` | v10.19 (terakhir: EKF=0.584m, pose beku) |
