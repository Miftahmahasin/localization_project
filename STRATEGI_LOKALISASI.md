# STRATEGI PENGEMBANGAN LOKALISASI OP3 — Rencana Eksekusi Bertahap

**Pendamping untuk**: `LOKALISASI_ROOT_CAUSE_ANALYSIS.md` (v10.20)
**Tanggal**: 2026-06-21
**Tujuan dokumen**: Mengubah temuan RCA menjadi rencana eksekusi bertahap dengan *decision gate* yang jelas, agar pengerjaan (oleh Claude Code) terstruktur dan berhenti menambal gejala. Setiap fase punya kriteria selesai dan gerbang keputusan eksplisit.

---

## 0. PRINSIP UTAMA

> **Perbaiki kualitas input (odometri + fitur) sebelum mengganti arsitektur.**

RCA cenderung mengarah ke penggantian arsitektur (ICP/NDT langsung, rethink AMCL). Itu jebakan klasik: berisiko membakar berminggu-minggu, punya masalah local-minima & chicken-and-egg yang sama, dan mengulang semua bug dari nol — untuk lari dari masalah yang sebenarnya ada di **kualitas odom** dan **kemiskinan fitur**. Kerangka probabilistik (MCL + EKF) yang dipakai sekarang adalah norma RoboCup dan terbukti; yang lemah adalah inputnya. Jadi kita perbaiki input dulu, baru pertimbangkan arsitektur kalau memang masih buruk.

---

## 1. REVIEW KRITIS ATAS RCA (wajib dipahami sebelum eksekusi)

### 1.1 Angka "21% odom" adalah HIPOTESIS, bukan fakta terukur
Seluruh RCA bertumpu pada satu angka (odom 21% efisien) dan menariknya jadi *root cause tunggal* (RCA §11). Tapi angka itu **hasil inferensi dari output EKF, bukan pengukuran /odom mentah**. Logika di RCA §3.1 ("kalau AMCL & Cox sama-sama salah, EKF ≈ odom murni") tidak valid sepenuhnya: kalau AMCL **beku**, EKF justru sedang **diseret mundur** oleh pose AMCL yang diam. Jadi displacement EKF +0.41m kemungkinan = odom (lebih besar) yang ditarik turun oleh AMCL beku → angka 21% bisa jadi **lebih buruk** dari odom mentah sebenarnya. RCA §12 sendiri menempatkan "ukur odom terisolasi" sebagai prioritas #1, yang berarti **ini belum dilakukan**. Ini kontradiksi yang harus diselesaikan lebih dulu.

### 1.2 Konsistensi ~20% justru petunjuk penting → kemungkinan SYSTEMATIC
v10.19 = 21%, v10.16 = 17%. Dua-duanya ~20%. Kalau penyebabnya **slip acak**, varians akan tinggi dan arah error berubah-ubah. Efisiensi yang konsisten ~20% lebih berbau **systematic scale error** (kalibrasi step length salah, stance detection melewatkan langkah, atau KF over-damped dengan process noise Q terlalu kecil). Implikasinya besar:

| Jenis error | Sifat | Solusi |
|-------------|-------|--------|
| **Systematic (scale)** | rasio konstan | nyaris trivial — cari konstantanya / terapkan correction factor; AMCL kemungkinan langsung jalan |
| **Random (slip)** | varians tinggi, drift | odom tak bisa dipercaya untuk prediction → butuh VO atau pendekatan feature-driven |

Dua titik data yang terkontaminasi **belum cukup** menyimpulkan yang mana — itulah persis kenapa Fase 0 (isolasi) jadi gerbang utama.

### 1.3 Saga filter center circle = luka yang dibuat sendiri
Seluruh drama `center_circle_filter` (FIX J → blindness center line → FIX R) berakar dari **mendeteksi lingkaran dengan detektor garis** (HoughLinesP membaca arc sebagai chord → normal salah → δx palsu). Solusinya **bukan** memfilter `r<1.6m` (yang lalu ikut membunuh center line → pose beku) — solusinya **mendeteksi center circle sebagai lingkaran** (Hough circle / ellipse fit). Keuntungan ganda: (1) filter tak perlu lagi → masalah #2 hilang; (2) pusat lingkaran = titik berkoordinat diketahui → constraint kuat; (3) lingkaran membatasi **x DAN y** → langsung menyerang masalah #3.

### 1.4 Masalah y sebagian artefak tes jalan lurus
Tes adalah jalan lurus sepanjang x di y≈0 — kasus **degenerate** untuk observabilitas y. Di pertandingan asli robot berbelok, melihat gawang & sudut, sehingga ny≠0 sering muncul. Jadi "y selalu melayang" bukan kondisi permanen sistem; tetap perlu diperbaiki demi robustness, tapi jangan over-index ke skenario tes ini.

### 1.5 Kesalahan kecil perhitungan FOV
RCA §8.1/§4.2 memakai `2×arctan(640/793.3) ≈ 77.8°` (lebar penuh), padahal seharusnya `2×arctan(320/793.3) ≈ 43.9°` (half-width). FOV asli lebih sempit → rentang y terlihat lebih kecil → analisis FIX R condong **terlalu optimis**. Harus diverifikasi ulang dengan intrinsik kamera asli, karena seluruh rasional filter=0.9m bergantung pada ini.

### 1.6 Daftar "constraint terlarang" (RCA §10) = gejala over-patching
Daftar panjang "yang tidak boleh diubah" adalah tanda sistem terjebak di *local optimum yang rapuh* (tiap fix melahirkan masalah baru). Sumber utama ketidakstabilan: loop `/initialpose` (Cox → scatter particles AMCL tiap 5s) → osilasi overshoot/undershoot (RCA §7.4) & divergensi v10.18. Constraint-constraint itu tetap **dihormati** selama input belum beres, tapi targetnya adalah menyederhanakan struktur sehingga daftar ini mengecil.

---

## 2. PETA KEPUTUSAN (DECISION GATES)

| Gate | Setelah | Pertanyaan | Cabang |
|------|---------|-----------|--------|
| **G0** | Fase 0 | Error odom systematic atau random? | systematic → Branch A; random → Branch B |
| **G1** | Fase 1 (odom + fitur beres) | Apakah AMCL sudah tracking (x-capture >70%, RMSE turun)? | ya → Fase 2; tidak → evaluasi ulang arsitektur (baru di sini ICP/factor-graph dipertimbangkan) |
| **G2** | Fase 2 | Apakah EKF stabil sebagai pemilik pose tanpa osilasi `/initialpose`? | ya → selesai/lanjut fitur match; tidak → rollback bertahap |

---

## 3. FASE 0 — ISOLASI ODOMETRI (WAJIB PERTAMA, MURAH, MENENTUKAN)

**Tujuan**: Karakterisasi `/odom` **mentah** (sebelum EKF) vs ground truth saat jalan lurus, untuk menentukan apakah error systematic (scale) atau random (slip). Ini gerbang yang menentukan seluruh strategi.

### 3.1 Persiapan
1. Identifikasi topik **odom mentah** dari node walking (BUKAN `/odometry/filtered` yang merupakan output EKF). Konfirmasi via launch file + `ros2 topic list` / node graph.
2. Identifikasi topik **ground truth** (pose dari Webots supervisor).
3. Pastikan gait scripted/open-loop (per RCA, lokalisasi tidak memengaruhi gait) → logging saat run normal aman, tapi run standalone tanpa lokalisasi lebih bersih.

### 3.2 Eksperimen
- **Exp-1 (lurus)**: robot jalan lurus ~2m di lantai datar. Log `raw_odom_pose`, `gt_pose`, timestamp pada rate tinggi (≥20 Hz).
- **Exp-2 (kecepatan)**: ulangi Exp-1 pada 2–3 kecepatan perintah berbeda.
- **Exp-3 (yaw)**: perintah putar di tempat (atau arc). Log `yaw_odom` vs `yaw_gt`.

### 3.3 Analisis
- Plot `odom_x(t)` vs `gt_x(t)`. Hitung rasio instan & kumulatif.
- Fit linear: apakah `gt_disp ≈ k · odom_disp` dengan k konstan? Laporkan **k dan R²**.
  - R² tinggi + k konstan → **systematic scale** (correctable).
  - R² rendah / k bervariasi → **random slip**.
- Laporkan **per-sumbu**: forward (x body), lateral (y), yaw.
- Dari Exp-2: plot k vs kecepatan. Konstan → scale. Memburuk saat cepat → slip.
- Dari Exp-3: laporkan yaw drift rate (relevan untuk `correct_yaw=False`).

### 3.4 Deliverable & Acceptance Criteria
- [ ] Script logging + analisis (reusable).
- [ ] Plot tersimpan (odom vs GT per sumbu; k vs kecepatan).
- [ ] Laporan markdown singkat berisi: nilai **k per sumbu**, **R²**, kurva k-vs-kecepatan, yaw drift.
- [ ] **Verdict eksplisit**: `SYSTEMATIC (scale, k=…)` / `RANDOM (slip)` / `MIXED`.
- [ ] **BERHENTI di sini** — jangan langsung memperbaiki. Laporkan hasil untuk keputusan G0.

---

## 4. FASE 1 — BERCABANG BERDASARKAN G0

### 4.1 Branch A — Error Systematic (scale/kalibrasi)
1. Temukan sumber kalibrasi (step length / stance detection / Q di KF odom) atau, sebagai stopgap, terapkan correction factor `1/k`.
2. Retest AMCL **apa adanya** dengan odom terkoreksi. Hipotesis: ini saja menyelesaikan 70–80% masalah tracking.
3. Ukur ulang metrik (lihat §8). Ke gerbang **G1**.

### 4.2 Branch B — Error Random (slip)
1. Odom tak bisa dipercaya untuk prediction.
2. Eksplor **visual odometry** — **tapi uji dulu** apakah lantai punya cukup tekstur (hijau polos + garis putih jarang sering gagal untuk optical flow). Bandingkan OpenCV optical flow ground-plane vs alternatif.
3. Sementara VO belum andal, naikkan ketergantungan pada koreksi absolut dari fitur (§4.3) dan kurangi peran prediksi odom.

### 4.3 Paralel (KEDUA branch) — Upgrade Feature Layer
Ini unlock untuk **observabilitas y** dan **disambiguasi**, tanpa sensor baru. Pipeline sekarang memperlakukan semua piksel putih sebagai "scan 2D" tak berstruktur (Voronoi LUT hanya merepresentasikan garis & normalnya → mewarisi `ny=0`). Tambahkan fitur kaya constraint:

1. **Center circle sebagai lingkaran**: ganti/augmentasi HoughLinesP dengan `cv2.HoughCircles` atau contour + `fitEllipse` pada mask putih. Saat terdeteksi → proyeksikan pusat lingkaran ke world → pakai sebagai **point landmark**. **Hapus** `center_circle_filter_radius` setelah ini benar.
2. **Junction garis (T & L)**: deteksi perpotongan segmen garis, klasifikasikan via geometri lapangan. Tiap junction = point landmark (constraint x & y penuh).
3. **Tiang gawang**: cek apakah perception bisa deteksi tiang gawang (vertikal putih). Jika ya → pakai *base of post* sebagai point landmark (sangat kuat untuk x & y).
4. **Penalty mark**: deteksi titik penalti bila terlihat.
5. Integrasikan sebagai observasi ke lokalisasi (mekanisme integrasi tergantung kerangka yang dipilih pasca-G0).

> Verifikasi ulang geometri visibilitas (FOV asli, pitch=-20°, tinggi kamera) sebelum mengandalkan asumsi RCA tentang apa yang "terlihat".

---

## 5. FASE 2 — BERSIHKAN STRUKTUR FUSION

Sumber ketidakstabilan utama adalah loop `/initialpose`. Target: EKF jadi **pemilik pose**, AMCL didemosi.

1. Pastikan `odom` + `/cox_pose` masuk EKF sebagai measurement (sudah ada via `pose1=/cox_pose`).
2. Alihkan koreksi Cox **terutama** ke EKF; jadikan AMCL untuk *global recovery / divergence detection* saja.
3. **CAVEAT v10.18 (KRITIS)**: v10.18 gagal karena `/initialpose` dimatikan **sementara AMCL masih pemilik x dan tak ada koreksi x lain**. Jadi: **restrukturisasi kepemilikan estimasi DULU** (EKF jadi pemilik), **baru** kurangi/cabut `/initialpose`. Validasi tiap langkah. Jangan ulangi urutan v10.18.
4. Atasi osilasi `/initialpose` (RCA §7.4): bila masih dipakai, turunkan frekuensi/magnitudo atau gate pada confidence.
5. Ke gerbang **G2**.

---

## 6. YANG TIDAK BOLEH DILAKUKAN (DULU)

- ❌ Rewrite ke ICP/NDT/factor-graph sebagai langkah pertama. (Hanya dipertimbangkan setelah G1 gagal.)
- ❌ Menambah band-aid tuning parameter di stack lama **sebelum** Fase 0.
- ❌ Mengubah constraint di §7 tanpa memahami alasannya (lihat tabel).
- ❌ Mencabut `/initialpose` sebelum EKF jadi pemilik pose (pelajaran v10.18).
- ❌ Menyimpulkan kualitas odom dari output EKF (harus dari `/odom` mentah).

---

## 7. CONSTRAINT YANG DIPERTAHANKAN (dari RCA §10)

Hormati ini selama input belum beres; tujuannya menyederhanakan sehingga daftar ini mengecil seiring waktu.

| Constraint | Alasan |
|-----------|--------|
| Cox prior HARUS `/amcl_pose` (bukan `/odometry/filtered`) | v10.14: prior=odom → Cox↔EKF loop → diverge |
| Jangan gate Cox berdasarkan estimasi posisi | v10.13: deadzone posisi → Cox ZERO aktivasi 169s |
| Cox `/initialpose` aktif (sampai EKF jadi pemilik pose) | v10.18: tanpa itu → AMCL x diverge, RMSE 3.83m |
| `startup_delay_s ≥ 20s` | AMCL butuh ~20s konvergensi awal |
| `correct_yaw=False` | Yaw Cox tak reliable dengan garis vertikal saja |
| `cov_y ≤ 0.001` | v10.16-17: cov_y=0.04 → y false minimum |
| `min_y_constraint=5.0` | tanpa ini δy noise 1000× saat ny=0 |
| `pose1=/cox_pose` di EKF | tanpa ini EKF hanya ikut error AMCL |

---

## 8. METRIK EVALUASI KONSISTEN

Gunakan `localization_evaluator.py` + format `pose_eval_*.csv` untuk tiap iterasi. Lacak:

- AMCL RMSE, EKF RMSE
- **x-capture %** (target >70%, idealnya >90%)
- AMCL_y range (target mendekati [−0.1, +0.1] saat GT_y≈0)
- Cox activation rate & **time-to-first-Cox-activation** (target turun drastis dari ~100s)
- Durasi test distandarkan (mis. selalu ≥180s) agar antar-versi sebanding — ingat RCA §4.4: v10.17 "terbaik" sebagian karena testnya lebih lama.

---

## 9. FILE REFERENSI (dari RCA §13)

| File | Fungsi |
|------|--------|
| `…/launch/localization_v14.launch.py` | Launch utama, semua parameter |
| `…/soccer_object_localization/cox_registration.py` | Cox WLS solver |
| `…/config/ekf_soccer.yaml` | Parameter fusi EKF |
| `voronoi_lut.npz` | LUT jarak/normal (ny=0 di trajektori) |
| `localization_evaluator.py` | Evaluasi RMSE |
| `pose_eval_1777514360.csv` | v10.17 (EKF=0.336m) |
| `pose_eval_1777542643.csv` | v10.19 (EKF=0.584m, pose beku) |

---

## RINGKASAN SATU PARAGRAF

Angka 21% odom adalah hipotesis yang belum diuji dan kemungkinan **systematic** (artinya mudah diperbaiki). **Fase 0**: isolasi odom mentah sebagai gerbang keputusan. **Fase 1**: perbaiki odom sesuai cabang (kalibrasi vs VO) **dan**, paralel, upgrade feature layer (center circle sebagai lingkaran + junction + tiang gawang) untuk membunuh masalah y. **Fase 2**: rapikan struktur fusion (EKF jadi pemilik pose, hati-hati caveat v10.18). **Jangan** rewrite arsitektur sebelum input beres.
