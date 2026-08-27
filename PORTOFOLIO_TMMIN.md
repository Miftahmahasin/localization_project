# Portofolio Engineering — Lokalisasi Robot Sepak Bola Humanoid (OP3)
### Bahan lamaran internship TMMIN · disusun dari kondisi implementasi TERBARU (per 2026-08-25)

> Catatan sumber: dokumen ini diambil dari kode + hasil eksperimen terbaru di workspace,
> bukan dari deskripsi rencana lama. Bila rencana awal berbeda dengan implementasi sekarang,
> yang dipakai adalah **yang benar-benar terimplementasi** (perubahan konteks dijelaskan di §9).
> Label keyakinan: **[V]** terverifikasi dari kode/hasil · **[K]** kemungkinan besar benar ·
> **[B]** belum dapat diverifikasi.

---

## 1. Definisi Project Terbaru

Sistem **lokalisasi visual murni (pure-vision)** untuk robot humanoid OP3 di kompetisi
sepak bola robot (KRSBI-Humanoid / RoboCup-like). Robot menaksir posisinya sendiri di
lapangan **hanya dari satu kamera** — mendeteksi *landmark* garis lapangan (persimpangan
L/T/X, tiang gawang, lingkaran tengah) dengan YOLO, mengubahnya menjadi taksiran pose
geometris tertutup, lalu memfusikannya dengan EKF **tanpa odometri tubuh**. Keluarannya
(pose robot di peta) dipakai lapisan *behavior/navigation* robot untuk mengambil keputusan
bermain. **Perubahan definisi:** project awalnya lokalisasi berbasis *particle filter AMCL*
memindai garis putih; kini di-*pivot* total menjadi **lokalisasi geometris berbasis landmark
bertipe** karena pendekatan lama gagal di level lapangan penuh (lihat §9).

## 2. Masalah Utama (maks 3)

1. **Ambiguitas scan garis putih.** AMCL lama (v10–v14) tidak bisa membedakan posisi di
   level lapangan penuh — global relocalize *frozen* (cov konstan 8.13 m² selama >150 s),
   RMSE gagal 2–3.4 m, sering *collapse* dekat gawang. **[V]**
2. **Tidak ada odometri yang andal.** Odom tubuh OP3 = *command-echo* gait yang buta
   terhadap slip → tak bisa jadi sumber pose absolut. Sistem harus akurat **tanpa** odom. **[V]**
3. **Simetri cermin 180° lapangan.** Lapangan sepak bola simetris sempurna → satu tampilan
   landmark punya dua pose yang identik secara geometris; harus dipatahkan tanpa "menebak". **[V]**

## 3. Alur Kerja Sistem Terbaru

```
Kamera (1920×1080)
  → Deteksi landmark YOLOv8n  (kelas: L, T, X, goalpost, center_circle)
  → Proyeksi ground (pinhole + rantai kinematik kepala OP3 → titik lapangan)
  → Solusi pose geometris tertutup (WLS + kovarians + data-association Mahalanobis + RANSAC)
  → Mirror-mode side-lock (pilih dari 2 kembar cermin via keyakinan sisi 'ref')
  → EKF no-odom (pose2=/landmark_pose PRIMER, + ZUPT anti-runaway, + line-heading yaw)
  → Pose robot di peta (/odometry/filtered)
  → dikonsumsi lapisan behavior/navigation
```

**Alur lama → baru & alasan:**
- **Lama:** `Kamera → scan garis putih → AMCL particle filter → pose`. Gagal (ambiguitas
  global, near-goal collapse).
- **Baru:** landmark **bertipe** memberi kendala geometris yang jauh lebih kuat daripada
  garis anonim; solusi **closed-form + EKF** menggantikan particle filter; **odom dibuang
  total** dan diganti *fix* geometris per-frame yang sering (~13 Hz). **[V]**
- Landmark **tidak pernah** masuk particle filter (aturan keras) — memisahkan jalur baru
  dari baseline lama agar tak saling merusak.

## 4. Kontribusi Saya (maks 5 bullet terkuat)

- **Dibuat sendiri — pipeline lokalisasi geometris no-odom** (4 paket ROS2 baru:
  `landmark_geometry`, `landmark_localization`, `landmark_detector`, `landmark_dataset_gen`):
  model kamera bersama, solver pose WLS, data-association, EKF no-odom. **[V]**
- **Dibuat sendiri — pemecah simetri cermin 180° (`MirrorModeTracker`)**: *side-lock*
  deterministik berbasis keyakinan sisi, membawa hasil dari *mirror-lock* acak menjadi
  **0% mirror, 0 flip** pada uji berjalan. **[V]**
- **Dibuat sendiri — active-vision gaze recovery + line-heading insurance + ZUPT**: tiga
  lapisan robustness yang menutup *blackout* saat kepala menunduk / menghadap jaring gawang
  dan mencegah EKF *runaway*. **[V]**
- **Riset/eksperimen — generator dataset landmark auto-label + detektor YOLOv8n**: 8000+1200
  frame Webots berlabel otomatis dari ground-truth, dilatih & di-deploy sebagai OpenVINO INT8. **[V]**
- **Riset/eksperimen berdisiplin (confirm-first, pra-registrasi)**: setiap keputusan diuji
  ≥5 run, kriteria ditulis **sebelum** melihat data; beberapa fitur saya bangun lalu
  **matikan** karena terbukti tak bermanfaat (chi²-gate, C4, EKF-trap) — menunjukkan
  *engineering judgment* berbasis bukti. **[V]**

**Dikembangkan dari sistem sebelumnya:** paket `soccer_*` (soccerbot-derived) dan stack
Webots OP3 tim dipakai sebagai fondasi; jalur AMCL/Cox lama tetap utuh sebagai baseline. **[K]**
**Pekerjaan tim / bukan saya:** *gait*/kestabilan berjalan OP3 (robot kadang jatuh di physics
baseline = ranah pemilik gait), integrasi GameController, dan detektor fieldline OpenCV legacy. **[K]**

## 5. Data Kuantitatif  *(Metric — Value — Sumber/Bukti)*

| Metric | Value | Sumber/Bukti |
|---|---|---|
| Akurasi tracking berjalan (median-of-median) | **0.054 m (~5 cm)** | 8c final 5/5, `LAPORAN_SIM_SOLVABLE.md` **[V]** |
| Baseline lama (AMCL) sebagai pembanding | 0.336 m | memori proyek → **~6× lebih baik [V]** |
| Akurasi re-entry (kidnap/penempatan ulang) | 0.097–0.230 m median | 8b regresi **[V]** |
| Mirror-lock / flips | **0% / 0** di semua run bersih | 8b & 8c regresi **[V]** |
| Yaw error (tracking sehat) | < 1.1° | 8c final **[V]** |
| Line-heading saat blackout + rotasi (yaw p95) | **176° → 11° (−93.5%)** | S1 spin-in-place **[V]** |
| Kerusakan FP vs oklusi (bobot presisi) | FP merusak yaw ~4×, pos ~1.7× > oklusi setara | S3 **[V]** |
| Gate anti-racun C2 (ref-blend) | 788/788 fix resid-tinggi ditolak; 188/188 bersih di-blend | S4.1 **[V]** |
| Laju fix geometris | ~13 Hz | landmark_eval **[V]** |
| Waktu konvergensi (dengan seed) | 0–2.9 s | 8b/8c **[V]** |
| Reprojection error model kamera | **0.0000 m median** (0–7 m) | GATE 1 sensitivity **[V]** |
| Dataset | **8000 train + 1200 val** frame, auto-label | `/media/miftah/backup/landmark_dataset` **[V]** |
| Jumlah kelas landmark | **5** (L, T, X, goalpost, center_circle) | data.yaml **[V]** |
| Label landmark | ~59.036 train / ~8.700 val | relabel sidecar **[V]** |
| Detektor | YOLOv8n imgsz 640, OpenVINO INT8, .pt 6.2 MB | `/media/miftah/Project/landmark_deploy` **[V]** |
| Unit test lokalisasi inti | **28** (backend 14 + mirror 9 + assoc 5) | `test/*.py` **[V]** |
| Unit test total workspace lokalisasi | 62 fungsi | grep `def test_` **[V]** |
| mAP / precision / recall detektor terlatih | **belum terverifikasi** — model ada, angka benchmark belum saya konfirmasi | **[B]** |
| FPS/latency inferensi di target (NUC/Orin) | **belum terverifikasi** (butuh hardware) | **[B]** |

## 6. Teknologi Utama (maks 8–10)

- **Language:** Python (ROS2 nodes), sedikit C++ (supervisor teleport Webots)
- **Framework:** ROS2 Humble, `robot_localization` (EKF)
- **Model/Algorithm:** YOLOv8n (Ultralytics), pose geometris closed-form (CLAP/ILM + WLS),
  data-association Mahalanobis + RANSAC, EKF, mirror-mode side-lock, ZUPT
- **Simulation:** Webots R2023b (world OP3)
- **Deployment:** OpenVINO INT8 (NUC) / TensorRT INT8 (Orin, disiapkan)
- **Communication:** ROS2 topics/TF (`/landmark_array`, `/landmark_pose`, `/odometry/filtered`)
- **Vision/Math:** OpenCV, NumPy (ekstraktor garis min-RGB achromatic)

## 7. Keputusan Engineering Penting (maks 3)  *(Masalah → Keputusan → Alasan → Dampak)*

1. **Ganti AMCL garis → lokalisasi landmark geometris no-odom.**
   Masalah: AMCL garis putih ambigu & collapse. Keputusan: pivot ke landmark bertipe +
   solusi closed-form → EKF, buang odom. Alasan: landmark bertipe memberi kendala jauh lebih
   kuat; fix per-frame sering menggantikan odom. Dampak: **0.336 m → 0.054 m (~6×), mirror 0%**. **[V]**

2. **Pecahkan simetri cermin dengan side-lock, BUKAN dengan gate probabilistik.**
   Masalah: fix mirror-drag membuat robot yakin di sisi salah. Keputusan: `MirrorModeTracker`
   memilih kembar terdekat ke keyakinan sisi + seed andal (depth-20, walk-first-reseed);
   chi²-gate fix-vs-prior saya **matikan** karena A/B menunjukkan ia *menstarve* tracking
   (flip 5.8 vs 0). Alasan: side-lock deterministik + seed otoritatif lebih stabil daripada
   menolak fix. Dampak: **mirror 0% / flips 0** menetap. **[V]**

3. **Ekstraktor garis achromatic min-RGB menggantikan threshold global.**
   Masalah: detektor garis klasik (gray≥200 + Canny + Hough) memberi **0 heading** di rumput
   Webots yang bertekstur → sempat disimpulkan "line-scan tak viable". Keputusan: ganti ke
   ridge min(B,G,R)≥90 & low-saturation. Alasan: garis cat bersifat achromatic-terang, bukan
   sekadar terang. Dampak: `/line_heading` terbit ~9 Hz, strength 0.70+; **membalik** kesimpulan
   "tak viable" → line-heading jadi asuransi yaw (blackout+rotasi: p95 176°→11°). **[V]**

## 8. Hasil Project (maks 4)

- **Berhasil:** tracking berjalan **~5 cm** (5/5 run bersih), mirror 0%, yaw <1.1°, no-runaway
  — tesis inti (fix geometris menggantikan odom) terbukti berangka. **[V]**
- **Berhasil:** robustness lengkap tervalidasi di sim — re-seed re-entry mirror-free 5/5;
  gaze recovery ~2 s; ZUPT hentikan runaway; integritas *bulletproof* di FP 0–3/frame &
  recall 0.5–1.0. **[V]**
- **Sebagian:** detektor YOLOv8n **terlatih & ter-deploy** (OpenVINO INT8), pipeline live
  jalan end-to-end di Webots — tetapi **angka mAP/precision & sim-to-real hardware belum
  divalidasi**. **[V hasil / B angka]**
- **Dalam pengembangan / batas diterima:** global-init tanpa seed = **tidak mungkin
  sensor-only** (fisika cermin) — bukan bug, melainkan batas; solusi = seed eksternal
  (sah di RoboCup: robot masuk dari sisi diketahui). Recovery *get-up* jatuh = blocker
  gait/physics, disubstitusi re-seed. **[V]**

## 9. Konteks Terbaru / Perubahan Project

- **Algoritma berganti:** AMCL particle-filter garis putih → lokalisasi geometris landmark
  closed-form + EKF. **[V]**
- **Architecture berubah:** **odom tubuh dibuang total** (no-odom); ditopang fix per-frame
  ~13 Hz + ZUPT + gaze + line-heading. **[V]**
- **Fitur dibuang / dimatikan (dengan alasan berbukti):** body odometry; detektor garis
  OpenCV klasik (diganti min-RGB); chi²-gate fix-vs-prior (menstarve tracking); EKF-trap
  watchdog (lapisan salah); C4 cond-inflation (tak ada manfaat terukur); auto global-reloc
  dari uniform (fisika cermin). **[V]**
- **Fitur baru ditambah:** MirrorModeTracker, gaze active-vision recovery, line-heading
  yaw-insurance, ZUPT vision-only, kontrak `/fall` (FREEZE ingest saat jatuh), gate C2
  ref-blend anti-racun. **[V]**
- **Baseline berubah:** 0.336 m (AMCL) → **0.054 m** (landmark). **[V]**
- **Scope:** murni-vision di lokalisasi (IMU hanya boleh di *behavior* untuk deteksi jatuh);
  hardware (Orin/turf) **ditunda** — semua angka di atas dari **simulasi**. **[V]**

## 10. Visual yang Layak Ditampilkan (maks 4)

1. **Diagram arsitektur pipeline** (§3): Kamera → YOLO → proyeksi → solver → mirror → EKF →
   pose. Perlihatkan aliran no-odom & lapisan robustness (gaze/line/ZUPT).
2. **Localization map live** (`live_viz.py`): peta lapangan skala Webots dengan panah EKF
   (cyan) menempel di panah ground-truth (putih) + titik fix mentah — bukti visual akurasi ~5 cm.
3. **Before–after / grafik error-vs-waktu:** baseline AMCL 0.336 m vs landmark 0.054 m,
   plus kurva blackout+rotasi yaw p95 176°→11° (nilai line-heading).
4. **Screenshot deteksi landmark** (detection_image YOLO): kotak L/T/X/goalpost/circle pada
   frame Webots — menunjukkan sumber persepsi pipeline.

---

## 11. Draft Ringkas untuk Portfolio (English)

### Project Description  *(≤70 kata)*
Pure-vision self-localization for an OP3 humanoid soccer robot (ROS2/Webots). A single camera
detects typed field landmarks (L/T/X junctions, goalposts, center circle) with YOLOv8n; a
closed-form geometric solver plus an odometry-free EKF estimate the robot's pose on the field.
Custom modules break the field's 180° mirror symmetry and keep tracking robust when the robot
looks down or faces the goal net.

### My Contribution  *(≤4 bullet)*
- Built an odometry-free geometric localization pipeline (4 new ROS2 packages) replacing an
  ambiguous line-based AMCL that failed at full-field scale.
- Designed a deterministic 180° mirror side-lock tracker, taking mirror-locks from frequent to
  **0%** on walking tests.
- Added active-vision gaze recovery, line-heading yaw insurance, and vision-only ZUPT to close
  camera-blackout and EKF runaway failure modes.
- Generated an 8000+1200-frame auto-labeled landmark dataset and trained/deployed a YOLOv8n
  detector (OpenVINO INT8).

### Technical Highlights  *(≤4 bullet)*
- Odometry-free EKF driven by ~13 Hz closed-form geometric fixes (WLS + Mahalanobis
  data-association + RANSAC).
- Evidence-driven engineering: pre-registered criteria, ≥5-run confirmation, and **disabling**
  features (chi²-gate, cond-inflation) proven unhelpful.
- Achromatic min-RGB line extractor that overturned a "line-scan not viable" conclusion,
  restoring yaw observability during blackout.
- 28 unit tests on the localization core; exact camera model (0.0000 m median reprojection).

### Key Result  *(≤3 bullet)*
- **~5 cm** walking-track accuracy (median), **~6× better** than the 0.336 m AMCL baseline.
- **0% mirror-lock, 0 flips**; re-entry localization 0.097–0.230 m, converging in 0–3 s.
- Heading error under blackout+rotation cut **176° → 11° p95 (−93.5%)** via line-heading.

### Tech Stack  *(satu baris)*
ROS2 Humble · Python/C++ · YOLOv8n (Ultralytics) · OpenVINO/TensorRT INT8 · robot_localization EKF · OpenCV/NumPy · Webots R2023b.

---

## Data yang masih perlu saya berikan
- **Angka benchmark detektor** (mAP@50/50-95, precision, recall per kelas) dari model terlatih.
- **FPS/latency** end-to-end di target hardware (NUC/Orin) — masih ditunda.
- **Validasi hardware/turf** (sim-to-real) — belum dilakukan; semua angka dari simulasi.

## Missing Information (maks 5 pertanyaan untuk memperkuat portofolio)
1. Berapa **mAP/precision/recall** model YOLOv8n terlatih (ada file hasil training/benchmark)?
2. Apakah portofolio ini **individu** atau bagian tim KRSBI — mana bagian yang murni Anda vs tim
   (mis. gait, GameController, paket `soccer_*` warisan)?
3. Sudahkah ada **uji di robot fisik/turf** dengan angka, atau seluruhnya masih simulasi?
4. Apakah TMMIN lebih menilai sisi **AI/persepsi** (YOLO/dataset) atau **sistem/kontrol**
   (EKF/robustness) — agar penekanan draft bisa disesuaikan?
5. Berapa **durasi & peran** Anda di project ini (mis. bulan, sendiri/berapa orang) untuk baris
   ringkas di CV?
