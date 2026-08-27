# Laporan Progres — Lokalisasi Landmark Geometris (OP3, ROS2/Webots)

> Dokumen serah-terima. Diperbarui **2026-08-23** berdasarkan perjalanan pengembangan
> & debugging. Ditujukan untuk rekan yang akan meninjau / melanjutkan.
> Nada laporan: **apa adanya** — yang berhasil disebut berhasil dengan angka; yang
> gagal, rapuh, atau masih terbuka disebut jujur (termasuk jalur buntu yang saya coba
> agar tidak diulang). Hal yang berpotensi diperbaiki saya angkat sebagai
> **permintaan saran** di §10.

---

## 1. Ringkasan eksekutif

Kita mengganti lokalisasi lama (AMCL scan garis putih yang **ambigu fundamental**)
dengan **lokalisasi geometris berbasis landmark bertipe** (L/T/X junction, goalpost,
center-circle) yang dideteksi YOLO, dihitung pose absolutnya secara **closed-form
(CLAP/ILM) + WLS**, lalu difusikan ke **EKF** (`robot_localization`). **Tanpa
odometri badan sama sekali** (odom gait = command-echo, buta terhadap slip — dibuang;
diganti fix per-frame yang cukup sering).

**Status inti (semua terbukti angka):**
- ✅ **Tracking no-odom saat bergerak** — berjalan RMSE **0.208–0.209 m** (baseline
  v10.17 = 0.336 m), tanpa odom. Tesis inti proyek.
- ✅ **Mirror 180° — SOLVED.** Gap yang paling lama menyita waktu sesi ini kini
  tertutup: side-hold lewat pusat lapangan **flips 0**, dan **seed sisi yang andal**
  memastikan `ref` tidak pernah terkunci ke cermin. Batch re-entry terakhir:
  **mirror 0% di 5/5 run**.
- ✅ **Re-entry ala-lomba (8b)** — robot ditaruh di titik masuk, di-seed, lanjut main
  (jalan): **5/5 TRUE, pos RMSE 0.230 m, mirror 0%, flips 0**.
- ✅ **Gaze recovery (6A)** memulihkan blackout down-gaze (~2–4 s), tak mengganggu
  tracking sehat (0.102 m / 0.98°).
- ✅ **Line-scan CV (6B)** bekerja di Webots (min-RGB akromatik; ~40–59% publish).
- ✅ **EKF static-runaway diperbaiki** (fix proses-noise kecepatan; lihat §5.4).
- ⚠️ **Global-init dari NOL (tanpa seed)** tetap **rapuh** (~2/3). Ini **dibatalkan
  sebagai deliverable** — di lomba, sisi diberi GameController (seed sah), bukan
  ditebak dari geometri (mirror tak terpecah sensor). Lihat §6 & §10.
- ⚠️ **Robot kadang jatuh** (~1/5 run) di physics baseline — **isu kestabilan gait,
  di luar lokalisasi**; kini dampaknya jinak (EKF tak lagi runaway saat robot diam).

**Kesimpulan singkat:** lokalisasi no-odom **saat robot bermain (berjalan) sudah
matang & andal**, termasuk mirror-hold dan re-entry pasca-penalti. Yang tersisa
bukan lagi lokalisasi, melainkan **integrasi (seed dari GameController) dan
kestabilan gait**.

---

## 1a. UPDATE lanjutan (2026-08-23) — regression suite, ZUPT, asosiasi per-profil

Sesi lanjutan mengunci robustness dan **memperbaiki akurasi 8b ~2×**. Baseline 8b
re-entry kini **median-of-medians 0.097 m** (dari 0.199 m), RMSE mean **0.129 m**
(dari 0.353 m), **mirror 0%, flips 0, 5/5 konvergen** — tervalidasi `run_regression.sh`.

- **C1 — Regression suite.** Satu perintah (`run_regression.sh`) menjalankan batch 8b/8c
  + gate PASS/FAIL vs baseline. Gate: posisi = **median-across-runs dari median tiap run**
  (steady-state; kebal transien re-entry), mirror/flips = **hitung run outlier, toleransi
  ≤1** (varians fisika inheren; regresi nyata kena ≥2 run). Melindungi tiap perubahan.
- **C3 — ZUPT vision-only.** Akar runaway lama: EKF no-odom **coasting** pada kecepatan
  hantu saat tak ada fix baru (mis. kidnap teleport → fix mati → lari 16 m). Perbaikan
  struktural: `geometric_pose_node` menerbitkan **zero-twist ke `/zupt`** tiap frame coast
  (mirror `hold`/`lost`/no-obs) + saat gait `stop`; EKF `twist0` mem-pin kecepatan ~0 →
  **tahan pose**, bukan lari. Q kecepatan dikembalikan wajar (0.02). **Runaway hilang 5/5.**
- **C5 — Asosiasi per-profil (penemuan akurasi).** Grup agnostik lama `{L,T,X}` membiarkan
  silang-cocok **X↔T** yang menambah noise **bahkan pada tracking berbibit**. Default sim
  kini **`{L,T}`** (X distinctif → cocok by-type saja); hardware `{L,T,X}`. Ini yang
  membawa akurasi **~2× lebih baik**. Param: `assoc_agnostic_group`.
- **C4 — Inflasi cov cond-number.** Mekanisme `_cond_inflate` terpasang (PoseFit.cond),
  **default MATI** — `cov=info⁻¹` sudah menggembungkan arah lemah; opt-in untuk view
  degenerate, aktifkan+ukur nanti. Bukan gate (tak menolak, hanya membobot).
- **Protokol run Webots (PENTING).** Skrip walk = **persis alur GUI**:
  `enable_ctrl_module "walking_module"` (inilah yang mendirikan robot) → params → start →
  stop. **JANGAN pernah `ini_pose`** (base_module init pose = jongkok; transisi ganda
  menumpuk lalu menjatuhkan robot). Bila robot kolaps: **reload Webots** (Ctrl+Shift+R) —
  memulihkan berdiri bersih + `stand_z_ref_` benar. Teleport hanya aman saat robot berdiri.
- **chi2-gate — DITUTUP PERMANEN** (bukan "menunggu kalibrasi"): prior EKF dibangun DARI
  fix, jadi gating fix-vs-prior = **umpan balik positif** (menolak fix yang justru
  mengoreksi drift). Outlier ditangani no_teleport (prior-independen) + C4 (bobot, bukan
  tolak). Lihat §5.1.

---

## 2. Arsitektur (ringkas)

```
YOLO landmark detector ──► landmark_projector ──► geometric_pose_node ──► EKF ──► /odometry/filtered
 (image_rect → boxes)      (box → titik tanah)   (asosiasi+WLS+mirror)   (pose2)   (TF map→odom)
                                  ▲                         ▲
                          Projector BERSAMA          prior = EKF output      /initialpose (seed sisi)
                          (1 sumber proyeksi,        (bukan odom)            = GameController di lomba
                           kalibrasi beku -5°)
```

Prinsip yang dipatuhi (aturan main proyek):
1. **Satu** modul proyeksi kamera bersama (`landmark_geometry.Projector`) — dipakai
   generator label **dan** runtime (invariant: kalibrasi runtime = kalibrasi sampler).
2. Landmark **tidak pernah** masuk particle filter AMCL. Backend = geometris → EKF.
3. Tidak mewajibkan 2-landmark simultan (single-corner + akumulasi temporal).
4. **Presisi > recall** (landmark palsu = fix absolut salah, jauh lebih merusak).
5. Lapor dengan **distribusi** (median/p95/n), bukan kata sifat.

---

## 3. Status per tahap

| Tahap | Isi | Status | Angka kunci |
|---|---|---|---|
| 4 (no-odom EKF) | fix per-frame → EKF, single-corner partial vs coast | ✅ | walk 145 s **0.209 m**; **partial menang** (gap 1.56 s vs coast 16.24 s) |
| 5 (mirror-hold) | side-lock deterministik + kidnap recovery | ✅ | lewat-pusat **TRUE 100% flips 0** |
| 6A (gaze policy) | angkat kepala ke horizon saat lokalisasi memburuk | ✅ | pulih **2.2–3.6 s**; tracking bersih **0.102 m/0.98°** |
| 6B (line-scan co-primary) | garis → heading mod 90° → EKF (bukan AMCL) | ✅ bekerja | min-RGB akromatik → **~40–59% publish, strength 0.70** |
| 7 (hardware fps) | profil fps Orin/NUC @imgsz 320/640 | ⏳ ditunda (di laptop dulu) | — |
| 8a (global-reloc dari nol) | tanpa seed | ⚠️ dibatalkan sbg deliverable | ~2/3 (mirror + false-min; seed memecah) |
| **8b (re-entry/kidnap)** | seed di titik masuk, lanjut jalan | ✅ | **5/5 TRUE, 0.230 m, mirror 0%, flips 0** |
| **8c (tracking walk)** | walk lewat-pusat, seeded, gaze on | ✅ | **5/5 TRUE, 0.208 m, mirror 0%** |

---

## 4. Hasil terukur (tervalidasi)

- **Walking straight 145 s (TAHAP 4):** pos RMSE **0.209 m**, median 0.098, p95 0.507,
  yaw 3.04°, sisi TRUE 98%. → tesis inti terbukti: fix geometris per-frame ganti odom.
- **8c tracking (seeded, walk lewat-pusat, gaze on, 5 run):** **5/5 TRUE, mirror 0%,
  pos RMSE 0.208 m** (mean batch bersih), konvergen instan.
- **8b re-entry (seed di titik masuk, lanjut jalan, 5 run):** **5/5 TRUE, mirror 0%,
  flips 0, pos RMSE mean 0.230 ± 0.101 m** (min 0.108, max 0.354), median **0.149 m**,
  yaw 3.25°, konvergen 5/5 (0–2.8 s).
- **Single-corner:** **partial-update MENANG** atas coast (coast → blackout 16 s
  saat menghadap jaring).
- **6A gaze:** trigger via gap; recovery 2.2 s; kepala kembali hadap-depan setelah pulih.
- **6A clean tracking (diam, gaze idle):** pos 0.102 m, yaw **0.98°** — node gaze idle
  tak mengganggu.

---

## 5. Investigasi inti sesi ini — memecahkan MIRROR & merapikan EKF

Fokus sesi ini: menutup **mirror 180°** (gap yang berulang) dan menuntaskan
deliverable evaluasi (8b/8c). Semua berbasis pengukuran & instrumentasi persisten,
bukan tebakan. Beberapa hipotesis saya **salah** dan saya buang — dicatat agar tidak
diulang.

### 5.1 Chi-square gate fix-vs-prior (B3.1) — DICOBA, MERUGIKAN, DIMATIKAN
Hipotesis: kegagalan mirror = "konvergen lalu satu fix outlier menyeret ke cermin",
ditutup gate Mahalanobis fix-vs-prior. **A/B live membantahnya:** dengan seed andal,
gate **OFF** = 5/5 TRUE (0.208 m); gate **ON** (ambang 16.27) **lebih buruk** — mirror
16.8%, flips rata-rata 5.8 (satu run 25 flips), konvergen 12 s. Sebab: seed membuat
prior ketat seketika, sehingga gate paling agresif justru saat prior percaya-diri →
**menolak fix sah** → osilasi. **Gate default → 0 (OFF)**; kode disimpan sebagai
opt-in bila nanti ada kalibrasi ambang yang benar.

### 5.2 Akar mirror-drag DITEMUKAN dari instrumentasi persisten
Saya tambah **`mirror_diag_csv`** (per-frame `ref` sebelum/sesudah, `kind`, `contra`,
raw-fix, chosen, reloc, prior — ke file, bukan log layar). Menyelaraskan dengan run
gagal menunjukkan mekanisme **pasti**:
1. Robot mendarat **jongkok** pasca-teleport (tanpa berdiri ulang) → kamera turun →
   **fix sampah ber-yaw ~−160°** → di-blend sebagai 'ok' → **`ref` terseret** dari
   (benar, yaw 0°) ke (garbage, yaw −160°).
2. **Re-seed sampai ke EKF tapi TIDAK ke `mirror.ref`** — burst `/initialpose`
   tertelan banjir callback 13 Hz (langganan depth 5).
3. `ref` ber-yaw −160° → pemilihan kembar (yaw-weighted) **mengunci MIRROR** (yaw
   kembar ~180° lebih dekat ke −160° daripada TRUE 0°).

### 5.3 Perbaikan mirror (terpasang, terbukti 5/5 mirror 0%)
- **Seed andal** (`scripts/seed_side.py`): publish `/initialpose` **10×** (3 s),
  menunggu subscriber; langganan node `/initialpose` **depth 5 → 20** (burst tak
  tertelan). Seed kini otoritatif ke **EKF DAN mirror.ref**.
- **Re-stand sebelum trust:** harness re-entry **berjalan dulu** (stance benar) lalu
  re-seed saat fix bersih — `ref` tak lagi terkorup fix-sampah-jongkok.
- Ditambah perbaikan sebelumnya: cross-class agnostik dibatasi ke **{L,T,X}**;
  center_circle/goalpost wajib kelas sendiri (mematikan false-minimum center_circle→L/T).

### 5.4 EKF static-runaway (residual EKF) — DIPERBAIKI
Gejala (8b): saat robot **jatuh**, RAW fix tetap **benar** (~0.8 m) tapi **EKF lari**
ke 8 m lalu 48 m. Akar: EKF no-odom **menaksir kecepatan** dari delta-pose; robot
berhenti → EKF *coast* pada kecepatan basi; fix beku ditolak → drift→tolak→drift.
**Jalur buntu (dicatat):** menaikkan `pose2_pose_rejection_threshold` (3→25) **gagal**
(malah 48 m) — cov ketat membuat fix apa pun banyak-sigma. **Lever yang benar =
proses-noise KECEPATAN:** `vx,vy` 0.04→**0.0008**, `vyaw`→0.001 (EKF **hampir
konstan-posisi**, tak bisa coast jauh tanpa sensor kecepatan); ambang dikembalikan
moderat (8.0). Dengan fix ~13 Hz (gerak ~0.015 m/fix), tracking 0.208 m tak melambat;
hanya menukar ~0.5 m ekstrapolasi blackout (ditutup gaze 6A ~2 s). Kini EKF **menahan
fix terakhir** saat robot down, bukan runaway.

> **DISUPERSEDE oleh C3 (§1a).** Q-kecepatan kecil (0.0008) menghentikan *static-stop*
> tapi **tak bisa meredam kecepatan yang disuntik teleport kidnap** (kr_run2 masih lari
> 16 m). Perbaikan struktural final: **ZUPT vision-only** (zero-twist `/zupt` di frame
> coast) mem-pin kecepatan → EKF tahan pose; Q dikembalikan wajar (0.02). Runaway hilang
> **5/5**. Q-kecil bukan lagi lever utama.

### 5.5 Framing deployment (penting, dari diskusi aturan RoboCup)
Mirror **tak terpecah sensor** (dua gawang identik) — **untuk semua tim**. Di lomba,
sisi diberi **eksternal**: penempatan awal di paruh sendiri + **GameController**
(tim/sisi, status penalti, titik re-entry). Jadi "seed" **bukan tambalan** — ia
memakai info yang aturan sediakan. `/initialpose` manual kita = **stand-in**
GameController. **Recovery otonom tanpa seed** = *stretch goal* sulit, dan tak
diperlukan bila re-seed dipicu event permainan.

### 5.6 Konfigurasi fisis Webots
Untuk eval, fisis world = **baseline** (hapus `contactProperties`+`defaultDamping`,
spawn z=0.3). **Kamera tetap 1920×1080** (kalibrasi projector beku ke resolusi ini).
Config "kaku" untuk dataset disimpan byte-for-byte di
`worlds/robotis_op3_extern.DATASET.wbt`; resep restore di `worlds/WEBOTS_CONFIG_NOTES.md`.

---

## 6. Yang KOKOH vs yang RAPUH (ringkas untuk peninjau)

| Kokoh (terbukti angka) | Rapuh / di luar lokalisasi |
|---|---|
| Tracking no-odom bergerak (0.208–0.209 m) | Global-init dari NOL tanpa seed (~2/3) — *dibatalkan sbg deliverable* |
| **Mirror-hold + seed andal (mirror 0%, flips 0, 5/5)** | Kestabilan **gait** (robot jatuh ~1/5) — isu physics/walking, bukan lokalisasi |
| Re-entry ala-lomba (0.230 m, 5/5) | Integrasi **GameController** (seed otomatis) — belum diwire |
| Gaze recovery + EKF tak-runaway | Manfaat kuantitatif line-scan — belum diukur |
| Kidnap se-sisi (seed) pulih | Sim-to-real detektor (rumput Webots ≠ turf nyata) |

---

## 7. Perubahan kode sesi ini (changelog)

- `soccer_object_localization/config/ekf_soccer_landmark.yaml` — **[FIX runaway]**
  proses-noise `vx,vy` 0.04→0.0008, `vyaw`→0.001; `pose2_pose_rejection_threshold`
  3→8.0.
- `landmark_localization/landmark_backend.py` — chi-square gate fix-vs-prior
  (`_chi2`, `chi2_gate`, `BackendResult.gated_chi2/chi2`); **default OFF** (0.0).
- `landmark_localization/geometric_pose_node.py` — param `chi2_gate` (OFF) &
  `mirror_diag_csv` (**[BARU]** diagnostik persisten); langganan `/initialpose`
  depth 5→**20**.
- `landmark_localization/scripts/seed_side.py` — **[BARU]** seed `/initialpose` andal
  (10× / 3 s, tunggu subscriber) → otoritatif ke EKF + mirror.ref.
- `landmark_localization/scripts/run_seeded_8c.sh`, `run_kidnap_8b.sh` — **[BARU]**
  orkestrator semi-otomatis (jeda terparametrik; re-entry = walk-dulu-lalu-reseed).
- `association.py` (sesi sebelumnya, tetap) — cross-class agnostik dibatasi {L,T,X};
  center_circle/goalpost same-class.
- `soccer_object_localization/launch/localization_v15_landmark.launch.py` — arg
  `chi2_gate`, `mirror_diag_csv`.
- `camera_info_publisher.py` / `landmark_projector.py` (A1) — fallback K derivasi
  FOV (bukan hardcode) + re-latch bila resolusi berubah; test invariansi resolusi CI.

**Lanjutan 2026-08-23:**
- `ekf_soccer_landmark.yaml` — **[C3]** input `twist0: /zupt` (pilih vx,vy,vyaw); Q
  kecepatan **dikembalikan 0.02** (ZUPT yang menghentikan runaway, bukan Q kecil lagi).
- `geometric_pose_node.py` — **[C3]** publisher `/zupt` + zero-twist tiap frame coast +
  langganan `/robotis/walking/command`; **[C5]** param `assoc_agnostic_group` (default
  sim {L,T}) + **[C4]** `cond_inflate_at/cap`; komentar chi2-gate → **ditutup permanen**.
- `landmark_backend.py` — **[C5]** `agnostic_group` diteruskan ke `DataAssociator`;
  **[C4]** `_cond_inflate` (default mati); komentar chi2-gate → ditutup permanen.
- `localization_v15_landmark.launch.py` — **[C5]** `assoc_agnostic_group: [0,1]` (sim),
  **[C4]** `cond_inflate_at/cap`.
- `scripts/walk_op3.py` — **[protokol]** buang `ini_pose` total; alur = GUI persis
  (`walking_module` mode → params → start → stop).
- `scripts/run_regression.sh` + `regression_gate.py` — **[C1/BARU]** suite satu-perintah;
  gate median-of-medians + toleransi-1-outlier.
- `test/test_backend.py` — +6 test (C4/C5); total **25/25 lulus**.

Semua paket build bersih; test unit lolos (asosiasi+backend 8/8 incl. 3 chi2 baru,
mirror 6/6, line-heading 8/8, resolusi 5/5).

---

## 8. Cara menjalankan / verifikasi

```bash
cd ~/basbot && source install/setup.bash
export SCRIPTS=src/motion_webots/src/localization_ws/landmark_localization/scripts

# T1 (Webots + op3_manager + stack lokalisasi) — SATU BARIS:
ros2 launch soccer_object_localization localization_v15_landmark.launch.py detector:=yolo use_gaze:=true use_line_heading:=false

# Seed sisi ANDAL (deployment = dipicu GameController; di sim = manual):
python3 $SCRIPTS/seed_side.py --x -2.5 --y 0 --yaw 0

# Eval batch semi-otomatis (jeda diatur di header skrip):
$SCRIPTS/run_seeded_8c.sh 5 c_run      # tracking walk
$SCRIPTS/run_kidnap_8b.sh 5 kr_run     # re-entry (RESEED=true)
python3 $SCRIPTS/landmark_multirun.py --label "skenario" c_run*.csv

# Diagnostik mirror persisten (bila menyelidiki flip): tambah mirror_diag_csv:=/path.csv ke T1
```

---

## 9. Permintaan saran (mohon masukan rekan)

Titik-titik berikut **berpotensi diperbaiki / butuh keputusan** — masukanmu sangat membantu:

1. **Integrasi GameController (prioritas #1 deployment).** Seed sisi kini andal &
   otoritatif via `/initialpose`. Yang belum: **memicunya otomatis** dari event
   permainan (Ready → pose formasi; unpenalized → titik re-entry). Apakah tim kita
   sudah punya jembatan GameController→ROS? Titik re-entry resmi liga kita di mana,
   dan apakah komunikasi antar-robot diizinkan (membuka opsi disambiguasi via tim)?

2. **Kestabilan gait (robot jatuh ~1/5).** Ini di luar lokalisasi tapi merusak run.
   Apakah sebaiknya pindah dari `op3_walking_module` (walk_op3) ke
   `op3_online_walking_module`, atau ada tuning gait/balance yang kamu sarankan untuk
   physics Webots baseline? (Dampak ke lokalisasi kini jinak — EKF tak runaway.)

3. **Recovery otonom tanpa seed.** Global-init/kidnap-tanpa-seed masih ~2/3 (mirror +
   false-minimum junction di view miskin). Karena lomba menyediakan seed, kami
   **membatalkannya sebagai deliverable**. Setuju memperlakukannya sebagai *stretch
   goal*, atau kamu lihat nilai membangun FSM-init / MHL-consensus untuk robustness
   ekstra (mis. bila GameController down)?

4. **Asosiasi X-distinctive vs X-agnostik.** Kini agnostik dibatasi {L,T,X}. Ukuran
   offline: X-distinctive sedikit lebih baik saat detektor andal (mis-assoc 0.123 vs
   0.141) tapi recall X kolaps saat class-confusion tinggi. Default mana untuk sim
   vs hardware? (Sudah jadi parameter — tinggal pilih.)

5. **Line-scan co-primary — worth?** CV min-RGB bekerja (~40–59% publish). Tapi
   **manfaat kuantitatif belum diukur** (yaw saat down-gaze dengan vs tanpa
   line-heading). Layak dijadikan co-primary tetap, atau cukup andalkan gaze-recovery
   (6A)? Butuh satu eval terarah.

6. **EKF near-konstan-posisi (tradeoff).** Kami turunkan proses-noise kecepatan untuk
   mematikan runaway; ini menukar ~0.5 m ekstrapolasi saat blackout down-gaze (gaze
   6A menutupnya ~2 s). Ada preferensi model lain (mis. ZUPT eksplisit saat fix beku)
   ketimbang menurunkan Q kecepatan?

7. **Hardware (Orin Nano / NUC @imgsz 320).** Risiko junction L/T/X tak terdeteksi di
   resolusi rendah → apakah degradasi ke goalpost+circle cukup? (Ukuran offline:
   junction ESENSIAL — valid-fix all-classes 93.6% vs goalpost+circle-only 30.8%.)
   Plus **sim-to-real**: model hanya melihat rumput Webots; pre-train TORSO-21
   dijalankan atau mitigasi apa bila recall junction anjlok di turf nyata?

Terima kasih — masukan pada titik-titik di atas menentukan langkah berikutnya.
