# Laporan — Pekerjaan yang MASIH bisa diselesaikan di SIM + permintaan saran
### Lokalisasi Landmark Geometris OP3 · full-vision · ROS2/Webots · rev 2026-08-23

> Konteks: core lokalisasi sudah **matang di sim** (8b re-entry **~0.10 m**, mirror 0%,
> flips 0, **runaway 0**, C1–C7 selesai, 28/28 unit test). Hardware **ditunda** (butuh
> Orin/NUC + turf). Dokumen ini menyaring **apa yang masih bisa dikerjakan TANPA hardware**,
> diurut nilai, dengan **permintaan saran** di tiap keputusan terbuka.

---

## Penilaian singkat
- **Core full-vision di sim: ~82/100** — bagian tersulit secara teori (no-odom, cermin 180°,
  runaway, kualitas asosiasi) sudah dipecahkan & terukur.
- **Sistem siap-lomba: ~58/100** — penentu terbesar (hardware sim-to-real) belum disentuh.
- Yang di bawah ini menaikkan angka **sim** dari ~82 → mendekati plafon-sim, TANPA besi.

---

## S1 — Validasi DEFINITIF nilai-asuransi line-heading (regime junction-langka) · **TIER 1**
**Kondisi:** C6-live menunjukkan line-heading **tak membantu yaw saat junction sehat**
(0.772° ON vs 0.774° OFF). Hipotesis asuransi: nilainya baru muncul saat **junction langka**
(offline: 99.9% fix bergantung junction; ~47% hilang bila junction lenyap). **Ini belum
dibuktikan langsung** — C6-live pakai detektor sehat.
**Bisa di sim:** degradasi detektor terkontrol (filter kelas L/T/X, atau turunkan recall
junction via param), lalu ulang yaw ON vs OFF. Jika ON jauh lebih baik saat junction langka →
asuransi TERBUKTI (bukan sekadar argumen); jika tidak → line-scan bisa ditutup, hemat.
**→ SARAN:** (a) buktikan di sim, atau cukup argumen offline? (b) cara degradasi mana yang
paling mewakili kegagalan hardware — filter kelas total, atau recall-drop stokastik naik-jarak?

## S2 — Loop recovery jatuh (vision-only) · **TIER 1 (tergantung aturan)**
**Kondisi:** ZUPT menahan pose saat jatuh (error terbatas), tapi **tak ada alur eksplisit**
"terdeteksi jatuh → setelah bangun → re-seed/re-lokalisasi". Proksi jatuh vision =
hilangnya fix-valid berkelanjutan saat gait aktif (tanpa IMU).
**Bisa di sim:** deteksi fase tak-percaya-fix + minta re-seed/auto-reloc saat pulih.
**→ SARAN (menentukan apakah perlu):** Di **KRSBI**, saat robot jatuh — apakah **wasit
mengangkat & menaruh ulang** robot (→ itu = re-entry, **re-seed manual sudah cukup**, loop
otomatis tak perlu)? Atau robot **bangun sendiri di tempat** (→ butuh auto-reloc vision)?

## S3 — Robustness oklusi & deteksi palsu · **TIER 2**
**Kondisi:** belum diuji — robot lain menutupi landmark, atau deteksi palsu (false-positive).
Di lapangan nyata ini sering; asosiasi bisa salah-cocok.
**Bisa di sim:** injeksi ke pipeline (drop landmark stokastik meniru oklusi + false-positive
kelas acak), ukur degradasi pos/yaw & laju salah-cocok, perkuat asosiasi/mirror bila perlu.
**→ SARAN:** seberapa padat oklusi realistis di KRSBI (berapa robot di lapangan, seberapa
sering & lama landmark tertutup)? Ini menentukan seberapa keras kita perlu mensimulasikan.

## S4 — Validasi lapisan pertahanan C2/C4 pada skenario terdegradasi · **TIER 2**
**Kondisi:** C2 (ref-blend anti-racun) & C4 (inflasi cov cond) **terpasang + unit-test**, tapi
**belum divalidasi pada skenario peracunan/view-miskin NYATA** di sim (baseline bersih tak
memicunya). C4 masih default-mati (belum ada bukti manfaat).
**Bisa di sim:** skenario terkontrol (crouch dipaksa / view 2-landmark) untuk mengukur (i) C2
benar menahan racun ref, (ii) C4 mengurangi false-minima → lalu **tune atau tutup** C4 dengan
data.
**→ SARAN:** prioritaskan validasi-skenario ini, atau terima C2/C4 sebagai defensif
terbukti-unit-test saja (dan tutup C4 bila tak ada skenario pemicu di sim)?

## S5 — Kurva degradasi pipeline penuh vs recall detektor · **TIER 3 (opsional)**
**Kondisi:** `fix_rate_eval` sudah model class_collapse untuk *fix-rate*; belum ada kurva
**pos/yaw error & laju-valid vs recall menurun** untuk pipeline penuh (titik-patah).
**Bisa di sim:** sapu recall/collapse, plot error & valid-rate → kurva keputusan sim-to-real
(“di recall X, lokalisasi mulai runtuh”).
**→ SARAN:** berguna sebagai angka keputusan menjelang hardware, atau berlebihan untuk sekarang?

---

## Rangkuman permintaan saran
1. **S1:** buktikan asuransi line-heading di sim (junction-langka), atau cukup argumen offline? Cara degradasi mana?
2. **S2 (paling menentukan):** aturan KRSBI untuk robot jatuh — wasit taruh-ulang (re-seed manual) vs bangun-di-tempat (auto-reloc)?
3. **S3:** kepadatan oklusi realistis KRSBI untuk model injeksi?
4. **S4:** validasi-skenario C2/C4 diprioritaskan, atau terima defensif + tutup C4?
5. **S5:** kurva degradasi pipeline berguna sekarang, atau tunda?

> Catatan jujur: **global-init tanpa seed tetap mustahil sensor-only** (fisika cermin) — bukan
> item yang bisa "diselesaikan", melainkan batas yang sudah diterima (seed eksternal = solusi).
> Akurasi mentah (0.10 m) sudah baik; menaikkannya lagi = imbal-hasil menurun, bukan prioritas.

---

# HASIL EKSEKUSI (2026-08-24) — S1–S5 SELESAI

Semua via harness terpadu **TAHAP H** (`degrade_relay` live-tunable + `harness_analyze.py`
split-metrik + `run_degrade_point.sh`), kriteria ditulis **sebelum data** (file `S*_PRAREGISTRASI*`).

| Item | Putusan | Bukti ringkas |
|---|---|---|
| **S1 line-heading** | **PERTAHANKAN** (default ON) | Blackout+rotasi: yaw p95 **176°→11°** dgn LH (turun 93.5%). Null saat ada landmark (C6). Regresi 8b LH-ON PASS (0.108m, mirror 0). `S1_PRAREGISTRASI_LINE_HEADING.md` |
| **S3 oklusi/FP** | Integritas bulletproof; **presisi>recall** | mirror 0%/flips 0 di FP 0-3 & recall 0.5-1.0. FP merusak yaw ~4× & pos ~1.7× > oklusi setara. Tak perlu perkuat asosiasi. `S3_PRAREGISTRASI_OKLUSI_FP.md` |
| **S5 spec detektor** | Ditulis | recall junction ≥50% @≤3.5m + FP junction ≤1/frame (presisi-utama). `SPEC_DETEKTOR_HARDWARE.md` |
| **S4.1 C2** | **Tervalidasi live** | 788 fix resid-tinggi → 0 meracuni ref; 188 fix bersih → blend. `S4_C2_C4.md` |
| **S4.2 C4** | **DITUTUP permanen** | cond-ON sama/lebih-buruk, mirror 0% tanpa C4 → tak ada manfaat. Inert (0.0). |
| **S2 recovery jatuh** | **OPSI 1 (re-seed)** | get-up fisik = blocker gait (nihil di Webots). Auto heading-reloc = relokalisasi global (8a, ditunda). Re-seed INSTAN (0.4°). Kontrak `/fall`=FREEZE; recovery=re-seed. `S2_PRAREGISTRASI_GETUP_SUBSTITUSI.md` |

**Perubahan runtime (semua inert-by-default, baseline tak turun):** launch `use_line_heading`
default→**true** (profil match); geometric_pose_node: kontrak `/fall` (FREEZE), kolom `resid_m`
di mirror_diag, arg `cond_inflate_at` (default 0.0). 28/28 unit test. IMU HANYA di behavior
(produser `/fall`), EKF lokalisasi tetap murni-vision.

**Tertunda (butuh hardware/riset):** auto heading-reloc pasca-jatuh = relokalisasi global (8a);
fps-profiling Orin; validasi IMU hardware (`/robotis/open_cr/imu`); angka rulebook KRSBI
(maks robot/tim, ambang PICK UP) — verifikasi pengguna.

**REGRESI FINAL (node final, 2026-08-24): PASS** — 8b median-of-med 0.120 m, mirror 0%, flips 0, runaway 0, 5/5 converge. Baseline TAK turun dengan semua perubahan sim-solvable (S1-S5) aktif.

**REGRESI 8c FINAL (node final, LH ON, 2026-08-24): PASS** — tracking median-of-med **0.054 m** (~5 cm), mirror 0%, flips 0, 5/5 TRUE, runaway 0, yaw <1.1°. Historis 8c 3/5 bersih → kini **5/5 bersih** (C-fix menuntaskan mirror). DoD "8b/8c tetap baseline" TERPENUHI penuh.
