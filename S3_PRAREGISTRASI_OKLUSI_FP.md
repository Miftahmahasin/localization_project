# S3 — Pra-registrasi sapuan Oklusi & FP (DITULIS SEBELUM DATA)

**Tanggal kunci:** 2026-08-24 · Otorisasi pengguna. Batasan: bobot lebih ke **FP** (robot lain
menyerupai tiang/garis; rezim **presisi > recall**, konsisten GATE 1 & B3).

## Setup
Stack penuh default (line-heading ON, gaze ON), walk-through-centre seeded (run_seeded_8c),
degradasi via `degrade_relay` (T1 `use_degrade:=true`). Walk 60 s, 2 run/titik.

## Sumbu & titik
- **FP (utama):** `fp_per_frame` = 0 → 1 → 2 → 3 (fp_classes = junction [0,1,2], paling merusak).
- **Oklusi (pembanding, ringan):** `recall` = 1.0 → 0.7 → 0.5 (semua kelas), FP=0.

## Metrik
pos RMSE/median/p95, yaw RMSE, **mirror% & side-flips** (laju salah-cocok proksi), fresh fix-rate.
Titik-patah = laju FP (atau recall) saat mirror%/flips>0 ATAU pos p95 > ~0.30 m (ambang permainan).

## PREDIKSI (dikunci sebelum data)
1. **FP rendah (≤1/frame):** asosiasi {L,T} (C5) + RANSAC + gate Mahalanobis menahan → pos/yaw
   ~baseline (pos median ~0.10 m), mirror 0%, flips 0.
2. **Titik-patah ~2–3 FP/frame:** false-match mulai menangkap fix → mirror% naik / pos p95 > 0.30 m.
3. **Oklusi lebih ramah:** recall 0.5 masih terlacak (fix-rate turun tapi cukup) — FP jauh lebih
   merusak daripada oklusi pada laju setara (bukti kuantitatif "presisi>recall").

## Putusan
Bila titik-patah FP terlampaui pada laju wajar (≤ maks-robot KRSBI, **pengguna verifikasi rulebook**)
→ perkuat asosiasi/mirror lalu ukur ulang. Bila kokoh sampai laju ekstrem → catat margin sbg
ketahanan-FP terukur.

## HASIL (2026-08-24) — INTEGRITAS SISI BULLETPROOF; presisi>recall terkuantifikasi
Stack penuh default (LH ON), walk 60 s, 2 run/titik.

### Sapuan FP (junction) — mirror 0% / flips 0 di SEMUA titik
| FP/frame | pos RMSE | median | p95 | yaw RMSE | mirror | flips |
|---|---|---|---|---|---|---|
| 0 | 0.072 | 0.053 | 0.103 | 0.41° | 0% | 0 |
| 1 | 0.093 | 0.040 | 0.151 | 0.69° | 0% | 0 |
| 2 | 0.178 | 0.072 | 0.296 | 1.14° | 0% | 0 |
| 3 | 0.197 | 0.079 | 0.318 | 3.17° | 0% | 0 |

### Sapuan oklusi (recall) — jauh lebih ramah
| recall | pos RMSE | median | p95 | yaw RMSE | mirror | flips |
|---|---|---|---|---|---|---|
| 1.0 | 0.072 | 0.053 | 0.103 | 0.41° | 0% | 0 |
| 0.7 | 0.127 | 0.073 | 0.168 | 0.61° | 0% | 0 |
| 0.5 | 0.117 | 0.060 | 0.226 | 0.84° | 0% | 0 |

### Perbandingan insult setara (~3 landmark/frame terganggu)
FP=3: pos 0.197 / p95 0.318 / yaw **3.17°**  vs  recall=0.5: pos 0.117 / p95 0.226 / yaw **0.84°**.
→ FP merusak yaw **~4×** & pos **~1.7×** lebih dari oklusi setara = **PRESISI > RECALL** terbukti angka.

### PUTUSAN
- **Tak ada titik-patah integritas** (mirror/flips=0) di rentang realistis; asosiasi/mirror TAK perlu
  diperkuat. Margin ketahanan-FP terukur: kokoh sampai 3 junction-FP/frame (ekstrem) tanpa mirror.
- Batas-lunak akurasi (pos p95 > 0.30 m) hanya tercapai di FP≥2–3/frame.
- Implikasi ke S5 (spec detektor): utamakan **presisi** (sedikit FP) di atas recall — detektor yang
  melewatkan landmark aman; yang berhalusinasi junction mahal.
- Prediksi: P1✓ P3✓ ; **P2 terlalu pesimis** (integritas jauh lebih tahan dari dugaan).
- Verifikasi pengguna: maks robot/tim KRSBI (untuk memetakan 3 FP/frame ke laju lapangan nyata).
