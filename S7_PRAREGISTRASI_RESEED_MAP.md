# S7 (opsi 2) — Pra-registrasi: peta keandalan re-seed (DITULIS SEBELUM DATA)

**Tanggal kunci:** 2026-08-24. SOP pengguna: re-seed manual tiap robot ditempatkan. Tujuan:
petakan **di mana penempatan** re-seed mengunci sisi benar seketika vs **rawan** (mirror /
lambat). Diduga zona bahaya = **dekat-tengah** (twin sejati & mirror berimpit; deadband 0.5 m).

## Metode
Untuk tiap penempatan (x, y, yaw=0): teleport robot ke sana → verifikasi GT → seed pose sama
(`seed_side.py`) → diam 5 s → ukur `ekf` vs `gt`. Robot BERDIRI (bukan walk); lokalisasi harus
mengunci sisi dari fix landmark. yaw=0 dijaga (hindari set_pose-theta yang flaky; jarak-ke-tengah
= variabel utama yang memicu ambiguitas mirror).

## Penempatan (sapu x, y=0, fokus lintas-tengah)
x ∈ {−3.0, −2.0, −1.0, −0.5, −0.25, +0.25, +0.5, +1.0, +2.0, +3.0} m. (Baris y=±1.5 opsional
lanjutan bila perlu.)

## Klasifikasi per penempatan (kunci sebelum data)
- **LOCK-OK**: |yaw_err| < 10° DAN pos_err < 0.30 m, stabil dalam 5 s.
- **MIRROR**: |yaw_err| > 90° (terkunci twin salah) ATAU pos_err ≈ 2|x| (lompat ke sisi seberang).
- **AMBIGU/LAMBAT**: di antaranya / belum stabil (deadband tengah).

## Kriteria & keluaran
- **Sukses peta**: identifikasi **radius-bahaya** dekat-tengah (|x| di mana mulai gagal) + konfirmasi
  LOCK-OK di luar itu. Tandai zona-aman-penempatan untuk SOP.
- **Temuan tak-terduga**: bila ada penempatan JAUH-dari-tengah yang gagal (mis. sudut lapangan
  miskin-landmark) → catat sbg zona rawan tambahan.
- Deliverable: tabel x → kelas + kalimat "tempatkan di |x| > R m untuk lock instan".

## HASIL (2026-08-24) — SISI ROBUST DI SELURUH LAPANGAN; prediksi zona-tengah SALAH
Peta (berdiri, n=1/penempatan, seed pose benar):
| x[m] | pos_err | yaw_err | kelas |
|---|---|---|---|
| −3.0 | 0.218 | 3.6° | LOCK-OK |
| −2.0 | 0.936 | 0.3° | AMBIGUOUS |
| −1.0 | 0.140 | 0.1° | LOCK-OK |
| −0.5 | 0.123 | 1.5° | LOCK-OK |
| −0.25 | 0.077 | 0.1° | LOCK-OK (dead center) |
| +0.25 | 0.028 | 0.1° | LOCK-OK (dead center) |
| +0.5 | 0.705 | 0.1° | AMBIGUOUS |
| +1.0 | 0.199 | 0.1° | LOCK-OK |
| +2.0 | 0.917 | 1.0° | AMBIGUOUS |
| +3.0 | 0.208 | 0.4° | LOCK-OK |

- **SISI/mirror: ROBUST DI SELURUH LAPANGAN** — yaw_err < 4° di 10/10 penempatan, **0 mirror-lock**,
  termasuk dead-center (±0.25). **Prediksi zona-bahaya-tengah SALAH.** Untuk SOP: **penempatan di
  mana pun aman-mirror**.
- 3 "AMBIGUOUS" (x=−2.0/+0.5/+2.0) = **offset POSISI 0.7–0.9 m dgn yaw sempurna** — bukan mirror,
  tapi **bias-fix-berdiri** (view statis kasih fix bias; sepupu ringan false-minima 8a). Berdiri
  saja → bias tak terata; **jalan mengubah view → diduga konvergen** (8c = 5 cm jalan).
- CAVEAT: n=1/penempatan (pelajaran ball-tracking); 3 offset bisa transien / berdiri-saja.
- TINDAK LANJUT: uji-ulang 3 titik itu + walk pendek → konfirmasi konvergen (bias berdiri-saja).

## FOLLOW-UP (2026-08-24) — 2/3 flag = noise; 1 titik lemah nyata
| titik | map(berdiri) | uji-ulang berdiri | setelah walk 15s | verdict |
|---|---|---|---|---|
| x=−2.0 | 0.94 | 0.11 | 0.018 | NOISE → LOCK-OK |
| x=+0.5 | 0.71 | 0.12 | 0.013 | NOISE → LOCK-OK |
| x=+2.0 | 0.92 | 0.95 | 0.47 | **TITIK LEMAH NYATA** |

## PUTUSAN FINAL OPSI 2
1. **Re-seed MIRROR-SAFE di seluruh lapangan** (yaw sempurna 10/10, 0 mirror, termasuk dead-center).
   SOP: tempatkan robot di mana pun → sisi terkunci benar. Prediksi zona-tengah SALAH.
2. **Akurasi posisi excellent hampir di mana-mana** (<0.25 m, konvergen ~cm saat jalan).
3. **Titik lemah tunggal: x≈+2 m (deep opponent-half, menghadap gawang lawan)** — bias ~0.5 m
   persisten bahkan setelah jalan (view ill-conditioned didominasi tiang, rezim A2). Bukan mirror
   (yaw sempurna). Region jarang ditempati → caveat akurasi, bukan blocker.
4. Pelajaran metode (lagi): 2/3 flag awal = noise n=1 → follow-up wajib sebelum vonis.
