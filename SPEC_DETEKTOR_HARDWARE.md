# SPEC PENERIMAAN DETEKTOR LANDMARK — untuk fase hardware (S5)

**Disintesis dari harness sim (2026-08-24):** sapuan cutoff-jarak (S1), sapuan FP & recall (S3),
komposisi-kelas offline (A2/GATE 4). Setiap angka ditandai **[ukur]** (terukur di sim) atau
**[ekstrapolasi]** (butuh konfirmasi hardware/1 sapuan tambahan). Tujuan: kalimat uji yang bisa
dipakai menerima/menolak model detektor turf sebelum dipercaya untuk lokalisasi.

---

## 1. PRIORITAS KELAS — junction (L,T,X) ESENSIAL
- **[ukur, A2 val n=1200]** 99,9% fix-valid butuh ≥1 junction; junction-saja → 85,9% fix-rate.
  Goalpost+center_circle **saja** → hanya 30,8% (ill-conditioned: cond p95 2121→130509).
- **[ukur, S1 live]** Saat SEMUA junction difilter, goalpost+circle masih menutupi di dekat gawang/
  tengah (fix 8–14/s) — **tapi** itu bergantung posisi; tak bisa diandalkan sebagai sumber utama.
- **PUTUSAN:** detektor **wajib** mendeteksi junction andal. Goalpost & center_circle = pelengkap
  (menutup celah, anti-mirror sekunder), **bukan** pengganti junction.

## 2. PRESISI > RECALL (persyaratan dominan)
- **[ukur, S3 FP sweep]** FP junction merusak **~4× (yaw)** & **~1,7× (pos)** lebih dari oklusi setara.
  - ≤**1 FP junction/frame** → lokalisasi ≈ baseline (pos ~0.09 m, mirror 0%, flips 0).
  - 2–3 FP/frame → pos p95 lewat 0.30 m (ekor), **tapi integritas sisi tetap** (mirror 0%, flips 0).
- **[ukur]** Integritas sisi/mirror **tak pernah pecah** sampai 3 FP/frame (asosiasi {L,T}+RANSAC+
  gate Mahalanobis+mirror-tracker). Tak ada titik-patah integritas di rentang realistis.
- **SPEC:** setel conf-threshold / NMS ke arah **presisi tinggi**: target **≤1 FP junction/frame**
  rata-rata. Lebih baik melewatkan junction daripada berhalusinasi junction.

## 3. RECALL junction (dalam jangkauan pakai)
- **[ukur, S3 recall sweep]** recall global **0.5** → EKF pos masih ~0.12 m, yaw <1°, mirror 0%,
  flips 0 (fix-partial single-corner + tapis EKF menyerap kehilangan). recall 0.7 → ~0.13 m.
- **SPEC:** **recall junction ≥ 50%** (dalam jangkauan pakai, lihat §4) memadai untuk tracking
  baseline. Di bawah ~50% risiko kelangkaan fix meningkat (belum diuji < 0.5 hidup).

## 4. JANGKAUAN — junction ≤ R meter
- **[ukur, error_model]** gate valid_range junction = **6 m** (di atas itu galat proyeksi meledak).
- **[ukur, S1 cutoff]** saat landmark >2 m dibuang, muncul kelangkaan bertahap (fix single bertahan);
  >1.5 m dibuang → blackout total. Jadi fungsi lokalisasi butuh junction terdeteksi di **jarak dekat–
  menengah**, bukan cuma jauh.
- **[ekstrapolasi]** Target penerimaan: **recall junction ≥50% pada ground-range ≤ 3.5 m**, menurun
  boleh di 3.5–6 m. Angka 3.5 m = kompromi (cukup fitur dalam pandang saat berdiri/jalan); **1 sapuan
  `recall_dist_slope` hidup** akan memakukan R eksak bila diperlukan.

## 5. KALIMAT UJI PENERIMAAN (ringkas, untuk model turf)
> **Detektor DITERIMA bila, pada rekaman turf pertandingan:**
> **(a)** recall junction (L/T/X) **≥ 50%** untuk junction pada ground-range **≤ 3.5 m**; **DAN**
> **(b)** false-positive junction **≤ 1 /frame** rata-rata (presisi-utama); **DAN**
> **(c)** goalpost & center_circle terdeteksi sbagai pelengkap (tak wajib per-frame).
> Recall/presisi diukur setelah proyeksi ke ground (yang salah-range otomatis ter-gate valid_range).

## 6. GAZE (berpasangan dengan spec ini)
- **[ukur, GATE 4.4-A]** fix plateau di head tilt −5..−25°; jatuh tajam < −30° (down-gaze tendang).
- **[ukur, S1]** saat down-gaze/blackout, **line-heading** menahan yaw (kolaps 108°→11° saat berputar).
- **SPEC:** detektor tak perlu bekerja di down-gaze ekstrem; gaze-policy (TAHAP 6A) mengangkat kepala
  untuk merebut fix, line-heading menutup lubang yaw sementara.

## 7. DITUNDA (butuh hardware/Orin)
- **fps & latensi end-to-end** di Orin (anggaran inferensi) — belum ada Orin. Bisa pakai rekaman frame
  Webots (tak butuh turf) saat Orin tersedia.
- Angka **3.5 px/junction @imgsz320** dari direktif — **belum diverifikasi**; ukur tinggi box dari
  dataset sebelum dijadikan fakta (pengguna).
- Maks robot/tim KRSBI → memetakan "≤1 FP/frame" ke laju lapangan nyata (pengguna verifikasi rulebook).
