# S1 — Pra-registrasi kriteria putusan line-heading (DITULIS SEBELUM DATA)

**Tanggal kunci:** 2026-08-24 · **Status:** dikunci sebelum pilot dijalankan (disiplin S1: kriteria
ditulis sebelum melihat data). Otorisasi pengguna: "saya izinkan kedua opsi nya" (setujui Z + izin pilot).

## Metrik (dari `harness_analyze.py`, dipisah per direktif H2)
- **(a) yaw @ jendela junction-langka** — `ekf_yaw_err` [deg] pada frame di dalam rentetan
  ≥5 frame beruntun tanpa fix-geometris baru. **Di sinilah asuransi line-heading berada.**
  Bandingkan **A = line-heading ON** vs **B = OFF**. Juga ekor p95 seluruh-run.
- **(b) error @ frame fix-baru** — pos/yaw pada frame ber-fix (potongan yang C6-live ukur; nol wajar).
- **fix-rate** — laju fix-geometris baru (kurva degradasi detektor, harus ~cocok A vs B).

## KRITERIA PUTUSAN Z (dikunci)
**PERTAHANKAN line-heading** bila menghapus junction membuat:
- **B (OFF) memburuk ke tingkat merugikan-permainan:** yaw **p95 seluruh-run > ~8°**,
  **SEMENTARA** A (ON) menahannya **< ~5°** — **ATAU**
- **yaw p95 @ jendela-langka turun ≥40%** (A vs B).

**TUTUP line-scan** (kurangi 1 node inferensi sebelum hardware) bila:
- **B tetap baik tanpa junction:** yaw **p95 seluruh-run < ~5°** (line-heading tak menambah nilai).

Zona antara (B p95 5–8° dan A tak jelas lebih baik) → **tak konklusif** → perlu sapuan penuh
(cutoff jarak → recall) sebelum putusan, bukan tebakan.

## Pilot (1 titik: filter junction total × A/B, 2 run masing-masing, walk 60 s)
- Kondisi A: T1 `use_line_heading:=true use_degrade:=true` → `run_degrade_point.sh filt_on 2 'filter_classes [0,1,2]'`
- Kondisi B: T1 `use_line_heading:=false use_degrade:=true` → `run_degrade_point.sh filt_off 2 'filter_classes [0,1,2]'`
- Analisis: `harness_analyze.py --a filt_on*.csv --b filt_off*.csv`

## HASIL PILOT (2026-08-24) — Z TIDAK DIPICU (confounded)
Filter-junction-total `[0,1,2]` **gagal menciptakan kelangkaan**: goalpost(3)+center_circle(4)
tak difilter → fix tetap 8–14/s, gap terpanjang ≤0.33 s di A **dan** B.
- fresh fix-rate A=52% vs B=96%; frame-langka A=34.6% vs **B=0.3% (n=5)** → **tak sebanding**.
- yaw seluruh-run p95: A=0.795° B=0.664° (dua-duanya « 5°).
- Cabang "tutup" harfiah cocok (B<5°) TAPI B tak pernah menghadapi kelangkaan → **tidak sah**.
- Valid: harness bekerja end-to-end; baseline tak turun (pos ~0.10 m, mirror 0%, flips 0);
  sisi-A menahan yaw p95 0.055° di 533 frame langka (menjanjikan, belum keputusan).
- **Klaim "99,9% andalkan junction" runtuh di live** — goalpost+circle menutupi.

## KOREKSI SAPUAN PENUH (disetujui 2026-08-24)
Generator kelangkaan = **cutoff jarak SEMUA kelas** `cutoff_classes=[0,1,2,3,4]`, sapu
R = {2.0, 1.5, 1.0} m (filter_classes direset ke [-1]). Kelangkaan muncul di A **dan** B →
`yaw@langka` sebanding terhadap Z. Kriteria Z di atas **tidak berubah**.

## HASIL SAPUAN CUTOFF (2026-08-24) — line-heading ON MEMPERBURUK yaw di kelangkaan
Sapuan cutoff-jarak semua-kelas R={2.0,1.5,1.0} m, A(LH ON) vs B(LH OFF), 2 run/titik:
| R | scarce | A yaw@langka p95 | B yaw@langka p95 |
|---|---|---|---|
| 2.0 | ~70% | 11.6° | 6.2° |
| 1.5 | 100% | 10.6° | 5.7° |
| 1.0 | 100% | 11.3° | 7.0° |
Konsisten 3 titik: LH ON ~2× lebih buruk. Mekanisme: garis lapangan ambigu 90°/180°;
tanpa landmark utk disambiguasi, estimasi LH mendorong yaw EKF salah; OFF cukup ZUPT-beku
di seed (benar). Cocok dgn C6-live (dgn landmark: 0.772°≈0.774°, LH tak menambah).

### CAVEAT METODOLOGIS (WAJIB sebelum putusan final)
Walk uji **LURUS** (angle=0) → yaw sejati nyaris tak berubah → yaw-beku B optimal by-construction
→ uji **bias ke B**. Skenario di mana LH seharusnya menang = blackout **sambil BERPUTAR**
(yaw-beku B jadi basi, LH bisa melacak rotasi). Belum diuji. → butuh 1 titik konfirmasi:
blackout (R=1.0) + walk berputar (angle≠0), A vs B. LH tetap kalah saat berputar → TUTUP tegas
(berbahaya). LH menang → PERTAHANKAN, gated ke blackout.

## UJI PUTAR — DECISIVE (2026-08-24): line-heading ESENSIAL saat blackout+rotasi
Blackout total (R=1.0, 0 fix), spin di tempat 6°/step (~270°), 2 run/kondisi:
| | A (LH ON) | B (LH OFF) |
|---|---|---|
| yaw median | 5.56° | 108.47° |
| yaw p95 | 11.45° | 176.38° |
Kriteria Z cabang "PERTAHANKAN" menyala telak: **yaw p95@langka turun 93.5%** (11.4 vs
176.4°) ≫ ambang ≥40%. Tanpa LH, yaw kolaps ~108° saat robot berputar tanpa landmark
(ZUPT beku tak bisa melacak rotasi); dgn LH terlacak ke 5.6°.

### GAMBARAN LENGKAP (nilai LH KONDISIONAL)
- Normal (ada landmark): null (0.77°≈0.77°, C6-live).
- Blackout + walk LURUS: LH sedikit lebih buruk (p95 ~11° vs ~6°; ambiguitas garis vs
  yaw-beku yg kebetulan benar).
- Blackout + ROTASI: LH esensial (p95 11° vs 176°).
Penalti lurus (~5° p95) sepele vs bencana 165° saat berputar; dan blackout+rotasi tak bisa
diprediksi → **always-on menang secara nilai-harapan**. Rotation-gating = micro-opt tak perlu.

## PUTUSAN S1 (FINAL): PERTAHANKAN line-heading — asuransi heading esensial, always-on.
Tindak lanjut SELESAI (2026-08-24): `use_line_heading` default ON (launch line 71, rebuilt) +
regresi 8b LH-ON PASS — pos median-of-med 0.108 m, mirror 0%, flips 0, runaway 0, 5/5 converge.
Baseline TAK turun dgn line-heading aktif.
