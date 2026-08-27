# S6 (opsi 1) — Pra-registrasi: konflik gaze bola vs lokalisasi (DITULIS SEBELUM DATA)

**Tanggal kunci:** 2026-08-24. Kondisi main paling realistis yang BELUM diuji: di pertandingan
kepala **mengejar bola** (ball_tracker), sering menunduk dalam (tilt < −35° → valid-fix ~0%,
GATE 4.4-A) & menyamping — landmark cuma terlihat sesekali. Semua uji sebelumnya pakai gaze
**ramah-lokalisasi**. Pertanyaan: apakah lokalisasi bertahan saat kepala sibuk dengan bola?

## Setup
Publish pola **head-tracking-bola** ke `/robotis/head_control/set_joint_states` (JointState
`[head_pan, head_tilt]`) selama walk seeded: tilt turun-dalam ~−45° (variasi −30..−55°, bola
dekat/jauh), pan berayun ±0.6 rad (ikuti bola kiri-kanan), ~10 Hz. Stack match (LH ON).
`ball_head_sim.py` (baru). head_control_module di-enable oleh gaze node (atau manual saat gaze off).

## Dua kondisi
- **A — realistis (gaze ON):** gaze node BERSAING merebut kepala saat lokalisasi memburuk.
  Ukur akurasi bersih + **berapa sering gaze merebut kepala** (= berapa sering robot "kehilangan
  bola" untuk relokalisasi — tradeoff nyata pertandingan).
- **B — kasus terburuk (gaze OFF):** kepala lengket ke bola, TANPA scanning recovery. Lantai.

## Metrik
pos RMSE/median/p95, yaw RMSE, **fresh fix-rate** (seberapa lapar saat kepala nunduk),
**mirror%/flips** (integritas), (A) **jumlah gaze-takeover**, kontribusi line-heading (yaw@langka).
Baseline pembanding = walk gaze-ramah (~0.05–0.10 m, mirror 0).

## KRITERIA (dikunci sebelum data)
- **BERTAHAN** bila: **integritas utuh** (mirror 0% / flips 0) DAN pos p95 **< ~0.5 m** (bisa-main)
  DAN yaw tetap teramati (line-heading tahan **< ~10°** walau landmark langka).
- **TRADEOFF terukur** (A): frekuensi gaze-takeover = biaya "lepas bola demi lokalisasi".
  Bila sangat sering (mis. tiap < ~3 s) → catat sbg biaya desain nyata.
- **CELAH NYATA** (perlu kerja) bila: integritas pecah (mirror/flips > 0) ATAU pos runaway
  (p95 > ~0.5 m / divergen) ATAU yaw lepas (> ~15° berkelanjutan).

## Putusan
Bertahan + tradeoff kecil → lokalisasi tahan kondisi main nyata (angka ditulis). Tradeoff besar
→ rekomendasi kebijakan (mis. gaze hanya merebut kepala saat benar-benar lost, bukan tiap gap).
Integritas pecah → celah nyata, prioritaskan.

## HASIL KONDISI A (2026-08-24) — BERTAHAN
Walk lurus + ball_head_sim (tilt −45°±12°, pan ±0.6) + gaze ON (bersaing). 1 run, 44s eval.
| metrik | A (ball-tracking) | baseline gaze-ramah |
|---|---|---|
| pos median | 0.115 m | ~0.05 m |
| pos p95 | **0.410 m** (< 0.5 ✓) | ~0.10 m |
| yaw RMSE | **1.95°** (< 10 ✓) | <1° |
| mirror / flips | **0% / 0** ✓ | 0% / 0 |
| gaze-takeover (head ke horizon) | **30.2%** (median tilt −37°) | — |
- **BERTAHAN**: integritas utuh, pos playable (p95 0.41 m), yaw ditahan (line-heading+ZUPT saat
  down-gaze). Bukan celah.
- **Tradeoff terukur**: gaze merebut kepala ~30% waktu = robot lepas-pandang-bola ~30% demi
  relokalisasi. Nyata tapi tak katastrofik.

## HASIL KONDISI B (2026-08-24) — gaze OFF; B > A (kontra-intuitif)
Sama seperti A tapi gaze OFF (kepala lengket bola 100% down, median −44°). 1 run.
| metrik | A (gaze ON, 30% takeover) | B (gaze OFF) |
|---|---|---|
| pos median | 0.115 | 0.114 |
| pos p95 | 0.410 | **0.203** |
| yaw RMSE | 1.95° | **1.23°** |
| mirror/flips | 0%/0 | 0%/0 |
- **Dua-duanya BERTAHAN**; tapi **B lebih baik** dari A (p95 & yaw). Mekanisme: down-gaze mantap
  (−44°) TETAP cukup — line-heading tahan yaw + fix junction near-field jaga posisi (raw fix
  436/440). Di A, gaze menyentak kepala 30% → gerak-leher cepat → kualitas fix turun (head-sync
  lag/blur) → akurasi lebih buruk meski lebih sering lihat horizon.
- **INSIGHT**: gaze-takeover agresif saat tracking-bola = biaya 30% lepas-bola untuk manfaat
  akurasi NEGATIF. Robot lebih baik pertahankan pandang-bola + andalkan line-heading+ZUPT+near-field.
- **CAVEAT**: n=1/kondisi → urutan A<B sugestif, belum terkunci; dua-duanya playable, integritas utuh.
- **REKOMENDASI KEBIJAKAN** (perlu ≥3-5 run konfirmasi sebelum ubah): kurangi agresivitas gaze saat
  ball-tracking aktif (jangan rebut kepala tiap fix-gap). Integritas TAK terancam di kedua mode.

## KONFIRMASI (kriteria ditulis sebelum data, 2026-08-24)
3 run A (gaze ON) vs 3 run B (gaze OFF), setup identik ball-tracking. Bandingkan **median-of-p95**
dan **median yaw RMSE** antar-kondisi.
- **Rekomendasi DITERIMA** (kurangi agresivitas gaze) bila: B **tak lebih buruk** dari A
  (B p95 ≤ A p95 dalam noise) DAN integritas utuh (mirror 0/flips 0) di SEMUA run → gaze-takeover
  tak memberi manfaat akurasi, jadi biaya 30% lepas-bola tak terbayar.
- **Rekomendasi DITOLAK** (pertahankan gaze agresif) bila: A jelas lebih baik dari B (gaze-takeover
  menurunkan p95/yaw secara konsisten) → takeover berbayar.

## HASIL KONFIRMASI (n=3 masing-masing, 2026-08-24) — REKOMENDASI DITOLAK oleh data
| n=3 | A (gaze ON) | B (gaze OFF) |
|---|---|---|
| pos median-of-med | **0.118 m** | 1.568 m |
| pos p95 (median) | **0.262 m** | 2.563 m |
| yaw RMSE | **~2.2°** | 4.7–66° |
| mirror/flips | 0/0 | 0/0 |
- **A jelas jauh lebih baik** dari B → cabang "PERTAHANKAN gaze agresif" menyala. Gaze-takeover
  **LOAD-BEARING** saat ball-tracking; tanpanya lokalisasi drift 1.5–2.5 m (near-field fix + line-
  heading saja TAK cukup andal di down-gaze −44°).
- **n=1 sebelumnya SALAH**: single-B 0.20 m = outlier beruntung; konfirmasi menangkapnya.
- **PUTUSAN FINAL opsi 1**: (1) lokalisasi **BERTAHAN kokoh** di kondisi ball-tracking realistis
  DENGAN gaze (A: p95 0.26 m, yaw 2.2°, mirror 0%, n=3); (2) kebijakan gaze saat ini (rebut kepala
  saat degrade) **BENAR & wajib** — JANGAN dikurangi. Integritas sisi utuh di kedua mode.
- Nilai proses: uji konfirmasi mencegah perubahan kebijakan yang keliru berdasar 1 run.
