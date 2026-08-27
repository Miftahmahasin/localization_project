# S2 — Pra-registrasi substitusi get-up (DITULIS SEBELUM DATA)

**Tanggal kunci:** 2026-08-24 · Otorisasi pengguna. Konteks: get-up FISIK tak berfungsi di Webots
(uji CLI + GUI dua-duanya nihil; GT z tetap 0.072 saat page 122/123) → blocker gait/physics.
Substitusi: reproduksi **kondisi pasca-get-up** (robot tegak lagi, posisi ~sama, **heading tak
menentu**) via teleport-stand, uji apakah lokalisasi memulihkan heading.

## Arsitektur yang disepakati (kontrak `/fall`, BUKAN IMU langsung di lokalisasi)
- Lokalisasi **murni-vision**: hanya subscribe event abstrak `/fall`, tak pernah sentuh IMU.
- fall_detector = node BEHAVIOR terpisah; sumber IMU: sim `/robotis_op3/imu` (11.7 Hz, jalan),
  hardware `/robotis/open_cr/imu` (0 Hz di sini; validasi hardware = tugas terpisah). Risiko
  "IMU hardware jalan/tidak" terisolasi di node itu; kontrak lokalisasi identik.
- Get-up FISIK = milik gait/behavior; substitusi sim = teleport-stand.

## Rencana dua fase
- **P1 (diagnostik, TANPA kode baru):** dari konvergen, teleport-rotate ke (x,y sama, theta+180°)
  = "get-up mendarat heading salah". Ukur apakah stack yang ADA (mirror-tracker + kidnap-reloc +
  C2 + ZUPT) memulihkan heading. Karakterisasi celah.
- **P2 (bila P1 kurang):** bangun kontrak `/fall` di geometric_pose_node — saat `/fall`=true:
  **suppress publish** (bukan cuma ref-blend; supaya fix-sampah saat jatuh tak masuk EKF) + coast
  ZUPT; saat `/fall`=false (pulih): picu **reloc fokus-heading** (inflasi cov-yaw). Uji ulang.

## METRIK & KRITERIA SUKSES (dikunci)
- **waktu-pulih heading**: dari saat gangguan sampai |yaw_err| < **10°** dan bertahan 3 s.
- **kebenaran sisi**: berakhir di sisi TRUE (mirror 0% pasca-pulih), flips tak menambah lock salah.
- **posisi terbatas**: tak runaway (pos tetap < ~0.5 m dari benar pasca-pulih).
- **SUKSES** bila: pulih heading **≤ ~10 s** (setara kidnap 8b ~7.8 s), berakhir TRUE, no runaway,
  konsisten di **≥5 gangguan**; **8b/8c baseline tak turun**.
- **GAGAL/perlu P2** bila: heading tak pulih / pulih > ~15 s / berakhir mirror / EKF terjebak heading lama.

## Substitusi penuh siklus (opsional, P2): pulsa `/fall`
Bila butuh menguji fase "jatuh" (fix-sampah), publish `/fall`=true sebelum teleport-rotate dan
`/fall`=false sesudah — mensimulasikan behavior mendeteksi jatuh→pulih, TANPA menjatuhkan robot fisik.

## HASIL P1 (2026-08-24) — stack ADA GAGAL pulih heading (celah nyata)
Teleport-rotate 180° @t=4.8s: ekf_yaw_err lompat ke 179.6° dan **tak pernah turun** (179° selama 30s).
AKAR: kidnap-recovery dipicu ketidaksepakatan **POSISI** (kidnap_resid_m=2.0 m); get-up = heading-saja,
posisi ~sama → resid≈0 → 'lost' tak pernah menyala → recovery tak jalan. fix_yaw ~0 (single-corner
warisi prior heading lama; full-fix dipetakan ke sisi committed by position) → sistem duduk di keadaan
salah-heading yang self-consistent. => celah nyata (berlaku untuk bump-and-spin apa pun), P2 DIBUTUHKAN.
**Bangun P2 disetujui.**

## HASIL P2 + PUTUSAN FINAL (2026-08-24)
- **OPSI 2 (auto heading-reloc) GAGAL**: harness robust (spin-wait rotasi + walk) → yaw tetap ~179°
  walau berjalan. AKAR: backend asosiasi memakai NILAI prior (heading lama); error ~180° membuat
  tiap fix (single-corner warisi prior; full-fix salah-asosiasi ke fitur peta terputar 180°)
  self-consistent SALAH. Inflasi VARIANS-yaw EKF tak menolong (backend pakai nilai, bukan varians).
  = problem **relokalisasi global (8a)**: ~50% di view statis, butuh jalan+waktu → **DITUNDA**.
- **OPSI 1 (re-seed) BEKERJA SEMPURNA**: seed pose benar (-2.5,0,180) pada robot terjebak →
  yaw_err **0.3–0.4° INSTAN** (<0.1s), tahan 8s. Matang (8b/GATE5), sah-KRSBI (robot masuk/
  ditempatkan di pose diketahui).
- **DESAIN FINAL S2**: `/fall`=true → **FREEZE** (suppress ingest, lindungi EKF dari fix-sampah
  saat jatuh); pemulihan fall yang mengubah heading = **RE-SEED** (/initialpose, sudah didukung,
  tanpa kode lokalisasi baru); auto-reloc heading (OPSI 2) di-REVERT (rusak) + didokumentasikan
  sbg relokalisasi-global tertunda. IMU tetap di produser /fall (behavior). **S2 SELESAI (OPSI 1).**

## KOREKSI CAKUPAN (2026-08-24) — re-seed = MANUAL, dan celah jatuh-tengah-main
Klarifikasi penting (pertanyaan pengguna): re-seed itu **MANUAL** (operator, pose diketahui), BUKAN
otomatis; robot tak menyimpan-lalu-reseed sendiri. Cakupan sebenarnya:
- **TERTUTUP oleh re-seed manual**: penempatan (kickoff/keluar/penalti/setup) + **PICK-UP wasit**
  (robot incapable → diangkat → ditempatkan di pose diketahui).
- **CELAH NYATA — jatuh-lalu-bangkit-sendiri di tengah main**: TAK bisa disentuh/re-seed manual;
  auto-recovery heading = relokalisasi global (ditunda). Ide "simpan koordinat" memberi POSISI benar
  (jatuh tak geser posisi) tapi HEADING salah (jatuh ubah heading) → belum solusi penuh (persis
  alasan P2 gagal).
- **Praktis**: jika get-up hardware TAK jalan → semua jatuh → PICK-UP → re-seed manual (tertutup).
  Jika get-up jalan → celah-heading jatuh-tengah-main tetap ada (butuh relokalisasi global).
