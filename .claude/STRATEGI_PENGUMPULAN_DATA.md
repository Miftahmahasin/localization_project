# STRATEGI PENGUMPULAN DATA — Sampling Pose Dataset Landmark Webots

**Pendamping untuk**: `STRATEGI_YOLO_LANDMARK.md` (Fase 1.1) dan `PROMPT_CLAUDE_CODE_DATASET_WEBOTS.md`
**Konteks**: generator `landmark_dataset_capture` sudah bekerja & terverifikasi visual (goalpost benar sebagai kelas terpisah, junction di perpotongan asli). Dokumen ini menjawab: **pose apa yang harus disampel, dan berapa banyak.**

---

## 0. Klarifikasi: bukan "jalur", tapi distribusi pose
Di simulasi, robot di-teleport instan ke banyak pose **independen** (bukan berjalan menyusuri jalur). Jadi "robot digerakkan mana ke mana" sebenarnya = **distribusi pose apa yang disampel.** Jalur berjalan hanya relevan bila butuh data berurutan (video); untuk melatih detektor per-frame, pose independen lebih efisien dan lebih beragam.

## 1. Kenapa sampling seragam sederhana TIDAK cukup
Kalau hanya sebar posisi acak seragam:
- **Kelas langka kurang terwakili.** X-cross hanya di sekitar pusat; L-corner hanya di sudut lapangan + sudut penalty area. Sampling seragam → sebagian besar frame tak memuatnya → detektor lemah justru pada **corner, yang merupakan landmark paling berharga** (mengikat x & y).
- **Jarak tak terstratifikasi.** Landmark jauh (kecil, sulit) jarang muncul → detektor buruk di jarak jauh, padahal itu yang paling butuh koreksi.
- **Sudut kepala tak terwakili.** Kalau semua di tilt≈−10°, detektor gagal saat active vision menoleh ke atas mencari gawang.

→ Butuh **stratified sampling**: coverage luas + oversample kelas/kondisi langka.

## 2. Dimensi yang WAJIB di-cover
1. **Posisi (x,y)** — seluruh lapangan, kedua paruh (simetri).
2. **Heading** — 0–360° penuh.
3. **Sudut kepala pan/tilt** — KRITIS karena active vision. Samakan dengan range yang benar-benar di-command behavior cari-bola + gaze-lokalisasi (tarik dari kode head-scan yang ada). Gawang butuh tilt ke atas; garis dekat butuh tilt ke bawah.
4. **Jarak ke landmark** — near (1–2m) / mid (2–4m) / far (4m+), per kelas.
5. **Partial/edge** — sebagian pose dengan landmark di tepi frame (realistis; landmark sering terpotong).

## 3. Strategi sampling (3 lapis)

**Lapis A — Coverage luas (dasar):**
- Grid posisi tiap ~0.5m + jitter acak.
- Di tiap posisi: sweep heading (mis. tiap 30–45°).
- Di tiap heading: sampel pan/tilt kepala dari range operasi.

**Lapis B — Landmark-centric (jamin kelas langka):**
Untuk TIAP kelas, khususnya X-cross & tiap tipe corner, generate batch pose yang sengaja diposisikan + diorientasikan agar landmark itu di FOV, di jarak & sudut bervariasi. Contoh konkret "mana ke mana":
- **Dekat tiap gawang** (4 posisi) → goalpost + goal-area corners.
- **Sekitar pusat lapangan**, menghadap berbagai arah → X-cross, center-line T, center circle.
- **Dekat tiap sudut lapangan** (4) → L-corner.
- **Sepanjang tiap sideline** → T-junction sideline.
Ini menjamin corner (paling berharga) cukup banyak, tak bergantung keberuntungan sampling.

**Lapis C — Stratifikasi jarak & edge:**
- Pastikan tiap kelas punya contoh near/mid/far.
- Sisipkan sebagian pose dengan landmark di tepi frame.

## 4. Target keseimbangan kelas
- Setelah generate, HITUNG distribusi kelas (berapa L / T / X / goalpost).
- Kalau ada kelas < ambang (mis. < setengah kelas terbanyak), tambah batch Lapis-B untuk kelas itu. Data sim murah → oversample kelas lemah gratis.

## 5. Kuantitas & split
- Mulai **~3000–5000 gambar berimbang** (karena pre-train di TORSO-21, fine-tune tak butuh jutaan). Evaluasi, tambah bila kelas lemah.
- **Train/val split**: generate val sebagai run sampling TERPISAH & independen (~10–15%). Tiap pose teleport independen → risiko kebocoran rendah, tapi jaga val benar-benar terpisah.

## 6. Domain randomization (murah, bantu sim-to-real)
Variasikan pencahayaan, tekstur/warna rumput sedikit, noise kamera antar-batch.

## 7. Isu yang harus diselesaikan DULU (dari screenshot capture)
- **center_circle bbox**: saat robot dekat/di dalam lingkaran, lingkaran membungkus robot → bbox ill-defined (lebar-tipis di horizon / raksasa). Bbox bukan representasi baik untuk kurva besar. Putuskan: **(a)** drop center_circle sebagai kelas YOLO, tangani via ellipse-fit geometris terpisah (STRATEGI §4.2); ATAU **(b)** label hanya saat lingkaran KOMPAK di frame (jauh, tampak elips kecil), skip saat robot di dalamnya. Jangan generate ribuan dengan bbox center_circle salah.
- **Cek pangkal gawang**: verifikasi tak ada kotak junction (L/T/X) tepat di pangkal tiang gawang.

---

## RINGKASAN "robot digerakkan mana ke mana"
Bukan jalur. Teleport ke: **(1)** grid seluruh lapangan × semua heading × range kepala (coverage), PLUS **(2)** batch sengaja di dekat tiap gawang, pusat, tiap sudut, tiap sideline (jamin kelas langka), **(3)** di jarak near/mid/far. Lalu cek distribusi kelas, tambah untuk kelas lemah. Sudut kepala pan/tilt WAJIB ikut divariasikan (active vision).
