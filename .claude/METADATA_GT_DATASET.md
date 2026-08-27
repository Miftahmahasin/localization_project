# METADATA GROUND TRUTH — Dataset Landmark Webots

**Pendamping untuk**: `STRATEGI_PENGUMPULAN_DATA.md`, `STRATEGI_YOLO_LANDMARK.md`
**Tujuan**: mendefinisikan metadata GT yang disimpan bersama dataset, agar evaluasi lokalisasi & pengukuran data-association nanti jadi otomatis.

---

## 0. Kenapa simpan GT SEKARANG (dan di sini)
Generator sudah TAHU pose GT robot + pose kamera + koordinat dunia tiap landmark (ia yang men-teleport & memproyeksikan). Menyimpannya sekarang = nyaris gratis. Menambahkannya SETELAH generate ribuan = generate ulang. Waktu sempit → simpan SEKARANG, sebelum generate penuh.

## 1. Dua "data lokasi" yang BERBEDA — jangan campur
- **Label YOLO** (`class cx cy w h`, dinormalisasi): untuk MELATIH detektor. Harus tetap MURNI format YOLO.
- **Metadata GT** (file TERPISAH): untuk EVALUASI & DEBUG lokalisasi. Ini yang ditambahkan.

Mencampur keduanya → training YOLO rusak. Pisahkan file/struktur.

## 2. Skema metadata per-frame (sidecar JSON atau manifest JSONL)
Per gambar:
```
{
  "image": "frame_00047.png",
  "gt_robot_pose": {"x": -0.36, "y": 0.00, "yaw_deg": -0.1},   // koordinat lapangan, FRAME SAMA dengan lokalisasi
  "camera": {
    "head_pan_deg": 0.0, "head_tilt_deg": -10.0,
    "cam_world_pos": [x, y, z], "cam_world_quat": [w, x, y, z]
  },
  "landmarks": [
    {"class": "L", "world_xy": [x, y], "pixel_uv": [u, v], "bbox_norm": [cx, cy, w, h], "distance_m": d}
    // ... tiap landmark terlihat  ← INI association ground truth (box mana = landmark dunia mana)
  ],
  "domain_rand": {"light_seed": 12, "grass_variant": 3}         // opsional, untuk reproduksibilitas
}
```

## 3. Tiga guna di masa depan (kenapa sepadan)
1. **Evaluasi lokalisasi otomatis**: jalankan pipeline lokalisasi di frame ini → bandingkan pose hasil vs `gt_robot_pose` → error langsung, tanpa setup terpisah. Mempercepat Fase 3.
2. **Ukur mis-association rate**: `landmarks[]` mencatat box mana = landmark dunia mana → bisa ukur salah-cocok ("T yang mana") — bagian tersulit Fase 2. Tanpa disimpan sekarang, tak bisa diukur nanti.
3. **Debug geometri**: kalau lokalisasi meleset, cek apakah error di deteksi / proyeksi / association, karena punya GT antara (world + pixel tiap landmark).

## 4. Aturan konsistensi koordinat (KRITIS)
`gt_robot_pose` & `world_xy` landmark HARUS di frame koordinat yang SAMA PERSIS dengan kode lokalisasi. Kalau beda (pelajaran spawn-offset), perbandingan eval nanti tak bermakna. **Reuse** definisi frame dari kode lokalisasi, jangan re-derive.

## 5. Per-frame vs temporal ("hubungan frame")
- **Dataset ini = pose INDEPENDEN** (teleport acak) → TIDAK ada hubungan antar-frame. Cocok untuk: melatih detektor + **evaluasi lokalisasi PER-FRAME** (pose geometris dari ≥2 corner vs GT). Ini validasi near-term-mu (tanpa IMU) — dan cukup untuk membuktikan lokalisasi absolut bekerja.
- **Hubungan frame (temporal)** = set TERPISAH: trajektori berurutan dengan gerak antar-frame diketahui, untuk menguji tracking EKF + bridging antar-glimpse. Ini bagian yang DITUNDA (butuh bridging/IMU). JANGAN paksakan ke dataset teleport-independen ini. Catat sebagai capture-mode terpisah untuk Fase 3.

## Ringkasan
Simpan GT per-frame DI SINI (gratis; enabling eval + association-measure + debug), sebagai file TERPISAH dari label YOLO murni, di frame koordinat yang sama dengan lokalisasi. Yang paling berharga: association GT (box↔world). "Hubungan frame" temporal adalah set terpisah untuk nanti — jangan campur.
