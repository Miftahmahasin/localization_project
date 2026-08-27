# Konfigurasi Webots — `robotis_op3_extern.wbt`

Catatan agar mudah **kembali ke konfigurasi awal** (untuk uji lokalisasi) atau
tetap di **konfigurasi kaku** (untuk pengumpulan dataset landmark).

---

## ‼️ STATUS AKTIF (2026-08-22): dikembalikan ke FISIS BASELINE untuk TAHAP 8

World sekarang = **fisis baseline (A)** — `contactProperties` & `defaultDamping`
DIHAPUS, spawn z = `0.3` — tapi **kamera TETAP 1920×1080** (+ antiAliasing/noise0/
motionBlur0 di proto, + lampu DR). Catatan lama yang bilang "A = 1280×720" **USANG
& JANGAN dipakai**: seluruh lokalisasi landmark dikalibrasi ke 1920×1080 (`fx=1185.6`);
menurunkan resolusi akan merusak proyeksi. Jadi resolusi kamera **1920×1080 di KEDUA
mode** sekarang; yang membedakan eval vs dataset hanyalah **blok fisis + spawn z**
(dan `settle_extra_s` di sampler).

**Config dataset (B) DISIMPAN byte-for-byte** di:
`worlds/robotis_op3_extern.DATASET.wbt`.

### Restore ke DATASET (saat mau collect data lagi) — dari `~/basbot`:
```bash
W=src/motion_webots/src/ROBOTIS-OP3-Simulations/op3_webots_ros2/worlds
cp "$W/robotis_op3_extern.DATASET.wbt" "$W/robotis_op3_extern.wbt"
cp "$W/robotis_op3_extern.wbt" install/op3_webots_ros2/share/op3_webots_ros2/worlds/robotis_op3_extern.wbt
# lalu di sampler: settle_extra_s = 2.5 ; Webots: File -> Reload World
```
### Kembali ke EVAL/BASELINE (fisis A) — dari `~/basbot`:
```bash
W=src/motion_webots/src/ROBOTIS-OP3-Simulations/op3_webots_ros2/worlds
# hapus blok contactProperties + defaultDamping dari WorldInfo, spawn z -> 0.3,
# JANGAN sentuh cameraWidth/Height (biarkan 1920/1080), lalu:
cp "$W/robotis_op3_extern.wbt" install/op3_webots_ros2/share/op3_webots_ros2/worlds/robotis_op3_extern.wbt
# sampler (jika relevan): settle_extra_s = 0.35 ; Webots: File -> Reload World
```
> Perbedaan eval↔dataset kini HANYA: fisis (contact/damping), spawn z (0.3↔0.255),
> `settle_extra_s` (0.35↔2.5). Kamera & DR sama di keduanya.

---

> Setelah mengedit world di `src/`, **sinkronkan** ke install tree lalu **reload
> world di Webots** (File → Reload World). File world **tidak** dilacak git, jadi
> catatan ini adalah satu-satunya sumber pemulihan.
>
> Sinkron cepat (jalankan dari `~/basbot`):
> ```bash
> SRC=src/motion_webots/src/ROBOTIS-OP3-Simulations/op3_webots_ros2/worlds/robotis_op3_extern.wbt
> cp "$SRC" install/op3_webots_ros2/share/op3_webots_ros2/worlds/robotis_op3_extern.wbt
> cp "$SRC" src/motion_webots/install/op3_webots_ros2/share/op3_webots_ros2/worlds/robotis_op3_extern.wbt
> ```

---

## A. Konfigurasi AWAL (baseline — untuk uji LOKALISASI)

Gravitasi default, tanpa contact/damping tuning, spawn robot di z=0.3.

**`WorldInfo`** — hanya seperti ini (TANPA `gravity`, `contactProperties`,
`defaultDamping`):
```
WorldInfo {
  info [
    "ROBOTIS OP3 robot."
    "The ROBOTIS OP3 robot simulation model can be programmed using the ROBOTIS OP3 motions files."
  ]
  title "ROBOTIS OP3"
  basicTimeStep 8
}
```

**`RobotisOp3` translation:**
```
  translation -0.36 0 0.3
```

**Sampler timing (jika dipakai):** `settle_extra_s` = `0.35`
(di `landmark_dataset_gen/landmark_dataset_sampler.py` baris ~194 dan
`launch/landmark_sampler.launch.py`).

---

## B. Konfigurasi KAKU (aktif sekarang — untuk PENGUMPULAN DATASET)

Gravitasi normal tetap ada, tapi kontak tanpa pantul + damping tinggi supaya
robot tak bergoyang tiap teleport. Spawn diturunkan ke tinggi settel.

**`WorldInfo`:**
```
WorldInfo {
  info [
    "ROBOTIS OP3 robot."
    "The ROBOTIS OP3 robot simulation model can be programmed using the ROBOTIS OP3 motions files."
  ]
  title "ROBOTIS OP3"
  basicTimeStep 8
  contactProperties [
    ContactProperties {
      coulombFriction [
        10
      ]
      bounce 0
      bounceVelocity 0
      softCFM 0.003
      softERP 0.9
    }
  ]
  defaultDamping Damping {
    linear 0.95
    angular 0.95
  }
}
```

**`RobotisOp3` translation:**
```
  translation -0.36 0 0.255
```

**Sampler timing:** `settle_extra_s` = `2.5` (dwell ~2.5 detik agar robot benar-benar
diam sebelum tiap capture).

---

## Ringkasan perbedaan A → B

| Item | A (awal / lokalisasi) | B (kaku / dataset) |
|---|---|---|
| `gravity` | default `0 0 -9.81` (tak ada baris) | default `0 0 -9.81` (tak ada baris) |
| `contactProperties` | tidak ada | `coulombFriction 10`, `bounce 0`, `bounceVelocity 0`, `softCFM 0.003`, `softERP 0.9` |
| `defaultDamping` | tidak ada | `linear 0.95`, `angular 0.95` |
| spawn z | `0.3` | `0.255` |
| `settle_extra_s` | `0.35` | `2.5` |
| camera resolusi | `1280 x 720` | `1920 x 1080` |
| camera antiAliasing | (tidak diset = FALSE) | `TRUE` + `noise 0`, `motionBlur 0` (di `../protos/RobotisOp3.proto`) |
| `post_capture_s` | (belum ada) | `0.75` |
| `min_emit_px` | `12` | `18` (skala 1.5x mengikuti resolusi) |

Untuk **kembali ke A**: hapus blok `contactProperties` + `defaultDamping` dari
`WorldInfo`, ubah z spawn ke `0.3`, kembalikan `settle_extra_s` ke `0.35`, lalu
sinkron + reload.

---

## Resolusi kamera & `imgsz` — jangan tertukar (TAHAP A1.2)

Dua hal BERBEDA, sering salah dibaca:

- **Resolusi tangkap** (`cameraWidth/cameraHeight` di world) = resolusi citra yang
  diproyeksikan. Kalibrasi lokalisasi (`fx=1185.6, cx=960, cy=540`) menskala **linear**
  dengan resolusi. Kalau resolusi berubah, **K harus ikut** — `camera_info_publisher`
  kini **menurunkan K dari FOV (1.3613 rad) + (W,H)** (bukan lagi hardcoded), jadi aman.
  Test CI `landmark_geometry/test/test_resolution_invariance.py` menjaga invarian ini.
- **`imgsz` YOLO** = ukuran **letterbox saat inferensi** saja. Ultralytics men-scale
  internal lalu mengembalikan box di **koordinat citra asli (full-res)**
  (`infer_node.py`, tervalidasi). ⇒ `imgsz 320` vs `640` **TIDAK** mengubah resolusi
  proyeksi; proyeksi selalu jalan di resolusi tangkap penuh. Jadi "kamera 1080p" dan
  "imgsz 320" tidak bertentangan.

Konsekuensi deploy: yang perlu benar di hardware adalah **CameraInfo yang cocok
resolusi tangkap** (dari kalibrasi `camera.yaml` atau derivasi FOV), bukan `imgsz`.
