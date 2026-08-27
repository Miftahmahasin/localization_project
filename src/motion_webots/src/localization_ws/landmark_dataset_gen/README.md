# landmark_dataset_gen

Generator dataset landmark lapangan **berlabel-otomatis** untuk YOLO, digerakkan
oleh pose *ground-truth* Webots. Robot diposisikan **manual di editor Webots**;
tool memproyeksikan landmark lapangan yang sudah diketahui (junction **L/T/X**
dan **goalpost**) ke piksel kamera, menampilkan overlay label **live** untuk
verifikasi, lalu menyimpan pasangan gambar + label saat ditekan `s`.

## Kelas (class map)
`0=L`, `1=T`, `2=X`, `3=goalpost`, `4=center_circle` (lihat `config/classes.txt`
/ `data.yaml` yang juga ditulis ke folder output).

## Geometri lapangan (Webots "kid", RoboCup 2021)
Landmark dibangun dari dimensi **field yang benar-benar dirender** (bukan nilai
sederhana di kode AMCL): field 9×6, **goal area 1×3** (sudut depan ±3.5/±1.5),
**penalty area 2×5** (sudut depan ±2.5/±2.5), penalty mark 1.5 m (±3.0), goal
2.6 m lebar × **1.5 m tinggi** (tiang ±1.3; tinggi render ≈1.5 m, bukan 1.25 m
teks proto), lingkaran tengah r=0.75. Total: **25 junction (12 L, 10 T, 3 X) +
4 goalpost + 1 center-circle.**

**X = cross mark**: center mark (0,0) + 2 penalty mark (±3.0,0), semuanya kelas X.

**Deteksi jauh**: `max_range` 9 m; kotak junction/mark yang jauh di-*pad* ke min
~12 px (`min_emit_px`) agar tetap terlabel (bukan dibuang karena terlalu tipis).

## Prasyarat
Webots + `op3_extern_controller` **sudah berjalan** (mem-publish
`/robotis_op3/camera/image_raw`, `/robotis_op3/camera/camera_info`,
`/ground_truth/odom`, `/robotis_op3/joint_states`).

## Menjalankan
```bash
source install/setup.bash
ros2 launch landmark_dataset_gen landmark_capture.launch.py
# atau override:
ros2 launch landmark_dataset_gen landmark_capture.launch.py \
     output_dir:=/data/op3_landmarks pitch_bias_deg:=-2.0 max_range_m:=5.0
```

## Tombol (di jendela OpenCV)
| Tombol | Fungsi |
|--------|--------|
| `s` | simpan frame (image + label YOLO + overlay debug) |
| `q` / ESC | keluar |
| `a` / `d` | head pan + / − (jika head control aktif) |
| `w` / `x` | head tilt + / − |
| `[` / `]` | kalibrasi `pitch_bias` − / + (derajat) |
| `;` / `'` | kalibrasi `base_z_offset` − / + (meter) |
| `r` | reset kalibrasi ke 0 |

## Kalibrasi (gerbang verifikasi)
Pose kamera dibangun dari rantai URDF eksak + pose ground-truth. Offset kecil
`pitch_bias_deg` dan `base_z_offset` menyerap selisih residual origin
Webots-vs-URDF. Setel sekali secara live (tombol di atas) sampai kotak jatuh
tepat di garis cat, lalu pindahkan nilai final ke argumen launch.

## Output
```
<output_dir>/images/NNNNNN.png
<output_dir>/labels/NNNNNN.txt      # class_id xc yc w h  (ternormalisasi 0..1)
<output_dir>/debug/NNNNNN.png       # overlay
<output_dir>/classes.txt  data.yaml
```
Index melanjutkan otomatis dari file yang sudah ada (aman untuk beberapa sesi).

## Arsitektur
- `field_landmarks.py` — daftar otoritatif: 25 junction (12 L, 10 T, 3 X) +
  4 goalpost + 1 center-circle, frame pusat-lapangan; dimensi = field Webots
  "kid" asli (diverifikasi terhadap proto cyberbotics R2025a + ukur render).
- `projection.py` — world→pixel via rantai URDF kepala + K dari `camera_info`,
  cek visibilitas (depan kamera / dalam frame / dalam jangkauan), konstruksi bbox
  (junction: stub garis 5 cm di tanah; goalpost: prisma tiang base→top).
- `landmark_dataset_capture.py` — node ROS2 + GUI overlay live + simpan.

## Guard gawang
Junction HANYA dari daftar marking garis; goalpost kelas terpisah dari geometri
tiang. Di titik (±4.5, ±1.3) yang berimpit, junction sah **dan** goalpost disimpan
sebagai dua kelas berbeda — tidak pernah men-junction-kan pangkal tiang yang
bukan perpotongan garis.
