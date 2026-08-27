# Latih YOLOv8n Landmark di Cloud GPU (Colab / Kaggle)

Dataset dilatih di cloud karena dev box CPU-only. Alur: **repack JPG → upload →
latih → export OpenVINO di cloud → unduh `best.pt` + IR**. TensorRT dibuat di Orin
(lihat `export_tensorrt_orin.sh`).

## 0. Repack (di workstation, sekali)
```bash
python scripts/package_for_cloud.py \
    --src /media/miftah/backup/landmark_dataset \
    --out /media/miftah/backup/landmark_yolo_cloud --quality 90 --tar
# -> landmark_yolo_cloud.tar.gz  (~10 GB @1080p q90)  berisi train/ val/ landmark.yaml
```
Upload `.tar.gz` ke Google Drive (Colab) atau sebagai Kaggle Dataset.
**Untuk tarball 10 GB, Kaggle lebih disarankan** daripada Colab: dataset persisten
(tak perlu re-upload tiap sesi), quota GPU 12 jam lebih stabil, dan ekstraksi lebih
cepat daripada mount Drive. Drive gratis 15 GB muat 10 GB tapi mepet.

## 1. Colab (GPU: Runtime → Change runtime type → T4)
```python
!pip -q install ultralytics
from google.colab import drive; drive.mount('/content/drive')
!tar xzf "/content/drive/MyDrive/landmark_yolo_cloud.tar.gz" -C /content/
%cd /content/landmark_yolo_cloud

# LATIH (imgsz 640; kelas L/T dominan by design → pantau mAP per-kelas)
# cache=disk (BUKAN ram): 9200 gambar bisa >12 GB di RAM Colab -> OOM. disk aman.
!yolo detect train model=yolov8n.pt data=landmark.yaml \
     imgsz=640 epochs=150 patience=30 batch=-1 cache=disk \
     close_mosaic=10 seed=0 project=runs name=landmark_v8n

# CEK akurasi + per-kelas (center_circle/X paling langka)
!yolo detect val model=runs/landmark_v8n/weights/best.pt data=landmark.yaml imgsz=640

# EXPORT OpenVINO INT8 (kalibrasi pakai val kita) — jalan di CPU Colab pun bisa
!yolo export model=runs/landmark_v8n/weights/best.pt format=openvino \
     int8=True data=landmark.yaml imgsz=640
!yolo export model=runs/landmark_v8n/weights/best.pt format=onnx \
     opset=12 simplify=True dynamic=False imgsz=640

# simpan hasil ke Drive
!cp -r runs/landmark_v8n/weights/best.pt runs/landmark_v8n/weights/best.onnx \
       runs/landmark_v8n/weights/best_int8_openvino_model \
       "/content/drive/MyDrive/"
```

## 2. Kaggle (alternatif, GPU T4 x2)
- Add dataset (tarball) → Notebook Settings: Accelerator = GPU.
- `!tar xzf /kaggle/input/<ds>/landmark_yolo_cloud.tar.gz -C /kaggle/working/`
- perintah `yolo detect train ...` sama; output di `/kaggle/working/runs`.

## 3. Bawa turun
- `best.pt`  → sumber untuk semua export (juga TensorRT di Orin).
- `best.onnx` → perantara portable.
- `best_int8_openvino_model/` → langsung dipakai NUC (`detect_openvino.launch.py`).

## Tuning bila perlu (opsional)
- Recall center_circle/X rendah → `epochs=200` atau `cls=1.0` (naikkan bobot loss
  kelas); JANGAN buang label L/T (menciptakan false-negative).
- Overfit ke look sim → tambah beberapa gambar REAL berlabel lalu fine-tune singkat
  (`model=best.pt epochs=30 lr0=0.001`).
- Target FPS ketat → setelah training, benchmark 512/416 (lihat `benchmark.py` +
  `latency_probe.py`) dan pilih imgsz terkecil dengan drop mAP dalam ambang.
