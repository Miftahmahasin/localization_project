import cv2

# Gunakan index kamera (video3 = index 3)
cap = cv2.VideoCapture(3)

# Cek apakah kamera berhasil dibuka
if not cap.isOpened():
    print("Error: Kamera tidak bisa dibuka")
    exit()

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Tidak bisa membaca frame")
        break

    # Tampilkan frame
    cv2.imshow("Camera /dev/video3", frame)

    # Tekan 'q' untuk keluar
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resource
cap.release()
cv2.destroyAllWindows()