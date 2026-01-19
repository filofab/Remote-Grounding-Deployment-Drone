import cv2
import time
from ultralytics import YOLO

# ==========================
# INPUT UTENTE
# ==========================
video_path = input("Inserisci il percorso del file video: ").strip()
frame_step = int(input("Esegui inferenza ogni quanti frame? (es. 1, 5, 10): "))

# ==========================
# CONFIGURAZIONE
# ==========================
OUTPUT_PATH = "Detection.mp4"
MODEL_PATH = "yolov8n.pt"
CONF_THRESHOLD = 0.4
LOG_INTERVAL = 10.0  # secondi

# ==========================
# INIZIALIZZAZIONE YOLO
# ==========================
model = YOLO(MODEL_PATH)

# ==========================
# APERTURA VIDEO INPUT
# ==========================
cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    raise RuntimeError("Impossibile aprire il file video")

width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps    = cap.get(cv2.CAP_PROP_FPS)
if fps <= 0:
    fps = 25.0

# ==========================
# VIDEO WRITER
# ==========================
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
out = cv2.VideoWriter(OUTPUT_PATH, fourcc, fps, (width, height))

print("\n[INFO] Avvio detection")
print(f"[INFO] Input video      : {video_path}")
print(f"[INFO] Inferenza ogni   : {frame_step} frame")
print(f"[INFO] Output video     : {OUTPUT_PATH}\n")

# ==========================
# LOOP PRINCIPALE
# ==========================
frame_count = 0
processed_frames = 0
last_log_time = time.time()
last_results = None

while True:
    ret, frame = cap.read()
    if not ret:
        print("[INFO] Fine video")
        break

    frame_count += 1
    processed_frames += 1

    # Inference solo ogni N frame
    if frame_count % frame_step == 0:
        last_results = model(
            frame,
            conf=CONF_THRESHOLD,
            classes=[0],
            verbose=False
        )

    # Disegno bounding box usando ultimo risultato valido
    if last_results is not None:
        for r in last_results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"PERSON {conf:.2f}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

    out.write(frame)

    # Log ogni 10 secondi
    now = time.time()
    if now - last_log_time >= LOG_INTERVAL:
        print(f"[LOG] Frame elaborati: {processed_frames}")
        last_log_time = now

    cv2.imshow("YOLO Person Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("[INFO] Interruzione manuale")
        break

# ==========================
# CLEANUP
# ==========================
cap.release()
out.release()
cv2.destroyAllWindows()

print(f"\n[INFO] Totale frame elaborati: {processed_frames}")
print("[INFO] Video salvato come Detection.mp4")
