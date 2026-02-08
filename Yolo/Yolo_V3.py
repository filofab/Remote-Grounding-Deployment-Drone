# python
## Yolo V3: aggiunta estetica di spazzi quando la detection è falsa per migliore lettura della console

import time
import cv2
from ultralytics import YOLO

# ---------------- CONFIG ----------------
VIDEO_PATH = "Video/_Video_clip.mp4"
MODEL_PATH = "modelli/colab_trained.pt"                 #modello addestrato
CONF_TH = 0.7                                   # soglia confidenza
PERSON_CLASS_ID = 0

PREVIEW_WIDTH = 640*2             # preview ridotta
PREVIEW_HEIGHT = 360*2

# ---------------- UTILITY ----------------
def draw_text_bottom_center(img, text):
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 2
    thickness = 3
    padding = 20

    h, w, _ = img.shape
    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)

    cx = w // 2
    cy = h // 2

    x1 = cx - tw // 2 - padding
    y1 = cy - th // 2 - padding
    x2 = cx + tw // 2 + padding
    y2 = cy + th // 2 + padding

    # sfondo nero
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), -1)

    # testo bianco centrato
    cv2.putText(
        img,
        text,
        (cx - tw // 2, cy + th // 2),
        font,
        scale,
        (0, 0, 0),
        thickness,
        cv2.LINE_AA
    )

# ---------------- MAIN ----------------
def main():
    model = YOLO(MODEL_PATH)
    cap = cv2.VideoCapture(VIDEO_PATH)

    if not cap.isOpened():
        raise RuntimeError("Errore apertura video")

    persona_detected = False
    last_no_person_print = time.time() - 1.0  # permette il primo print immediato se non rilevata persona

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Reset flag ogni frame
        persona_detected = False

        results = model(frame, conf=CONF_TH, verbose=False)

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])

                if cls == PERSON_CLASS_ID:
                    persona_detected = True
                    print("Persona rilevata")

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # Overlay solo se detection attiva
        if persona_detected:
            draw_text_bottom_center(frame, "PERSONA RILEVATA")
            # reset del timer per evitare stampa immediata appena la persona scompare
            last_no_person_print = time.time()
        else:
            # se non c'è persona, stampa uno spazio ogni secondo
            if time.time() - last_no_person_print >= 1.0:
                print(" ")
                last_no_person_print = time.time()

        # Riduzione preview
        frame_small = cv2.resize(
            frame,
            (PREVIEW_WIDTH, PREVIEW_HEIGHT),
            interpolation=cv2.INTER_AREA
        )

        cv2.imshow("Preview Detection", frame_small)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
