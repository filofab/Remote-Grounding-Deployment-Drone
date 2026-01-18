# python
import argparse
import os
import sys
import cv2
from time import time

try:
    from ultralytics import YOLO
except Exception:
    print("Errore: pacchetto `ultralytics` non trovato. Installa con `pip install ultralytics`.")
    sys.exit(1)


def process_video(input_path, output_path, model_path="yolov8n.pt", conf=0.4, show=False):
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"File non trovato: {input_path}")

    model = YOLO(model_path)

    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        raise RuntimeError("Impossibile aprire il file video")

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    print(f"[INFO] Elaborazione {input_path} -> {output_path}")
    start = time()
    frame_idx = 0
    saved = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame, conf=conf, classes=[0], verbose=False)

        for r in results:
            if getattr(r, "boxes", None) is None:
                continue
            for box in r.boxes:
                try:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                except Exception:
                    coords = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = map(int, coords)
                try:
                    conf_score = float(box.conf[0])
                except Exception:
                    conf_score = 0.0

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"PERSON {conf_score:.2f}"
                cv2.putText(frame, label, (x1, max(15, y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                saved += 1

        out.write(frame)
        frame_idx += 1

        if show:
            cv2.imshow("Person Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    out.release()
    if show:
        cv2.destroyAllWindows()

    elapsed = time() - start
    print(f"[INFO] Finito. Frame elaborati: {frame_idx}, box disegnati (approx): {saved}, tempo: {elapsed:.2f}s")


def build_default_output(input_path):
    base, _ = os.path.splitext(os.path.basename(input_path))
    return os.path.join(os.path.dirname(input_path) or ".", f"{base}_boxed.mp4")


# python
def main():
    parser = argparse.ArgumentParser(description="Aggiunge bounding box persone a un MP4 usando YOLOv8")
    parser.add_argument("input", nargs="?", default=None, help="Percorso file video di input (MP4)")
    parser.add_argument("output", nargs="?", help="Percorso file video di output (MP4)")
    parser.add_argument("--model", default="yolov8n.pt", help="Percorso modello YOLOv8")
    parser.add_argument("--conf", type=float, default=0.4, help="Soglia confidenza")
    parser.add_argument("--show", action="store_true", help="Mostra anteprima durante l'elaborazione")
    args = parser.parse_args()

    input_path = args.input
    if not input_path:
        try:
            input_path = input("Inserisci il percorso del file video: ").strip('\"\'')
        except EOFError:
            print("Nessun file di input fornito. Esegui lo script con un percorso o configura i parametri di run.")
            sys.exit(2)

    output_path = args.output or build_default_output(input_path)

    try:
        process_video(input_path, output_path, model_path=args.model, conf=args.conf, show=args.show)
    except Exception as e:
        print(f"Errore: {e}")
        sys.exit(1)



if __name__ == "__main__":
    main()