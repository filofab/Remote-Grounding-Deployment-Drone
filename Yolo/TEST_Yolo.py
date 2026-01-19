##python
import os
import sys
import argparse
import threading
import time
import cv2

# Provo a importare l'SDK; se presente lo userò, altrimenti userò il fallback
try:
    from inference import InferencePipeline  # type: ignore
    HAS_INFERENCE_SDK = True
except Exception:
    InferencePipeline = None
    HAS_INFERENCE_SDK = False

class LocalInferencePipeline:
    """
    Fallback minimale che apre un video (file / device id / rtsp) e chiama la callback
    on_prediction(result, frame) per ogni frame.
    Interfaccia compatibile con init_with_workflow, start, join, stop.
    """
    def __init__(self, video_reference=0, max_fps=30, on_prediction=None, conf=0.4):
        self.video_reference = video_reference
        self.max_fps = max_fps or 30
        self.on_prediction = on_prediction
        self._stop = threading.Event()
        self._thread = None
        self.conf = conf

    @staticmethod
    def init_with_workflow(api_key=None, workspace_name=None, workflow_id=None,
                           video_reference=0, max_fps=30, on_prediction=None, conf=0.4):
        return LocalInferencePipeline(video_reference=video_reference,
                                      max_fps=max_fps,
                                      on_prediction=on_prediction,
                                      conf=conf)

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def join(self):
        if self._thread:
            self._thread.join()

    def stop(self):
        self._stop.set()

    def _run(self):
        # apertura VideoCapture (accetta sia int che path)
        try:
            cap = cv2.VideoCapture(int(self.video_reference))
        except Exception:
            cap = cv2.VideoCapture(self.video_reference)

        if not cap.isOpened():
            print(f"[WARN] Impossibile aprire video '{self.video_reference}' - terminating fallback pipeline")
            return

        min_frame_time = 1.0 / float(self.max_fps)
        while not self._stop.is_set():
            t0 = time.time()
            ret, frame = cap.read()
            if not ret:
                break

            # risultato fittizio: ritorniamo solo l'immagine e nessuna prediction
            result = {
                "output_image": type("IW", (), {"numpy_image": frame.copy()})(),
                "predictions": []
            }

            if callable(self.on_prediction):
                try:
                    self.on_prediction(result, frame)
                except Exception as e:
                    print(f"[WARN] callback on_prediction ha sollevato: {e}")

            elapsed = time.time() - t0
            to_sleep = max(0.0, min_frame_time - elapsed)
            if to_sleep > 0:
                time.sleep(to_sleep)

        cap.release()

def my_sink(result, video_frame):
    if result.get("output_image"):
        cv2.imshow("Workflow Image", result["output_image"].numpy_image)
        cv2.waitKey(1)
    print(result.get("predictions", result))

def build_pipeline(args):
    # Se l'utente vuole usare il workflow remoto e l'SDK è disponibile, provo ad inizializzare.
    if args.use_workflow and HAS_INFERENCE_SDK:
        try:
            api_key = args.api_key or os.environ.get("INFERENCE_API_KEY")
            # la chiamata può sollevare eccezioni di parsing (es. WorkflowSyntaxError)
            return InferencePipeline.init_with_workflow(
                api_key=api_key,
                workspace_name=args.workspace,
                workflow_id=args.workflow,
                video_reference=args.video,
                max_fps=args.max_fps,
                on_prediction=my_sink
            )
        except Exception as e:
            print(f"[ERROR] init_with_workflow fallita: {e}\n[RIPIEGO] usando pipeline locale.")
            # cadere al fallback
    # fallback locale
    return LocalInferencePipeline.init_with_workflow(
        video_reference=args.video,
        max_fps=args.max_fps,
        on_prediction=my_sink,
        conf=args.conf
    )

def main():
    parser = argparse.ArgumentParser(description="Yolo_V2: usa InferencePipeline o fallback locale")
    parser.add_argument("--use-workflow", action="store_true", help="Tentare init_with_workflow dell'SDK `inference`")
    parser.add_argument("--api-key", help="API key (o usa variabile d'ambiente INFERENCE_API_KEY)")
    parser.add_argument("--workspace", default="yolo-ytzsf", help="workspace name (quando si usa workflow)")
    parser.add_argument("--workflow", default="find-people", help="workflow id (quando si usa workflow)")
    parser.add_argument("--video", default=0, help="video reference: device id, path MP4 o RTSP")
    parser.add_argument("--max-fps", type=int, default=30, help="massimo FPS di elaborazione")
    parser.add_argument("--conf", type=float, default=0.4, help="soglia confidenza (solo per fallback/compatibilità)")
    args = parser.parse_args()

    pipeline = build_pipeline(args)

    try:
        pipeline.start()
        pipeline.join()
    except KeyboardInterrupt:
        print("[INFO] Interruzione da tastiera, arresto pipeline...")
        try:
            pipeline.stop()
        except Exception:
            pass
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
