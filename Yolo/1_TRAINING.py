from ultralytics import YOLO

def main():
    model = YOLO("yolov8n.pt")  # oppure yolov8s.pt, yolov8m.pt

    model.train(
        data="dataset/data.yaml",
        epochs=100,
        imgsz=640,
        batch=16,
        device=0,        # 0 = GPU, 'cpu' se non hai CUDA
        workers=8,
        name="yolo_custom"
    )

if __name__ == "__main__":
    main()
