from ultralytics import YOLO

model = YOLO("runs/detect/yolo_custom/weights/best.pt")
metrics = model.val(data="dataset/data.yaml")

print(metrics)
