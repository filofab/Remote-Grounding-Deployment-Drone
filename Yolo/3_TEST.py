from ultralytics import YOLO

model = YOLO("best.pt")
model.predict(
    source="test.jpg",
    conf=0.4,
    show=True
)
