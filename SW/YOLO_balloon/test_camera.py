import time
from pathlib import Path

import cv2
from ultralytics import YOLO

MODEL = Path(__file__).parent / "../arms_ws/src/arms_detection/docker/models/best.pt"
CAMERA = 0
CONF = 0.5
IOU = 0.45

print(f"[INFO] 모델 로드 중: {MODEL.resolve()}")
model = YOLO(str(MODEL), task="detect")
print(f"[INFO] 모델 로드 완료 | 클래스: {list(model.names.values())}")

cap = cv2.VideoCapture(CAMERA)
print(f"[INFO] 카메라 {CAMERA} | 해상도: {int(cap.get(3))}x{int(cap.get(4))} | 종료: q / ESC")
prev = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.predict(frame, conf=CONF, iou=IOU, verbose=False)

    fps = 1.0 / max(time.time() - prev, 1e-6)
    prev = time.time()

    for box in results[0].boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        conf = float(box.conf[0])
        label = f"balloon {conf:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.circle(frame, ((x1+x2)//2, (y1+y2)//2), 4, (0, 0, 255), -1)

    cv2.putText(frame, f"FPS {fps:.1f}", (12, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2)
    cv2.imshow("Balloon Detection", frame)

    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
        break

cap.release()
cv2.destroyAllWindows()
