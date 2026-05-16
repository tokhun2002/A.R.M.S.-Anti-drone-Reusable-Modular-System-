# YOLO

## 모델 다운로드

- https://drive.google.com/file/d/1ZD6JaYdnwULNY5RCRy2HP1EUoAv17KHk/view?usp=drive_link

## test_camera.py — YOLO 모델 테스트

카메라 영상을 받아 ONNX 모델로 실시간 풍선 탐지.

### 의존성 설치

```bash
# 가상환경 사용을 권장
pip install ultralytics opencv-python onnx onnxruntime
```

### 실행

```bash
python test_camera.py
```

`test_camera.py` 상에서 카메라 인덱스나 파라미터를 수정할 수 있음.

```python
CAMERA = 0    # 카메라 인덱스
CONF   = 0.5  # confidence threshold
IOU    = 0.45 # NMS IoU threshold
```

## export_trt.py — TensorRT 엔진 변환

`best.onnx` → FP16 양자화 → `best_fp16.engine` 변환.  
변환된 엔진은 Jetson Docker 컨테이너에서 고속 추론에 사용됨.

### 실행

```bash
python export_trt.py
```

완료 후 `models/best_fp16.engine` 이 생성됨.

### 주의사항

- **반드시 Jetson에서 실행**: `.engine` 파일은 빌드한 GPU 아키텍처(SM 버전)에 종속됨. 노트북/x86에서 만든 엔진은 Jetson에서 동작하지 않음.
- **빌드 시간**: Jetson Orin 기준 약 3~10분 소요.
- `tensorrt` 패키지는 Jetson JetPack 환경에 기본 포함되어 있음. 별도 설치 불필요.
