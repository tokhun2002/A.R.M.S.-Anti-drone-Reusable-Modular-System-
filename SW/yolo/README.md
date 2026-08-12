# YOLO

## 모델 다운로드

- https://drive.google.com/file/d/1ZD6JaYdnwULNY5RCRy2HP1EUoAv17KHk/view?usp=drive_link

## test_camera.py — YOLO 모델 테스트

카메라 영상을 받아 YOLO 모델로 실시간 풍선 탐지.

### 의존성 설치

```bash
# 가상환경 사용을 권장
pip install ultralytics opencv-python
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

`best.pt` → FP16 양자화 → `best.engine` 변환 (ultralytics `format=engine`).
ultralytics 로 export 하면 엔진에 메타데이터(class names / imgsz / task)가 함께
기록되어 추론 노드의 `YOLO('...engine')` 가 그대로 로드할 수 있다.
변환된 엔진은 Jetson Docker 컨테이너에서 고속 추론에 사용됨.

### 실행 (반드시 추론 컨테이너와 같은 base 이미지 안에서)

`.engine` 은 GPU 아키텍처뿐 아니라 **TensorRT 버전**에도 종속된다. 호스트의
TensorRT(예: 10.3)와 컨테이너의 TensorRT(예: 10.7)가 다르면 엔진이 로드되지
않으므로, 추론이 돌아가는 것과 동일한 ultralytics jetson 이미지 안에서 빌드한다.

```bash
cd SW/YOLO_balloon
MODELS=../arms_ws/src/arms_detection/docker/models
docker run --rm --runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all -w /tmp \
    -v "$PWD/$MODELS:/models" \
    -v "$PWD/export_trt.py:/tmp/export_trt.py:ro" \
    ultralytics/ultralytics:latest-jetson-jetpack6 \
    python3 /tmp/export_trt.py
```

완료 후 `models/best.engine` 이 생성됨. 이후 compose 의 `ARMS_MODEL` 을
`/models/best.engine` 로 두면 노드가 엔진으로 추론한다.

### 주의사항

- **반드시 Jetson의 컨테이너 안에서 실행**: 위 참고. 호스트엔 torch/ultralytics 가
  없을 수 있고, TensorRT 버전도 컨테이너와 다를 수 있다.
- **메모리**: TRT 엔진 빌드는 GPU 여유 메모리가 최소 2~3GB 필요하다. Jetson 은
  GPU·시스템 메모리를 공유(예: 8GB)하므로, **데스크톱/Gazebo/브라우저 등을 닫아**
  메모리를 확보한 뒤 실행할 것. 여유가 부족하면 TRT 가 tactic 을 건너뛰며
  `Cuda Runtime (out of memory)` 로 실패한다. (재부팅 직후가 가장 안전)
- **Jetson 워크어라운드**: PyTorch 2.x 의 `expandable_segments` 가 Tegra NvMap 에서
  실패하므로 스크립트가 최상단에서 `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:False`
  를 설정한다. (`NVML_SUCCESS == r` / `NvMap error 12` 크래시 방지)
- **빌드 시간**: Jetson Orin 기준 약 3~10분 소요.
