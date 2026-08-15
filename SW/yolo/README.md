# YOLO

## 드론 카메라 영상 재학습

`prepare_camera_dataset.py`는 `SW/camera_test` 영상을 2 FPS로 샘플링하고,
원형 Gaussian red score로 초벌 라벨을 만든다. 전체 프레임 외에 192px proposal
ROI와 풍선 반대편 hard-negative crop도 함께 생성한다.

```bash
python3 prepare_camera_dataset.py \
  --videos ../camera_test \
  --output /tmp/arms_camera_dataset \
  --sample-fps 2

python3 train_camera.py \
  --data /tmp/arms_camera_dataset/data.yaml \
  --model yolo11n.pt \
  --epochs 15 --imgsz 320 --batch 16 \
  --project /tmp/arms_yolo_runs
```

학습 결과 `weights/best.pt`를
`arms_ws/src/arms_detection/docker/models/balloon_camera.pt`로 복사한다.
Jetson에서는 먼저 `.pt`로 검증하고, 성능이 필요하면 아래 TensorRT 변환을 한다.

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
CAMERA = 0     # 카메라 인덱스
CONF   = 0.32  # 재학습 모델 validation F1 최적 threshold
IOU    = 0.45 # NMS IoU threshold
```

## export_trt.py — TensorRT 엔진 변환

`balloon_camera.pt` → FP16 양자화 → `balloon_camera.engine` 변환
(ultralytics `format=engine`).
ultralytics 로 export 하면 엔진에 메타데이터(class names / imgsz / task)가 함께
기록되어 추론 노드의 `YOLO('...engine')` 가 그대로 로드할 수 있다.
변환된 엔진은 Jetson Docker 컨테이너에서 고속 추론에 사용됨.

### 실행 (반드시 추론 컨테이너와 같은 base 이미지 안에서)

`.engine` 은 GPU 아키텍처뿐 아니라 **TensorRT 버전**에도 종속된다. 호스트의
TensorRT(예: 10.3)와 컨테이너의 TensorRT(예: 10.7)가 다르면 엔진이 로드되지
않으므로, 추론이 돌아가는 것과 동일한 ultralytics jetson 이미지 안에서 빌드한다.

```bash
cd SW/yolo
MODELS=../arms_ws/src/arms_detection/docker/models
docker run --rm --runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all -w /tmp \
    -v "$PWD/$MODELS:/models" \
    -v "$PWD/export_trt.py:/tmp/export_trt.py:ro" \
    ultralytics/ultralytics:latest-jetson-jetpack6 \
    python3 /tmp/export_trt.py
```

완료 후 `models/balloon_camera.engine` 이 생성됨. 이후 compose 의 `ARMS_MODEL` 을
`/models/balloon_camera.engine` 로 두면 노드가 엔진으로 추론한다.

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
