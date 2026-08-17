"""
balloon_camera.pt → TensorRT FP16 엔진 변환 (ultralytics 방식)

ultralytics 의 export 를 쓰면 엔진 파일에 메타데이터(class names / imgsz / task)가
함께 기록되어, 추론 노드의 `YOLO('...engine')` 가 그대로 로드할 수 있다.
(raw TensorRT 로 만든 엔진은 이 메타데이터가 없어 ultralytics 가 로드하지 못한다.)

입력 : <models>/balloon_camera.pt
출력 : <models>/balloon_camera.engine   (FP16 양자화)

주의:
  * .engine 은 빌드한 GPU 아키텍처 + TensorRT 버전에 종속된다.
    반드시 실기체(Jetson)의 ultralytics 컨테이너 안에서 실행할 것.
    호스트에는 torch/ultralytics 가 없을 수 있다.
  * 추론 컨테이너와 동일한 base 이미지(ultralytics jetson jetpack6)에서 만들면
    TensorRT 버전이 일치해 호환된다.

컨테이너에서 실행 예 (models 디렉토리를 /models 로 마운트):
    # 스크립트는 반드시 /tmp 등에 마운트할 것. 루트(/)에 두면 컨테이너의
    # /ultralytics 디렉토리가 패키지를 가려 import 가 깨진다.
    docker run --rm --runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all -w /tmp \\
        -v "$PWD/../arms_ws/src/arms_detection/docker/models:/models" \\
        -v "$PWD/export_trt.py:/tmp/export_trt.py:ro" \\
        ultralytics/ultralytics:latest-jetson-jetpack6 \\
        python3 /tmp/export_trt.py

또는 ultralytics CLI 한 줄로도 동일하다:
    yolo export model=/models/balloon_camera.pt format=engine half=True device=0 imgsz=320
"""

import os

# Jetson(통합 메모리) 워크어라운드: PyTorch 2.x 의 expandable_segments 가
# Tegra NvMap 에서 큰 가상영역 예약에 실패해 export 중 "NVML_SUCCESS == r" /
# NvMap error 12 로 죽는다. torch import 전에 꺼야 효과가 있으므로 최상단에서 설정.
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:False")

from pathlib import Path

from ultralytics import YOLO

# 컨테이너에서는 /models/balloon_camera.pt, 호스트에서는 repo 모델을 기본값으로.
_default = Path(__file__).parent / "../arms_ws/src/arms_detection/docker/models/balloon_camera.pt"
MODEL_PATH = os.environ.get("ARMS_PT", "/models/balloon_camera.pt")
IMGSZ = int(os.environ.get("ARMS_IMGSZ", "320"))
if not Path(MODEL_PATH).exists():
    MODEL_PATH = str(_default.resolve())

def main():
    print(f"[INFO] 모델 로드: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)

    print("[INFO] TensorRT 엔진 빌드 시작 (FP16 half=True 고정) — 수 분 소요...")
    engine_path = model.export(format="engine", half=True, device=0, imgsz=IMGSZ)

    print(f"[INFO] 저장 완료: {engine_path}")

    # ultralytics 는 PT→ONNX→TensorRT 순으로 빌드하며 중간 .onnx 를 남긴다.
    # 엔진만 있으면 되므로 중간 산출물은 지운다(추론에 불필요, 용량만 차지).
    onnx_path = Path(engine_path).with_suffix(".onnx")
    if onnx_path.exists():
        onnx_path.unlink()
        print(f"[INFO] 중간 ONNX 삭제: {onnx_path}")


if __name__ == "__main__":
    main()
