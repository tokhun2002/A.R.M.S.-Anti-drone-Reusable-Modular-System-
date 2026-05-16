"""
ONNX → TensorRT FP16 변환 스크립트

입력 : SW/arms_ws/src/arms_detection/docker/models/best.onnx
출력 : SW/arms_ws/src/arms_detection/docker/models/best_fp16.engine

실행:
    python export_trt.py

요구사항: tensorrt, CUDA GPU
주의: .engine 파일은 빌드한 GPU 아키텍처(Jetson Orin 등)에 종속됨.
      반드시 실기체(Jetson)에서 실행할 것.
"""

from pathlib import Path
import tensorrt as trt

ONNX_PATH   = Path(__file__).parent / "../arms_ws/src/arms_detection/docker/models/best.onnx"
ENGINE_PATH = ONNX_PATH.parent / "best_fp16.engine"

TRT_LOGGER = trt.Logger(trt.Logger.INFO)


def build_engine(onnx_path: Path, engine_path: Path):
    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser  = trt.OnnxParser(network, TRT_LOGGER)

    print(f"[INFO] ONNX 파싱 중: {onnx_path}")
    with open(onnx_path, "rb") as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print(f"[ERROR] {parser.get_error(i)}")
            raise RuntimeError("ONNX 파싱 실패")

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 2 << 30)  # 2 GB

    if builder.platform_has_fast_fp16:
        config.set_flag(trt.BuilderFlag.FP16)
        print("[INFO] FP16 모드 활성화")
    else:
        print("[WARN] 이 GPU는 FP16을 지원하지 않음 → FP32로 빌드")

    print("[INFO] 엔진 빌드 중 (수 분 소요)...")
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("엔진 빌드 실패")

    with open(engine_path, "wb") as f:
        f.write(serialized)

    print(f"[INFO] 저장 완료: {engine_path.resolve()}")
    print(f"[INFO] 파일 크기: {engine_path.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    build_engine(ONNX_PATH.resolve(), ENGINE_PATH.resolve())
