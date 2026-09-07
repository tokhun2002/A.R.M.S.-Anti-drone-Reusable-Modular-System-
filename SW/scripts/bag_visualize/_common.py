#!/usr/bin/env python3
"""bag_visualize 공용 헬퍼: rosbag2 읽기 · 스토리지 자동감지 · 메시지 평탄화.

여기서만 rosbag2_py / rosidl 런타임을 import 한다(그래프 스크립트는 CSV 만 읽어
ROS 없이도 돈다). ROS2(rosbag2 python)가 있는 환경에서만 이 모듈을 쓴다.
"""
from pathlib import Path


def detect_storage(bag: str) -> str:
    """bag 의 metadata.yaml 에서 storage_identifier(mcap/sqlite3)를 읽는다. 실패 시 'mcap'."""
    meta = Path(bag) / "metadata.yaml"
    try:
        import yaml
        with open(meta) as f:
            data = yaml.safe_load(f)
        sid = (data.get("rosbag2_bagfile_information", {})
                   .get("storage_identifier"))
        if sid:
            return str(sid)
    except Exception:
        pass
    return "mcap"


def open_reader(uri: str, storage: str):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=storage),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def topic_types(bag: str, storage: str) -> dict:
    """{토픽명: 타입문자열} 반환 (예: '/arms/detections' -> 'arms_msgs/msg/DetectionArray')."""
    reader = open_reader(bag, storage)
    out = {t.name: t.type for t in reader.get_all_topics_and_types()}
    del reader
    return out


def load_message_class(type_str: str):
    """'pkg/msg/Type' 또는 'pkg/Type' 문자열 → 메시지 클래스."""
    from rosidl_runtime_py.utilities import get_message
    return get_message(type_str)


def sanitize_topic(topic: str) -> str:
    """'/arms/detections_raw' -> 'arms_detections_raw' (CSV 파일명용)."""
    return topic.strip("/").replace("/", "_") or "root"


# 평탄화에서 건너뛸 필드(잡음). MultiArray 의 layout 등.
_SKIP_FIELDS = {"layout"}


def flatten_msg(msg, prefix: str = "", max_array: int = 16) -> dict:
    """ROS 메시지를 {열이름: 스칼라} 로 평탄화한다.

    - 중첩 메시지: prefix 로 재귀 (header.stamp -> header_stamp_sec ...).
    - 원시 배열: 앞 max_array 개를 name_0, name_1 ... 로 전개.
    - 메시지 배열(예: DetectionArray.detections): 개수(name_count)와
      첫 원소(name_0_*)만 남긴다(표적은 0/1개라 이걸로 충분).
    """
    out = {}
    fields = msg.get_fields_and_field_types()
    for name in fields:
        if name in _SKIP_FIELDS:
            continue
        val = getattr(msg, name)
        key = f"{prefix}{name}"
        if hasattr(val, "get_fields_and_field_types"):          # 중첩 메시지
            out.update(flatten_msg(val, key + "_", max_array))
        elif isinstance(val, (str, bytes, bool, int, float)):   # 스칼라
            out[key] = val
        elif hasattr(val, "__len__"):                           # 배열/시퀀스
            seq = list(val)
            if seq and hasattr(seq[0], "get_fields_and_field_types"):
                out[key + "_count"] = len(seq)                  # 메시지 배열 → 개수 + 첫 원소
                out.update(flatten_msg(seq[0], key + "_0_", max_array))
            else:
                for i, v in enumerate(seq[:max_array]):
                    out[f"{key}_{i}"] = v
        else:
            out[key] = val
    return out
