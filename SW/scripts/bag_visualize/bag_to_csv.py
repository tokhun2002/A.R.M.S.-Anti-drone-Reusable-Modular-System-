#!/usr/bin/env python3
"""ROS2 bag(sqlite3/mcap) 의 데이터 토픽들을 토픽별 CSV 로 변환한다.

영상 토픽(CompressedImage/Image)은 건너뛰고(그건 bag_to_mp4.py 담당), 나머지
메시지를 평탄화해 `<outdir>/<토픽>.csv` 로 저장한다. 각 행은 bag 수신시각
t_ns(원본) 와 t_rel(첫 메시지 기준 초) 로 시작한다.

사용:
    python3 bag_to_csv.py <bag_dir> [--storage mcap|sqlite3] [--outdir DIR]
"""
import argparse
import csv
import sys
from pathlib import Path

from _common import (detect_storage, flatten_msg, load_message_class,
                     open_reader, sanitize_topic, topic_types)

# CSV 로 만들지 않을 타입(영상). bag_to_mp4.py 가 따로 처리한다.
_SKIP_TYPES = {"sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/Image"}


def bag_to_csv(bag: str, storage: str = "", out_dir: str = "") -> dict:
    """bag 의 데이터 토픽을 CSV 로 변환. {토픽: csv경로} 반환."""
    from rclpy.serialization import deserialize_message

    storage = storage or detect_storage(bag)
    out = Path(out_dir or (bag.rstrip("/") + "_viz/csv"))
    out.mkdir(parents=True, exist_ok=True)

    types = topic_types(bag, storage)
    targets = {t: ty for t, ty in types.items() if ty not in _SKIP_TYPES}
    if not targets:
        print(f"[csv] 변환할 데이터 토픽이 없음 (영상만 있는 bag?): {bag}")
        return {}
    classes = {t: load_message_class(ty) for t, ty in targets.items()}

    rows = {t: [] for t in targets}       # 토픽별 행(dict) 모음
    t0 = None
    reader = open_reader(bag, storage)
    while reader.has_next():
        tname, data, t_ns = reader.read_next()
        if tname not in targets:
            continue
        if t0 is None:
            t0 = t_ns
        msg = deserialize_message(data, classes[tname])
        row = {"t_ns": t_ns, "t_rel": (t_ns - t0) / 1e9}
        row.update(flatten_msg(msg))
        rows[tname].append(row)
    del reader

    written = {}
    for tname, items in rows.items():
        if not items:
            continue
        # 여러 행의 키 합집합으로 헤더 구성(예: 검출 0/1개로 열이 달라질 때).
        header = ["t_ns", "t_rel"]
        seen = set(header)
        for r in items:
            for k in r:
                if k not in seen:
                    seen.add(k)
                    header.append(k)
        path = out / f"{sanitize_topic(tname)}.csv"
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=header, extrasaction="ignore")
            w.writeheader()
            w.writerows(items)
        written[tname] = str(path)
        print(f"[csv] {tname}: {len(items)}행 → {path}")
    return written


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="bag 디렉토리")
    ap.add_argument("--storage", default="", help="mcap | sqlite3 (기본: 자동감지)")
    ap.add_argument("--outdir", default="", help="CSV 출력 폴더 (기본: <bag>_viz/csv)")
    args = ap.parse_args()
    written = bag_to_csv(args.bag, args.storage, args.outdir)
    return 0 if written else 1


if __name__ == "__main__":
    sys.exit(main())
