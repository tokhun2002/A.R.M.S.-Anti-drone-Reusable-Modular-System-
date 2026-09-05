#!/usr/bin/env python3
"""ROS2 bag(sqlite3/mcap)의 CompressedImage(JPEG) 토픽을 mp4(H.264) 동영상으로 변환.

재생(ros2 bag play) 없이 bag 을 직접 읽어 변환한다(라이브 토픽 충돌 없음).
bag 안의 JPEG 바이트를 ffmpeg 에 그대로 파이프해 H.264(yuv420p)로 인코딩 →
브라우저/폰/기본 플레이어 어디서나 재생된다. fps 는 타임스탬프에서 자동 계산.

필요: ffmpeg (libx264). 사용:
    python3 bag_to_mp4.py <bag_dir> [--topic /arms/ui_image/compressed]
                                    [--out out.mp4] [--fps N] [--storage sqlite3|mcap]
예:
    python3 SW/scripts/bag_to_mp4.py ui_rec
"""
import argparse
import shutil
import subprocess
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage


def open_reader(uri: str, storage: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=storage),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="bag 디렉토리 (예: ui_rec)")
    ap.add_argument("--topic", default="/arms/ui_image/compressed")
    ap.add_argument("--out", default=None, help="출력 mp4 (기본: <bag>.mp4)")
    ap.add_argument("--fps", type=float, default=0.0, help="0=타임스탬프에서 자동")
    ap.add_argument("--storage", default="sqlite3", help="sqlite3 | mcap")
    args = ap.parse_args()

    if shutil.which("ffmpeg") is None:
        print("[ERR] ffmpeg 가 없습니다. sudo apt install ffmpeg")
        return 1

    out_path = args.out or (args.bag.rstrip("/") + ".mp4")

    # --- pass 1: 프레임 수 & 타임스탬프 범위로 fps 계산 ---
    reader = open_reader(args.bag, args.storage)
    n, t_first, t_last = 0, None, None
    while reader.has_next():
        tname, _data, t = reader.read_next()
        if tname != args.topic:
            continue
        if t_first is None:
            t_first = t
        t_last = t
        n += 1
    del reader

    if n == 0:
        print(f"[ERR] '{args.topic}' 메시지가 bag 에 없음. 'ros2 bag info {args.bag}' 확인.")
        return 1

    fps = args.fps
    if fps <= 0.0:
        span_s = (t_last - t_first) / 1e9 if (t_last and t_first and n > 1) else 0.0
        fps = (n - 1) / span_s if span_s > 0 else 15.0
    print(f"[INFO] {n} 프레임, fps={fps:.2f} → {out_path} (H.264)")

    # --- ffmpeg: 표준입력으로 들어오는 JPEG 스트림을 H.264 로 인코딩 ---
    #   -vf: H.264 yuv420p 는 짝수 해상도 필요 → 홀수면 1px 패딩.
    ffmpeg = subprocess.Popen(
        [
            "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
            "-f", "image2pipe", "-framerate", f"{fps:.4f}", "-i", "-",
            "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            out_path,
        ],
        stdin=subprocess.PIPE,
    )

    # --- pass 2: bag 의 JPEG 바이트를 그대로 파이프 ---
    reader = open_reader(args.bag, args.storage)
    written = 0
    try:
        while reader.has_next():
            tname, data, _t = reader.read_next()
            if tname != args.topic:
                continue
            msg = deserialize_message(data, CompressedImage)
            ffmpeg.stdin.write(bytes(msg.data))
            written += 1
    finally:
        ffmpeg.stdin.close()
        ffmpeg.wait()

    if ffmpeg.returncode != 0:
        print(f"[ERR] ffmpeg 실패 (code {ffmpeg.returncode})")
        return 1
    print(f"[INFO] 완료: {written} 프레임 → {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
