#!/usr/bin/env python3
"""마스터 스크립트: bag 하나를 넣으면 CSV 변환 + 오버뷰 그래프 + mp4 영상까지 한 번에.

    python3 bag_visualize.py <bag_dir> [옵션]

결과는 <bag>_viz/ 아래에 모인다:
    <bag>_viz/csv/*.csv          토픽별 CSV (bag_to_csv.py)
    <bag>_viz/target_plane.png   화면비 그래프: 표적 위치 raw vs KF
    <bag>_viz/attitude_cmd.png   roll/pitch/yaw 자세 + 제어명령 6개 시계열
    <bag>_viz/<bag>.mp4          UI 화면 영상 (bag_to_mp4.py)

각 단계 스크립트(bag_to_csv.py / plot_overview.py / bag_to_mp4.py)는 따로도 실행 가능.
필요: ros2(rosbag2 python), ffmpeg, matplotlib, pandas.
"""
import argparse
import os
import sys
from pathlib import Path

# 같은 폴더의 모듈을 import (cwd 와 무관하게).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from _common import detect_storage  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="bag 디렉토리 (예: ~/arms_flight_log/20260907_1530)")
    ap.add_argument("--outdir", default="", help="결과 폴더 (기본: <bag>_viz)")
    ap.add_argument("--storage", default="", help="mcap | sqlite3 (기본: 자동감지)")
    ap.add_argument("--video-topic", default="/arms/ui_image/compressed")
    ap.add_argument("--fps", type=float, default=0.0, help="영상 fps (0=자동)")
    ap.add_argument("--aspect", default="16:9", help="target_plane 화면비")
    ap.add_argument("--no-video", action="store_true", help="mp4 변환 건너뜀")
    ap.add_argument("--no-csv", action="store_true", help="CSV 변환 건너뜀(→ 그래프도 생략)")
    ap.add_argument("--no-plots", action="store_true", help="그래프 생성 건너뜀")
    args = ap.parse_args()

    bag = args.bag.rstrip("/")
    if not Path(bag).exists():
        print(f"[master] bag 이 없음: {bag}")
        return 1
    storage = args.storage or detect_storage(bag)
    outdir = Path(args.outdir or (bag + "_viz"))
    csv_dir = outdir / "csv"
    outdir.mkdir(parents=True, exist_ok=True)
    print(f"[master] bag={bag}  storage={storage}  out={outdir}")

    results = {}

    # 1) CSV
    if not args.no_csv:
        try:
            from bag_to_csv import bag_to_csv
            results["csv"] = bag_to_csv(bag, storage, str(csv_dir))
        except Exception as e:
            print(f"[master] CSV 변환 실패: {e}")

    # 2) 그래프 (CSV 필요)
    if not args.no_plots and not args.no_csv:
        try:
            from plot_overview import plot_overview
            results["plots"] = plot_overview(str(csv_dir), str(outdir), args.aspect)
        except Exception as e:
            print(f"[master] 그래프 생성 실패: {e}")
    elif not args.no_plots and args.no_csv:
        print("[master] --no-csv 라 그래프도 생략(그래프는 CSV 를 읽음).")

    # 3) 영상
    if not args.no_video:
        try:
            from bag_to_mp4 import bag_to_mp4
            out_mp4 = str(outdir / (Path(bag).name + ".mp4"))
            results["video"] = bag_to_mp4(bag, args.video_topic, out_mp4,
                                          args.fps, storage)
        except Exception as e:
            print(f"[master] 영상 변환 실패: {e}")

    # 요약
    print("\n[master] 완료 요약")
    csvs = results.get("csv") or {}
    print(f"  CSV   : {len(csvs)}개 → {csv_dir}")
    for p in (results.get("plots") or []):
        print(f"  그래프: {p}")
    if results.get("video"):
        print(f"  영상  : {results['video']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
