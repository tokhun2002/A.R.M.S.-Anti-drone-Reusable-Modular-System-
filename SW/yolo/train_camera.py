#!/usr/bin/env python3
"""드론 카메라 데이터로 기존 풍선 YOLO를 fine-tune한다."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--project", type=Path, required=True)
    args = parser.parse_args()

    model = YOLO(str(args.model.resolve()), task="detect")
    model.train(
        data=str(args.data.resolve()),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=2,
        project=str(args.project.resolve()),
        name="balloon_camera",
        exist_ok=True,
        patience=12,
        pretrained=True,
        cache=False,
        close_mosaic=5,
        degrees=8.0,
        translate=0.08,
        scale=0.35,
        fliplr=0.5,
        hsv_h=0.02,
        hsv_s=0.35,
        hsv_v=0.30,
        mosaic=0.5,
        mixup=0.0,
        plots=True,
    )


if __name__ == "__main__":
    main()
