#!/usr/bin/env python3
"""드론 카메라 영상에서 풍선 YOLO fine-tuning 데이터셋을 만든다.

Gaussian red score와 원형도 기반으로 초벌 라벨을 생성한다. 전체 프레임과
후보 중심의 확대 ROI를 함께 저장해, 실기체의 proposal-ROI 추론과 학습 분포를
맞춘다. 자동 라벨은 완벽하지 않으므로 ``review`` contact sheet를 반드시 확인한다.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml


def red_probability(bgr: np.ndarray, hue_sigma: float = 12.0) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
    hue, sat, val = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    hue_dist = np.minimum(hue, 180.0 - hue)
    hue_score = np.exp(-0.5 * (hue_dist / hue_sigma) ** 2)
    sat_score = 1.0 / (1.0 + np.exp(-(sat - 75.0) / 22.0))
    val_low_score = 1.0 / (1.0 + np.exp(-(val - 30.0) / 12.0))
    val_high_score = 1.0 / (1.0 + np.exp((val - 225.0) / 18.0))
    return hue_score * sat_score * val_low_score * val_high_score


def find_balloon(bgr: np.ndarray, previous=None):
    """가장 풍선다운 붉은 원형 blob의 (x, y, w, h, score)를 반환한다."""
    h, w = bgr.shape[:2]
    prob = red_probability(bgr)
    mask = (prob >= 0.20).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for contour in contours:
        x, y, bw, bh = cv2.boundingRect(contour)
        frac = bw * bh / float(w * h)
        if not (0.00002 <= frac <= 0.008):
            continue
        aspect = min(bw, bh) / max(1.0, float(max(bw, bh)))
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)
        circularity = 4.0 * np.pi * area / (perimeter * perimeter) if perimeter else 1.0
        shape = max(float(circularity), 0.65 * aspect)
        if shape < 0.22 or aspect < 0.35:
            continue
        color = float(prob[y:y + bh, x:x + bw].mean())
        margin = max(12, 2 * max(bw, bh))
        px0, py0 = max(0, x - margin), max(0, y - margin)
        px1, py1 = min(w, x + bw + margin), min(h, y + bh + margin)
        context = cv2.cvtColor(bgr[py0:py1, px0:px1], cv2.COLOR_BGR2GRAY)
        texture = float(cv2.Laplacian(context, cv2.CV_32F).var()) if context.size else 1e6
        smooth = 1.0 / (1.0 + texture / 120.0)
        # 넓은 빨간 구조물보다 작고 둥글며 색이 진한 표적을 선호한다.
        size_penalty = 1.0 / (1.0 + max(0.0, frac - 0.001) * 800.0)
        score = (color * (0.35 + 0.65 * shape) * (0.5 + 0.5 * aspect) *
                 size_penalty * (0.15 + 0.85 * smooth))
        if previous is not None:
            pcx, pcy = previous
            cx, cy = (x + bw / 2) / w, (y + bh / 2) / h
            dist = float(np.hypot(cx - pcx, cy - pcy))
            # 0.5초 간격 프레임에서 표적 궤적은 연속적이다. 순간적으로 더 밝은
            # 건물 LED가 생겨도 기존 풍선 근처 후보를 유지한다.
            score *= 0.15 + 0.85 * float(np.exp(-0.5 * (dist / 0.12) ** 2))
        if best is None or score > best[-1]:
            best = (x, y, bw, bh, score)
    return best


def write_label(path: Path, box, width: int, height: int):
    if box is None:
        path.write_text("", encoding="utf-8")
        return
    x, y, bw, bh, _ = box
    # 압축된 작은 점도 충분한 문맥을 포함하도록 박스를 약간 확장한다.
    pad = max(2, int(round(max(bw, bh) * 0.25)))
    x0, y0 = max(0, x - pad), max(0, y - pad)
    x1, y1 = min(width, x + bw + pad), min(height, y + bh + pad)
    xc, yc = (x0 + x1) / 2 / width, (y0 + y1) / 2 / height
    nw, nh = (x1 - x0) / width, (y1 - y0) / height
    path.write_text(f"0 {xc:.7f} {yc:.7f} {nw:.7f} {nh:.7f}\n", encoding="utf-8")


def save_roi(frame, box, image_path: Path, label_path: Path, side: int = 192):
    h, w = frame.shape[:2]
    x, y, bw, bh, _ = box
    cx, cy = x + bw // 2, y + bh // 2
    half = side // 2
    x0, y0 = max(0, cx - half), max(0, cy - half)
    x1, y1 = min(w, x0 + side), min(h, y0 + side)
    x0, y0 = max(0, x1 - side), max(0, y1 - side)
    crop = frame[y0:y1, x0:x1]
    if crop.size == 0:
        return
    cv2.imwrite(str(image_path), crop, [cv2.IMWRITE_JPEG_QUALITY, 94])
    local = (x - x0, y - y0, bw, bh, box[-1])
    write_label(label_path, local, crop.shape[1], crop.shape[0])


def save_negative_roi(frame, box, image_path: Path, label_path: Path, side: int = 192):
    """풍선에서 가장 먼 모서리 crop을 빈 라벨 hard negative로 저장한다."""
    h, w = frame.shape[:2]
    centers = [(side // 2, side // 2), (w - side // 2, side // 2),
               (side // 2, h - side // 2), (w - side // 2, h - side // 2)]
    if box is None:
        cx, cy = centers[-1]
    else:
        tx, ty = box[0] + box[2] / 2, box[1] + box[3] / 2
        cx, cy = max(centers, key=lambda p: (p[0] - tx) ** 2 + (p[1] - ty) ** 2)
    x0, y0 = max(0, cx - side // 2), max(0, cy - side // 2)
    crop = frame[y0:min(h, y0 + side), x0:min(w, x0 + side)]
    if crop.size:
        cv2.imwrite(str(image_path), crop, [cv2.IMWRITE_JPEG_QUALITY, 94])
        label_path.write_text("", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--videos", type=Path, default=Path("../camera_test"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sample-fps", type=float, default=2.0)
    args = parser.parse_args()

    root = args.output.resolve()
    if root.exists():
        shutil.rmtree(root)
    for split in ("train", "val"):
        (root / "images" / split).mkdir(parents=True)
        (root / "labels" / split).mkdir(parents=True)
    (root / "review").mkdir(parents=True)

    videos = sorted(args.videos.resolve().glob("*.mp4"))
    if not videos:
        raise SystemExit(f"no mp4 files in {args.videos}")
    # 가운데 영상은 건물/조명이 많은 hard validation 장면이다.
    val_video = videos[1] if len(videos) > 1 else videos[0]
    durations = {}
    for video in videos:
        probe = cv2.VideoCapture(str(video))
        vf = probe.get(cv2.CAP_PROP_FPS) or 24.0
        durations[video] = probe.get(cv2.CAP_PROP_FRAME_COUNT) / vf
        probe.release()
    # [전체 이미지 수, 풍선 후보 프레임 수]
    stats = {"train": [0, 0], "val": [0, 0]}
    review_frames = []

    for video in videos:
        split = "val" if video == val_video else "train"
        cap = cv2.VideoCapture(str(video))
        fps = cap.get(cv2.CAP_PROP_FPS) or 24.0
        stride = max(1, int(round(fps / args.sample_fps)))
        frame_i = 0
        previous = None
        sample_i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if frame_i % stride:
                frame_i += 1
                continue
            time_sec = frame_i / fps
            # 마지막 약 3초는 전송 손상 프레임일 수 있으므로 데이터셋에서 제외한다.
            if time_sec >= durations[video] - 3.0:
                frame_i += 1
                continue
            box = find_balloon(frame, previous)
            if box is not None:
                previous = ((box[0] + box[2] / 2) / frame.shape[1],
                            (box[1] + box[3] / 2) / frame.shape[0])
            stem = f"{video.stem}_{frame_i:06d}"
            image_path = root / "images" / split / f"{stem}.jpg"
            label_path = root / "labels" / split / f"{stem}.txt"
            if box is not None:
                stats[split][1] += 1
                # 320 학습에서 12px 미만 표적은 6px 미만으로 줄어 P3 검출 헤드에
                # 지나치게 작다. 그런 원거리 표적은 proposal ROI로만 학습한다.
                if max(box[2], box[3]) >= 12:
                    cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 94])
                    write_label(label_path, box, frame.shape[1], frame.shape[0])
                    stats[split][0] += 1
                save_roi(frame, box,
                         root / "images" / split / f"{stem}_roi.jpg",
                         root / "labels" / split / f"{stem}_roi.txt")
                stats[split][0] += 1
            else:
                cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 94])
                write_label(label_path, None, frame.shape[1], frame.shape[0])
                stats[split][0] += 1
            if sample_i % 3 == 0:
                save_negative_roi(
                    frame, box,
                    root / "images" / split / f"{stem}_negative.jpg",
                    root / "labels" / split / f"{stem}_negative.txt")
                stats[split][0] += 1
            if len(review_frames) < 120 or frame_i % (stride * 15) == 0:
                preview = frame.copy()
                if box is not None:
                    x, y, bw, bh, score = box
                    cv2.rectangle(preview, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                    cv2.putText(preview, f"{score:.2f}", (x, max(16, y - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                review_frames.append(cv2.resize(preview, (320, 240)))
            frame_i += 1
            sample_i += 1
        cap.release()

    # 검수용 contact sheets.
    for page, start in enumerate(range(0, len(review_frames), 24)):
        batch = review_frames[start:start + 24]
        while len(batch) < 24:
            batch.append(np.zeros_like(batch[0]))
        sheet = np.vstack([np.hstack(batch[i:i + 4]) for i in range(0, 24, 4)])
        cv2.imwrite(str(root / "review" / f"page_{page:02d}.jpg"), sheet)

    data = {
        "path": str(root), "train": "images/train", "val": "images/val",
        "names": {0: "balloon"},
    }
    (root / "data.yaml").write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    print(f"dataset: {root}")
    for split, (total, candidate_frames) in stats.items():
        print(f"{split}: {total} images ({candidate_frames} candidate frames; "
              "each contributes an ROI and, when large enough, a full frame)")


if __name__ == "__main__":
    main()
