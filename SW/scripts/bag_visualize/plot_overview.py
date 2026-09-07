#!/usr/bin/env python3
"""bag_to_csv.py 가 만든 CSV 들로 오버뷰 그래프(PNG)를 생성한다.

ROS 의존성이 없다 — CSV 만 있으면 어느 PC 에서나 돈다(matplotlib, pandas 필요).

그래프:
  1) target_plane.png  — 화면(정규화 좌표) 상 표적 위치. detection_raw vs KF 적용값.
  2) attitude_cmd.png  — roll/pitch/yaw(자세)와 roll/pitch/yaw(제어명령) 6개 시계열.

사용:
    python3 plot_overview.py <csv_dir> [--outdir DIR] [--aspect 16:9]
"""
import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")            # 헤드리스(디스플레이 없이 파일 저장)
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd              # noqa: E402

# CRSF 채널 값 → 정규화 스틱 [-1, 1]
_CRSF_MIN, _CRSF_MAX = 172, 1811
_CRSF_CENTER = (_CRSF_MIN + _CRSF_MAX) / 2.0
_CRSF_HALF = (_CRSF_MAX - _CRSF_MIN) / 2.0


def _load(csv_dir: Path, name: str):
    """<csv_dir>/<name>.csv → DataFrame, 없으면 None."""
    path = csv_dir / f"{name}.csv"
    if not path.exists():
        return None
    try:
        df = pd.read_csv(path)
        return df if len(df) else None
    except Exception as e:
        print(f"[plot] {path} 읽기 실패: {e}")
        return None


def _crsf_norm(series):
    return ((series - _CRSF_CENTER) / _CRSF_HALF).clip(-1.0, 1.0)


def _empty_axis(ax, text):
    ax.text(0.5, 0.5, text, ha="center", va="center", transform=ax.transAxes,
            color="#888", fontsize=10)
    ax.set_xticks([])
    ax.set_yticks([])


def plot_target_plane(csv_dir: Path, out_path: Path, aspect="16:9") -> bool:
    """화면비 그래프: 정규화 좌표(0..1)의 표적 위치. raw(주황) vs KF(파랑)."""
    raw = _load(csv_dir, "arms_detections_raw")
    kf = _load(csv_dir, "arms_detections")
    if raw is None and kf is None:
        print("[plot] detections_raw / detections 둘 다 없음 → target_plane 생략")
        return False

    try:
        aw, ah = (float(x) for x in aspect.split(":"))
    except Exception:
        aw, ah = 16.0, 9.0

    xcol, ycol = "detections_0_x_center", "detections_0_y_center"
    fig, ax = plt.subplots(figsize=(9, 9 * ah / aw))
    if raw is not None and xcol in raw:
        r = raw.dropna(subset=[xcol, ycol])
        ax.scatter(r[xcol], r[ycol], s=12,
                   c="#ff9800", alpha=0.45, label=f"raw ({len(r)})", zorder=2)
    if kf is not None and xcol in kf:
        k = kf.dropna(subset=[xcol, ycol])
        ax.plot(k[xcol], k[ycol], "-", c="#1f77b4",
                lw=1.0, alpha=0.8, zorder=3)
        ax.scatter(k[xcol], k[ycol], s=8,
                   c="#1f77b4", alpha=0.7, label=f"KF ({len(k)})", zorder=4)

    ax.axvline(0.5, color="#bbb", ls="--", lw=0.8, zorder=1)
    ax.axhline(0.5, color="#bbb", ls="--", lw=0.8, zorder=1)
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.invert_yaxis()                 # 이미지 좌표(위=0)
    ax.set_aspect(ah / aw)            # 화면비 유지(예: 16:9)
    ax.set_xlabel("x (normalized)")
    ax.set_ylabel("y (normalized)")
    ax.set_title("Target position in frame — raw vs KF")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"[plot] {out_path}")
    return True


def plot_attitude_cmd(csv_dir: Path, out_path: Path) -> bool:
    """roll/pitch/yaw(자세, crsf_rx) + roll/pitch/yaw(제어명령, crsf_tx) 6개 subplot."""
    rx = _load(csv_dir, "arms_crsf_rx")
    tx = _load(csv_dir, "arms_crsf_tx")
    if rx is None and tx is None:
        print("[plot] crsf_rx / crsf_tx 둘 다 없음 → attitude_cmd 생략")
        return False

    fig, axes = plt.subplots(2, 3, figsize=(15, 7), sharex=True)

    # 1행: 자세(수신 텔레메트리, deg)
    att = [("roll_deg", "roll (deg)"), ("pitch_deg", "pitch (deg)"),
           ("yaw_deg", "yaw (deg)")]
    for ax, (col, title) in zip(axes[0], att):
        if rx is not None and col in rx:
            ax.plot(rx["t_rel"], rx[col], lw=0.9, color="#1f77b4")
        else:
            _empty_axis(ax, "no /arms/crsf_rx")
        ax.set_title(f"attitude {title}")
        ax.grid(True, alpha=0.3)

    # 2행: 제어명령(송신 CRSF 채널 → 정규화 [-1,1]). CH1=roll, CH2=pitch, CH4=yaw.
    cmd = [("data_0", "roll cmd"), ("data_1", "pitch cmd"), ("data_3", "yaw cmd")]
    for ax, (col, title) in zip(axes[1], cmd):
        if tx is not None and col in tx:
            ax.plot(tx["t_rel"], _crsf_norm(tx[col]), lw=0.9, color="#d62728")
            ax.set_ylim(-1.05, 1.05)
        else:
            _empty_axis(ax, "no /arms/crsf_tx")
        ax.set_title(f"command {title} (norm)")
        ax.set_xlabel("t (s)")
        ax.grid(True, alpha=0.3)

    fig.suptitle("Attitude (crsf_rx) & control commands (crsf_tx)", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"[plot] {out_path}")
    return True


def plot_overview(csv_dir: str, out_dir: str = "", aspect="16:9") -> list:
    """모든 오버뷰 그래프 생성. 저장된 PNG 경로 목록 반환."""
    csv_dir = Path(csv_dir)
    out = Path(out_dir) if out_dir else csv_dir.parent
    out.mkdir(parents=True, exist_ok=True)
    made = []
    if plot_target_plane(csv_dir, out / "target_plane.png", aspect):
        made.append(str(out / "target_plane.png"))
    if plot_attitude_cmd(csv_dir, out / "attitude_cmd.png"):
        made.append(str(out / "attitude_cmd.png"))
    return made


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_dir", help="bag_to_csv.py 가 만든 CSV 폴더")
    ap.add_argument("--outdir", default="", help="PNG 출력 폴더 (기본: csv_dir 의 상위)")
    ap.add_argument("--aspect", default="16:9", help="target_plane 화면비 (예: 16:9)")
    args = ap.parse_args()
    made = plot_overview(args.csv_dir, args.outdir, args.aspect)
    return 0 if made else 1


if __name__ == "__main__":
    sys.exit(main())
