#!/usr/bin/env python3
"""
kf_analyze.py — CA(등가속도) 모델 Kalman Filter 적용 후 raw vs KF 비교 그래프

상태 벡터: [x, y, vx, vy, ax, ay]  (6D)
관측 벡터: [x, y]                  (2D)

사용법:
  python3 kf_analyze.py detections_20260101_120000.csv
  python3 kf_analyze.py detections_20260101_120000.csv --save  # PNG 저장
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# CA 모델 Kalman Filter
# ---------------------------------------------------------------------------

def make_F(dt: float) -> np.ndarray:
    """상태 전이 행렬 (dt 가변)."""
    return np.array([
        [1, 0, dt, 0,  0.5*dt**2, 0        ],
        [0, 1, 0,  dt, 0,         0.5*dt**2],
        [0, 0, 1,  0,  dt,        0        ],
        [0, 0, 0,  1,  0,         dt       ],
        [0, 0, 0,  0,  1,         0        ],
        [0, 0, 0,  0,  0,         1        ],
    ], dtype=float)

H = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
], dtype=float)


def run_kalman(times: np.ndarray, xs: np.ndarray, ys: np.ndarray,
               q_pos=1e-4, q_vel=1e-3, q_acc=1e-2,
               r_pos=1e-3) -> tuple[np.ndarray, np.ndarray]:
    """
    CA Kalman Filter 실행.

    q_pos/q_vel/q_acc : 프로세스 노이즈 (위치/속도/가속도)
    r_pos             : 관측 노이즈

    반환: (kf_x, kf_y) — raw와 같은 길이의 필터링 결과
    """
    n = len(times)
    Q_diag = [q_pos, q_pos, q_vel, q_vel, q_acc, q_acc]
    Q = np.diag(Q_diag)
    R = np.eye(2) * r_pos

    # 초기화: 첫 관측으로 초기 상태 설정
    x = np.array([xs[0], ys[0], 0, 0, 0, 0], dtype=float)
    P = np.eye(6) * 1.0

    kf_x = np.zeros(n)
    kf_y = np.zeros(n)
    kf_x[0], kf_y[0] = xs[0], ys[0]

    for i in range(1, n):
        dt = times[i] - times[i - 1]
        if dt <= 0:
            dt = 1e-3

        F = make_F(dt)

        # Predict
        x = F @ x
        P = F @ P @ F.T + Q

        # Update
        z = np.array([xs[i], ys[i]])
        y_res = z - H @ x
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x = x + K @ y_res
        P = (np.eye(6) - K @ H) @ P

        kf_x[i] = x[0]
        kf_y[i] = x[1]

    return kf_x, kf_y


# ---------------------------------------------------------------------------
# 그래프
# ---------------------------------------------------------------------------

# upward camera resolution → aspect ratio for normalized coords
CAM_W, CAM_H = 1280, 720


def plot(df: pd.DataFrame, kf_x: np.ndarray, kf_y: np.ndarray,
         save_path: Path | None = None):
    rx = df["x_center"].values
    ry = df["y_center"].values

    # figure size: width fixed, height scaled to camera aspect ratio
    fig_w = 9
    fig_h = fig_w * CAM_H / CAM_W
    fig, ax = plt.subplots(figsize=(fig_w, fig_h))

    ax.plot(rx, ry, color="blue", linewidth=1.0, label="raw")
    ax.plot(kf_x, kf_y, color="tomato", linewidth=1.5, label="CA-KF")

    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.invert_yaxis()   # image coord: y+ is downward
    ax.set_aspect("equal")
    ax.set_xlabel("x_center")
    ax.set_ylabel("y_center")
    ax.set_title("Detection Trajectory: Raw vs CA-KF")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # stats
    res_x = rx - kf_x
    res_y = ry - kf_y
    print("\n=== residual (raw - KF) ===")
    print(f"  x  RMSE: {np.sqrt(np.mean(res_x**2)):.5f}  "
          f"std: {res_x.std():.5f}  max: {np.abs(res_x).max():.5f}")
    print(f"  y  RMSE: {np.sqrt(np.mean(res_y**2)):.5f}  "
          f"std: {res_y.std():.5f}  max: {np.abs(res_y).max():.5f}")

    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"\nsaved: {save_path}")
    else:
        plt.show()


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", help="log_detections.py 로 생성한 CSV 파일")
    parser.add_argument("--save", action="store_true", help="PNG로 저장 (화면 표시 안 함)")
    parser.add_argument("--q-pos", type=float, default=1e-4, help="프로세스 노이즈 위치 (default: 1e-4)")
    parser.add_argument("--q-vel", type=float, default=1e-3, help="프로세스 노이즈 속도 (default: 1e-3)")
    parser.add_argument("--q-acc", type=float, default=1e-2, help="프로세스 노이즈 가속도 (default: 1e-2)")
    parser.add_argument("--r-pos", type=float, default=1e-3, help="관측 노이즈 (default: 1e-3)")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"파일 없음: {csv_path}", file=sys.stderr)
        sys.exit(1)

    df = pd.read_csv(csv_path)
    print(f"로드: {csv_path}  ({len(df)} 샘플, "
          f"{df['time_sec'].iloc[-1]:.1f}s)")

    t  = df["time_sec"].values
    xs = df["x_center"].values
    ys = df["y_center"].values

    kf_x, kf_y = run_kalman(t, xs, ys,
                             q_pos=args.q_pos,
                             q_vel=args.q_vel,
                             q_acc=args.q_acc,
                             r_pos=args.r_pos)

    save_path = csv_path.with_suffix(".png") if args.save else None
    plot(df, kf_x, kf_y, save_path)


if __name__ == "__main__":
    main()
