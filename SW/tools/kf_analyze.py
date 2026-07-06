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
import matplotlib.gridspec as gridspec


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

def plot(df: pd.DataFrame, kf_x: np.ndarray, kf_y: np.ndarray,
         save_path: Path | None = None):
    t  = df["time_sec"].values
    rx = df["x_center"].values
    ry = df["y_center"].values

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("Detection Raw vs CA-KF", fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.4, wspace=0.35)

    # --- x(t) ---
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(t, rx,  alpha=0.4, linewidth=0.8, label="raw",  color="steelblue")
    ax1.plot(t, kf_x, linewidth=1.5,           label="KF",   color="tomato")
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("x_center (normalized)")
    ax1.set_title("X(t)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # --- y(t) ---
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, ry,  alpha=0.4, linewidth=0.8, label="raw",  color="steelblue")
    ax2.plot(t, kf_y, linewidth=1.5,           label="KF",   color="tomato")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("y_center (normalized)")
    ax2.set_title("Y(t)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # --- x-y 궤적 ---
    ax3 = fig.add_subplot(gs[1, :])
    sc = ax3.scatter(rx, ry, c=t, cmap="Blues", s=4, alpha=0.5, label="raw")
    ax3.plot(kf_x, kf_y, color="tomato", linewidth=1.5, label="KF")
    plt.colorbar(sc, ax=ax3, label="time [s]")
    ax3.set_xlabel("x_center")
    ax3.set_ylabel("y_center")
    ax3.set_title("X-Y 궤적  (색 = 시간 경과)")
    ax3.invert_yaxis()   # 이미지 좌표계: y 아래가 +
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # --- 통계 출력 ---
    residual_x = rx - kf_x
    residual_y = ry - kf_y
    print(f"\n=== 잔차 통계 (raw - KF) ===")
    print(f"  x  RMSE: {np.sqrt(np.mean(residual_x**2)):.5f}  "
          f"std: {residual_x.std():.5f}  "
          f"max: {np.abs(residual_x).max():.5f}")
    print(f"  y  RMSE: {np.sqrt(np.mean(residual_y**2)):.5f}  "
          f"std: {residual_y.std():.5f}  "
          f"max: {np.abs(residual_y).max():.5f}")

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"\n그래프 저장: {save_path}")
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
