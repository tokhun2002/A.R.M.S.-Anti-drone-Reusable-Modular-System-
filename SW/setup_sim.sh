#!/usr/bin/env bash
# =============================================================================
#  A.R.M.S. SITL 셋업 — clone 후 한 번만 실행.
#  airframe 등록 → PX4 빌드 → ROS2 워크스페이스 빌드 까지 전부 처리.
#  이거 끝나면 ./run_arms.sh 로 바로 실행.
#
#  전제: ROS2 Humble / Gazebo Harmonic / MAVSDK 설치됨,
#        PX4-Autopilot 이 ~/PX4-Autopilot 에 clone 돼 있음.
#        (PX4 위치 다르면:  PX4_DIR=<경로> ./setup_sim.sh)
# =============================================================================
set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
SW_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
AIRFRAME_SRC="$SW_DIR/simulation/airframes/4021_gz_arms_drone"
PX4_AIRFRAME_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
CMAKE="$PX4_AIRFRAME_DIR/CMakeLists.txt"

# --- PX4 폴더 있는지 확인 ---
if [ ! -d "$PX4_DIR" ]; then
  echo "!! PX4-Autopilot 이 $PX4_DIR 에 없음."
  echo "   git clone https://github.com/PX4/PX4-Autopilot.git --recursive 먼저 하고,"
  echo "   다른 위치면 PX4_DIR=<경로> ./setup_sim.sh 로 실행."
  exit 1
fi

echo "[1/4] airframe 복사 (4021_gz_arms_drone)"
cp "$AIRFRAME_SRC" "$PX4_AIRFRAME_DIR/"

echo "[2/4] airframe CMake 등록"
if ! grep -q "4021_gz_arms_drone" "$CMAKE"; then
  if grep -q "4001_gz_x500" "$CMAKE"; then
    sed -i 's/\(4001_gz_x500\)/\1\n\t4021_gz_arms_drone/' "$CMAKE"
    echo "      추가됨"
  else
    echo "      !! 경고: CMakeLists 에 4001_gz_x500 패턴 없음 → 수동 등록 필요"
  fi
else
  echo "      이미 등록됨"
fi

echo "[3/4] PX4 SITL 빌드 (수 분 소요)"
( cd "$PX4_DIR" && make px4_sitl )

echo "[4/4] ROS2 워크스페이스 빌드"
source /opt/ros/humble/setup.bash
( cd "$SW_DIR/arms_ws" && colcon build --symlink-install )

echo ""
echo "============================================"
echo " 셋업 완료! 이제 ./run_arms.sh 로 실행하면 된다."
echo "============================================"
