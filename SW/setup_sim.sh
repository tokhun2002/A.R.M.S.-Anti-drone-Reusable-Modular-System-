#!/usr/bin/env bash
# =============================================================================
#  A.R.M.S. SITL 셋업 — clone 후 한 번만 실행.
#  모델 링크 → airframe 등록 → PX4 빌드 → ROS2 워크스페이스 빌드 까지 전부 처리.
#  이거 끝나면 ./run_arms.sh 로 바로 실행.
#
#  전제: ROS2 Humble / Gazebo Harmonic / MAVSDK 설치됨,
#        PX4-Autopilot 이 ~/PX4-Autopilot 에 clone 돼 있음.
#        (PX4 위치 다르면:  PX4_DIR=<경로> ./setup_sim.sh)
# =============================================================================
set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
SW_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
AIRFRAME_SRC="$SW_DIR/simulation/airframes/4022_gz_arms_drone"
PX4_AIRFRAME_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
PX4_MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"
CMAKE="$PX4_AIRFRAME_DIR/CMakeLists.txt"

# --- PX4 폴더 있는지 확인 ---
if [ ! -d "$PX4_DIR" ]; then
  echo "!! PX4-Autopilot 이 $PX4_DIR 에 없음."
  echo "   git clone https://github.com/PX4/PX4-Autopilot.git --recursive 먼저 하고,"
  echo "   다른 위치면 PX4_DIR=<경로> ./setup_sim.sh 로 실행."
  exit 1
fi

echo "[1/5] airframe 복사 (4022_gz_arms_drone)"
# 구버전 4021 잔재 제거 (x500_flow와 번호 충돌 방지)
rm -f "$PX4_AIRFRAME_DIR/4021_gz_arms_drone"
cp "$AIRFRAME_SRC" "$PX4_AIRFRAME_DIR/"

echo "[2/5] airframe CMake 등록"
if ! grep -q "4022_gz_arms_drone" "$CMAKE"; then
  if grep -q "4001_gz_x500" "$CMAKE"; then
    sed -i 's/\(4001_gz_x500\)/\1\n\t4022_gz_arms_drone/' "$CMAKE"
    echo "      추가됨"
  else
    echo "      !! 경고: CMakeLists 에 4001_gz_x500 패턴 없음 → 수동 등록 필요"
  fi
else
  echo "      이미 등록됨"
fi

echo "[3/5] 드론 모델 심볼릭 링크 (레포 → PX4 모델 폴더)"
#   PX4 가 'gz_arms_drone' 스폰 시 모델을 PX4 트리에서 찾으므로,
#   레포의 모델 폴더를 PX4 모델 경로에 링크해 둔다. (ln -sfn = 있으면 갱신)
mkdir -p "$PX4_MODELS_DIR"
for m in arms_drone x500 x500_base; do
  if [ -d "$SW_DIR/simulation/models/$m" ]; then
    ln -sfn "$SW_DIR/simulation/models/$m" "$PX4_MODELS_DIR/$m"
    echo "      링크: $PX4_MODELS_DIR/$m -> 레포/$m"
  fi
done

echo "[4/5] PX4 SITL 빌드 (수 분 소요)"
( cd "$PX4_DIR" && make px4_sitl )

echo "[5/5] ROS2 워크스페이스 빌드"
source /opt/ros/humble/setup.bash
( cd "$SW_DIR/arms_ws" && colcon build --symlink-install )

echo ""
echo "============================================"
echo " 셋업 완료! 이제 ./run_arms.sh 로 실행하면 된다."
echo "============================================"
