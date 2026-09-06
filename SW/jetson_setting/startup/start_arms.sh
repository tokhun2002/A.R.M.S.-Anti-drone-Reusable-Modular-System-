#!/usr/bin/env bash
#
# A.R.M.S. 실기체 부팅 자동 실행 스크립트
#   `ros2 launch arms_bringup arms.launch.py` 를 안전하게 기동한다.
#   부팅 직후 도커 데몬/USB 장치가 아직 안 올라왔을 수 있어 준비를 기다린다.
#
# 이 스크립트는 systemd 서비스(arms.service) 또는 데스크톱 autostart
# (arms-autostart.desktop) 에서 호출한다. 설정법은 같은 폴더 README.md 참고.
#
# 환경변수로 조정 가능:
#   ARMS_ROOT        리포 루트 (기본: 아래 경로)
#   ROS_DOMAIN_ID    ROS 도메인 (기본 0)
#   ARMS_DEVICE_WAIT 부팅 후 USB 장치 안정화 대기[s] (기본 5)
#   ARMS_LOG_DIR     로그 폴더 (기본 $HOME/arms_logs)

set -u

# ── 경로 설정 (환경이 다르면 여기 또는 ARMS_ROOT 환경변수로 수정) ──
ARMS_ROOT="${ARMS_ROOT:-/home/arms/A.R.M.S.-Anti-drone-Reusable-Modular-System-}"
WS="$ARMS_ROOT/SW/arms_ws"

# ── 로그 (systemd 는 journald 로도 잡히지만, 파일로도 남긴다) ──
LOG_DIR="${ARMS_LOG_DIR:-$HOME/arms_logs}"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/arms_startup_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOG") 2>&1
echo "[startup] $(date) — A.R.M.S. 실기체 기동 시작 (ARMS_ROOT=$ARMS_ROOT)"

# ── ROS 2 환경 ────────────────────────────────────────────────
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[startup][ERROR] ROS 2 Humble 없음 (/opt/ros/humble)"; exit 1
fi
# ROS/ament 의 setup 스크립트는 nounset-clean 이 아니라(예: AMENT_TRACE_SETUP_FILES
# 미정의 참조) `set -u` 하에서 죽는다 → 소싱 구간만 nounset 잠시 해제.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "[startup][ERROR] 워크스페이스 빌드 없음: $WS/install"
  echo "                 먼저: cd $WS && colcon build"
  exit 1
fi
set +u
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ── 도커 데몬 준비 대기 (detection 컨테이너용) ────────────────
echo "[startup] 도커 데몬 대기..."
docker_ok=0
for _ in $(seq 1 60); do
  if docker info >/dev/null 2>&1; then docker_ok=1; break; fi
  sleep 1
done
if [ "$docker_ok" = 1 ]; then
  echo "[startup] 도커 준비됨"
else
  echo "[startup][WARN] 도커 데몬 60s 내 준비 안 됨 — detection 컨테이너가 안 뜰 수 있음"
fi

# ── USB 장치 안정화 대기 ──────────────────────────────────────
#   ESP32 조종기(/dev/serial/by-id/...)·카메라(/dev/video0)·CRSF UART(/dev/ttyTHS1)
#   가 enumerate 될 시간. 없어도 런치는 진행(각 노드가 자체 재시도).
sleep "${ARMS_DEVICE_WAIT:-5}"

# ── 실기체 런치 ───────────────────────────────────────────────
echo "[startup] ros2 launch arms_bringup arms.launch.py"
exec ros2 launch arms_bringup arms.launch.py
