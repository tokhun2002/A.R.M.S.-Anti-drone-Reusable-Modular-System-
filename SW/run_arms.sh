#!/usr/bin/env bash
# =============================================================================
#  A.R.M.S. SITL 원클릭 실행기 (inair 경로 고정판)
#  ./run_arms.sh 하나로 PX4(arms_drone) + 전체 스택 + 패널 GUI 실행.
#        (PX4 위치 다르면:  PX4_DIR=<경로> ./setup_sim.sh)
# =============================================================================
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
ARMS_SW="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
# =============================================================================

ARMS_WS="$ARMS_SW/arms_ws"
export ARMS_SW
ROS_SETUP="source /opt/ros/humble/setup.bash; source $ARMS_WS/install/setup.bash; export ARMS_SW=$ARMS_SW"
export GZ_SIM_RESOURCE_PATH="$ARMS_SW/simulation/models:$GZ_SIM_RESOURCE_PATH"

# 이전 인스턴스 깨끗이 정리
pkill -9 -f px4 2>/dev/null; pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f ruby 2>/dev/null; pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f arms_control 2>/dev/null
pkill -9 -f balloon_referee 2>/dev/null; pkill -9 -f arms_panel 2>/dev/null
pkill -9 -f fusion_detector 2>/dev/null; pkill -9 -f arms_ui 2>/dev/null
pkill -9 -f "socat.*crsf" 2>/dev/null
sleep 3

# CRSF 가상 시리얼 쌍 생성 (socat PTY pair)
#   arms_control → /tmp/crsf_tx → [socat] → /tmp/crsf_rx → arms_comm_sitl
if ! command -v socat >/dev/null; then
  echo "!! socat 없음: sudo apt install socat"
  exit 1
fi
rm -f /tmp/crsf_tx /tmp/crsf_rx
socat PTY,raw,echo=0,link=/tmp/crsf_tx PTY,raw,echo=0,link=/tmp/crsf_rx &
echo "CRSF 가상 시리얼: /tmp/crsf_tx (arms_control) ↔ /tmp/crsf_rx (arms_comm)"
sleep 1

# 수정한 월드(red_ball kinematic 등)를 PX4 월드 폴더로 복사 → 항상 최신 반영
PX4_WORLDS="$PX4_DIR/Tools/simulation/gz/worlds"
if [ -d "$PX4_WORLDS" ] && [ -f "$ARMS_SW/simulation/worlds/arms_sitl.sdf" ]; then
  cp "$ARMS_SW/simulation/worlds/arms_sitl.sdf" "$PX4_WORLDS/arms_sitl.sdf"
  echo "월드 복사: $PX4_WORLDS/arms_sitl.sdf"
fi

if command -v gnome-terminal >/dev/null; then
  TERM_CMD() { gnome-terminal --title="$1" -- bash -c "$2; exec bash"; }
elif command -v xterm >/dev/null; then
  TERM_CMD() { xterm -T "$1" -e bash -c "$2; exec bash" & }
else
  echo "gnome-terminal/xterm 없음"; exit 1
fi

echo "[1/2] PX4 SITL + Gazebo (arms_drone)..."
# Wayland 세션에서 gz sim 3D 뷰 마우스 입력이 막히는 문제 → Qt 를 X11(xcb)로 강제.
# (휠 줌 / 드래그 회전이 안 먹으면 대부분 이게 원인. gnome-terminal 자식이 부모 env 를
#  못 물려받는 경우가 있어 명령 문자열 안에서 직접 export.)
TERM_CMD "PX4 SITL" \
  "export QT_QPA_PLATFORM=xcb; cd $PX4_DIR && PX4_GZ_WORLD=arms_sitl make px4_sitl gz_arms_drone"

echo "      부팅 대기 (15s)..."
sleep 15

echo "[2/2] ARMS 스택 (브리지+제어+UI+검출+풍선+패널)..."
TERM_CMD "ARMS stack" \
  "$ROS_SETUP && ros2 launch arms_bringup arms_sitl.launch.py"

# 자동 disarm / 페일세이프 끄기 (GCS/RC 없는 SITL 자율비행 셋업).
#   COM_DISARM_PRFLT 0  = arm 후 이륙 지연 자동 disarm 해제 (가만히 둬도 안 멈춤)
#   NAV_DLL_ACT      0   = GCS(데이터링크) 연결 끊김 페일세이프 해제
#   NAV_RCL_ACT      0   = RC 연결 끊김 페일세이프 해제
sleep 5
SHELL_PY="$PX4_DIR/Tools/mavlink_shell.py"
if [ -f "$SHELL_PY" ]; then
  printf 'param set COM_DISARM_PRFLT 0\nparam set NAV_DLL_ACT 0\nparam set NAV_RCL_ACT 0\n' \
    | python3 "$SHELL_PY" udp:127.0.0.1:14550 >/dev/null 2>&1 &
  echo "      자동 disarm/페일세이프 해제 (COM_DISARM_PRFLT/LAND, NAV_DLL_ACT/RCL_ACT)"
fi

echo ""
echo "완료. 잠시 후 패널/영상 창이 뜬다."
echo "상태가 IDLE→SEARCH→LOCK→TRACK→FIRE 로 가면 성공."
echo "드론이 풍선 반대로 가면 패널 Roll/Pitch sign 버튼, 못 따라가면 상승추력 슬라이더 ↑"
