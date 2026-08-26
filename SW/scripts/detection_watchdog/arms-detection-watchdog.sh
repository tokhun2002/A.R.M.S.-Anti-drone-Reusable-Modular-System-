#!/bin/bash
# arms-detection-watchdog — detection 도커 컨테이너는 떠 있는데 호스트에서 발행이
#   안 보이는(FastDDS 발견 실패) 상태를 감지해 컨테이너를 자동 재시작한다.
#
# 배경: 컨테이너는 network_mode: host 이지만 UDP-only 프로파일을 쓰고 호스트 노드는
#   기본 transport(SHM+UDP)를 써서, 멀티캐스트 발견이 간헐적으로 어긋난다. 그러면
#   "컨테이너는 Up 인데 /arms/detections 발행자가 호스트에 0" 이 된다. 재시작하면
#   participant 가 다시 announce 되며 복구되므로, 그걸 자동화한다.
#
# 판정: /arms/detections 에 **호스트 구독자는 있는데(=시스템 가동중) 발행자는 0** 이면
#   컨테이너만 발견 실패로 보고 재시작. (전체 시스템이 꺼진 상태에선 재시작 안 함.)

# ROS/ament setup 스크립트는 nounset-clean 이 아니라 `set -u` 하에서 죽는다 →
# 소싱을 먼저 하고 그 다음에 set -u 를 켠다.
source /opt/ros/humble/setup.bash 2>/dev/null || true
WS="${ARMS_WS:-/home/arms/A.R.M.S.-Anti-drone-Reusable-Modular-System-/SW/arms_ws}"
source "$WS/install/setup.bash" 2>/dev/null || true
set -u

CONTAINER="${ARMS_DET_CONTAINER:-docker-arms_detection-1}"
CHECK_INTERVAL="${ARMS_WD_INTERVAL:-20}"    # 점검 주기[s]
CONFIRM="${ARMS_WD_CONFIRM:-3}"             # 연속 실패 확인 횟수(오탐 방지)
RECOVER_WAIT="${ARMS_WD_RECOVER:-20}"       # 재시작 후 안정화 대기[s]

topic_state() {   # echo "subs pubs"
  local info subs pubs
  info=$(timeout 6 ros2 topic info /arms/detections 2>/dev/null)
  subs=$(echo "$info" | grep -oP 'Subscription count: \K[0-9]+'); subs=${subs:-0}
  pubs=$(echo "$info" | grep -oP 'Publisher count: \K[0-9]+');    pubs=${pubs:-0}
  echo "$subs $pubs"
}

echo "[watchdog] 시작 (컨테이너=$CONTAINER, 주기=${CHECK_INTERVAL}s, 확인=${CONFIRM}회)"
while true; do
  sleep "$CHECK_INTERVAL"
  # 컨테이너가 안 떠 있으면(사용자가 껐거나 다른 문제) 건드리지 않는다.
  docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || continue

  fails=0
  for ((i=0; i<CONFIRM; i++)); do
    read -r subs pubs < <(topic_state)
    if [ "$subs" -ge 1 ] && [ "$pubs" -eq 0 ]; then
      fails=$((fails+1)); sleep 3
    else
      fails=0; break
    fi
  done

  if [ "$fails" -ge "$CONFIRM" ]; then
    echo "[watchdog] $(date '+%F %T') detection 발행 미검출(subs>0, pubs=0) → $CONTAINER 재시작"
    docker restart "$CONTAINER" >/dev/null 2>&1
    sleep "$RECOVER_WAIT"
  fi
done
