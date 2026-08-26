# detection 컨테이너 발견(discovery) 워치독

detection 도커 컨테이너는 **Up 인데 호스트에서 `/arms/detections` 발행이 안 보이는**
(FastDDS 발견 실패) 문제를 자동 감지·복구한다. 그때 컨테이너를 재시작하면 participant 가
다시 announce 되며 복구되므로, 그걸 사람이 안 봐도 되게 자동화한다.

## 왜 생기나
- 컨테이너는 `network_mode: host` 이지만 **UDP-only 프로파일**(`fastdds_udp.xml`)을 쓰고,
  호스트 노드는 **기본 transport(SHM+UDP)** 를 쓴다 → transport 가 어긋난 상태에서
  **멀티캐스트 발견**이 여러 인터페이스(eth/wlan/docker0/lo)로 엇갈려 간헐적으로 실패한다.
- 증상: 호스트에서 `ros2 topic info /arms/detections` → **Publisher count: 0**
  (구독자는 있는데 컨테이너 발행자만 안 보임). `ros2 node list` 에 detection 노드 없음.
- 근본 해결은 FastDDS **Discovery Server** 도입이지만(전체 노드 env + 서버 프로세스 필요),
  이 워치독은 **재시작만으로 확실히 복구되는 점**을 이용한 실용적 자동복구다.

## 판정 로직
`/arms/detections` 의 **구독자 ≥ 1(=시스템 가동중) 이면서 발행자 = 0** 이 `CONFIRM` 회
연속되면 컨테이너만 발견 실패로 보고 재시작한다. (전체 시스템이 꺼진 상태에선 재시작 안 함.)

## 파일
- `arms-detection-watchdog.sh` — 워치독 본체(호스트에서 실행, ROS·docker 필요)
- `arms-detection-watchdog.service` — systemd **유저** 서비스

## 설치
```bash
install -m 755 arms-detection-watchdog.sh ~/.local/bin/arms-detection-watchdog.sh
mkdir -p ~/.config/systemd/user
cp arms-detection-watchdog.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now arms-detection-watchdog.service
```

## 확인 / 로그 / 중지
```bash
systemctl --user status arms-detection-watchdog.service     # 가동 여부
journalctl --user -u arms-detection-watchdog.service -f     # 재시작 이력
systemctl --user disable --now arms-detection-watchdog.service   # 끄기
```

## 튜닝 (환경변수)
| 변수 | 기본 | 설명 |
| --- | --- | --- |
| `ARMS_WD_INTERVAL` | 20 | 점검 주기[s] |
| `ARMS_WD_CONFIRM` | 3 | 연속 실패 확인 횟수(오탐 방지) |
| `ARMS_WD_RECOVER` | 20 | 재시작 후 안정화 대기[s] |
| `ARMS_DET_CONTAINER` | `docker-arms_detection-1` | 대상 컨테이너 이름 |
| `ARMS_WS` | (레포 경로)/SW/arms_ws | 워크스페이스 |

> 자동 로그인 유저 세션에서 도는 유저 서비스라 `default.target` 에 등록된다.
> 그래픽 세션 없이도 돌게 하려면 `loginctl enable-linger $USER`.
