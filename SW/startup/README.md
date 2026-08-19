# A.R.M.S. 실기체 부팅 자동 실행 (startup)

젯슨 전원을 켜면 실기체 런치(`ros2 launch arms_bringup arms.launch.py`)가
자동으로 뜨게 하는 설정이다.

```
SW/startup/
├── start_arms.sh            # 실제 기동 스크립트 (ROS/워크스페이스 source → 도커·장치 대기 → launch)
├── arms.service             # 방법 A: systemd 서비스 (권장, 자동 재시작·로그)
├── arms-autostart.desktop   # 방법 B: 데스크톱 autostart (가장 단순, GUI 세션에 자연스럽게 뜸)
└── README.md
```

두 방법 중 **하나만** 쓰면 된다. GUI(arms_ui OpenCV 창)가 모니터에 떠야 하므로
디스플레이 접근이 관건이다. 아래 참고.

---

## 0. 사전 준비 (필수)

부팅 자동실행 전에 아래가 되어 있어야 한다.

1. **워크스페이스 빌드**
   ```bash
   cd ~/A.R.M.S.-Anti-drone-Reusable-Modular-System-/SW/arms_ws
   colcon build
   ```
2. **detection 도커 이미지 빌드** (한 번)
   ```bash
   cd ~/A.R.M.S.-Anti-drone-Reusable-Modular-System-/SW/arms_ws/src/arms_detection/docker
   docker compose -f docker-compose.jetson.yml build
   ```
   - YOLO 엔진(`models/balloon.engine` 등)은 **이 젯슨에서 빌드된 것**이어야 한다
     (`SW/YOLO_balloon/export_trt.py` 참고). compose 기본값은 `/models/balloon.engine`.
3. **스크립트 실행권한**
   ```bash
   chmod +x ~/A.R.M.S.-Anti-drone-Reusable-Modular-System-/SW/startup/start_arms.sh
   ```
4. **장치·권한** — CRSF UART(`/dev/ttyTHS1`, nvgetty 비활성), 서보 PWM(핀15 pinmux +
   `/sys/class/pwm` 권한), ESP32 시리얼(dialout), 카메라(`/dev/video0`). 각 하위
   README(서보/CRSF) 참고.
5. **경로 확인** — 리포가 위 경로와 다르면 `start_arms.sh`의 `ARMS_ROOT`,
   `arms.service`/`arms-autostart.desktop`의 `Exec` 경로를 **본인 경로로 수정**.

---

## 방법 A — systemd 서비스 (권장)

자동 재시작·`journalctl` 로그가 되어 실기체에 적합. 단 GUI라서 **DISPLAY 지정**이 필요.

1. **DISPLAY 확인** (데스크톱 로그인 상태의 터미널에서):
   ```bash
   echo $DISPLAY        # 예: :0  또는 :1
   ls ~/.Xauthority     # XAUTHORITY 경로 확인
   ```
2. `arms.service`의 `Environment=DISPLAY=...` 값을 위에서 확인한 값으로 수정
   (그리고 `User=`, `ExecStart=` 경로가 맞는지 확인).
3. 설치·활성화:
   ```bash
   sudo cp ~/A.R.M.S.-Anti-drone-Reusable-Modular-System-/SW/startup/arms.service \
           /etc/systemd/system/arms.service
   sudo systemctl daemon-reload
   sudo systemctl enable arms.service     # 부팅 시 자동 시작 등록
   sudo systemctl start arms.service      # 지금 바로 시작(테스트)
   ```
4. 상태·로그:
   ```bash
   systemctl status arms.service
   journalctl -u arms.service -f          # 실시간 로그
   ```
5. 중지·해제:
   ```bash
   sudo systemctl stop arms.service       # 지금 중지
   sudo systemctl disable arms.service    # 부팅 자동시작 해제
   ```

> GUI가 안 뜨면 대개 DISPLAY/XAUTHORITY 문제다. 자동로그인이 켜져 있어야 부팅 시
> X 세션이 떠서 창이 보인다(설정 → 사용자 → 자동 로그인). 자동로그인이 없으면
> 방법 B를 쓰거나, 로그인 후 서비스가 뜨도록 조정한다.

---

## 방법 B — 데스크톱 autostart (가장 단순, GUI 친화)

데스크톱 로그인 세션에서 실행되므로 DISPLAY가 자동으로 잡힌다. 자동로그인 + 키오스크
용도로 가장 간단하다. (단 재시작/journald 로그는 없음 — 로그는 `~/arms_logs/`에 파일로 남음)

```bash
mkdir -p ~/.config/autostart
cp ~/A.R.M.S.-Anti-drone-Reusable-Modular-System-/SW/startup/arms-autostart.desktop \
   ~/.config/autostart/arms-autostart.desktop
```
- 해제: `rm ~/.config/autostart/arms-autostart.desktop`
- **자동 로그인**을 켜 둬야 부팅 시 세션이 떠서 실행된다.

---

## 동작 / 로그

- `start_arms.sh`는 ROS·워크스페이스를 source 하고, **도커 데몬(최대 60s)** 과
  **USB 장치(기본 5s)** 를 기다린 뒤 런치한다.
- 로그: `~/arms_logs/arms_startup_YYYYmmdd_HHMMSS.log` (+ systemd는 journald).
- 환경변수로 조정: `ARMS_ROOT`, `ROS_DOMAIN_ID`, `ARMS_DEVICE_WAIT`, `ARMS_LOG_DIR`.

## 실행 중지 (뜬 것 끄기)

autostart(방법 B)는 `Terminal=false` 라 창에서 Ctrl-C 가 안 된다. 터미널에서 런치를 끈다:

```bash
pkill -INT -f "ros2 launch arms_bringup"
```
→ video·command·control·ui 노드가 정리된다.

검출 **도커 컨테이너는 별도**(`docker compose up -d` 로 뜸)라 위 명령으로 안 꺼진다. 같이 내리려면:

```bash
docker stop docker-arms_detection-1
```

한 번에 다 끄기:

```bash
pkill -INT -f "ros2 launch arms_bringup"; docker stop docker-arms_detection-1
```

> systemd(방법 A)로 띄웠다면: `sudo systemctl stop arms.service`.
> UI 우상단 **OFF 버튼은 젯슨 전원 자체를 끔**(poweroff) — 런치만 끄는 것과 다르다.
> **다음 부팅부터 자동실행 자체를 끄려면**: `rm ~/.config/autostart/arms-autostart.desktop`.

## 먼저 수동 테스트

부팅 자동화 전에 스크립트가 잘 뜨는지 확인:
```bash
~/A.R.M.S.-Anti-drone-Reusable-Modular-System-/SW/startup/start_arms.sh
```
UI 창이 뜨고 노드들이 올라오면 성공. Ctrl-C 로 종료.

## ⚠️ 안전

- 부팅 자동실행이면 **켜자마자 CRSF 출력이 FC로 나가고 서보가 동작**한다(IDLE=열림).
- 다만 **arm 재토글 안전장치**로 부팅 시엔 스위치가 arm이어도 **disarm으로 시작**한다
  (DISARM→ARM 재토글 + 수동은 스틱 idle 이어야 arm). 그래도 첫 전원 인가 시 프로펠러
  제거 등 기본 안전수칙을 지킬 것.
- 문제 시 즉시 멈추려면: `pkill -INT -f "ros2 launch arms_bringup"`
  (systemd 방식은 `sudo systemctl stop arms.service`). 자세한 건 위 "실행 중지" 절.
