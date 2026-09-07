# bag_visualize

비행 로그 rosbag 을 **CSV + 오버뷰 그래프 + mp4 영상**으로 한 번에 변환하는 도구.
(`auto_save:=true` 로 녹화한 `~/arms_flight_log/<일시>/` bag 을 분석/시각화)

## 한 방에 (마스터 스크립트)

```bash
python3 SW/scripts/bag_visualize/bag_visualize.py ~/arms_flight_log/20260907_1530
```

결과가 `<bag>_viz/` 아래에 모인다:

```
<bag>_viz/
  csv/
    arms_detections_raw.csv   # KF 적용 전 원시 검출(측정값)
    arms_detections.csv       # KF 적용값(최종 표적)
    arms_crsf_rx.csv          # 수신 텔레메트리(배터리/링크/자세 roll·pitch·yaw)
    arms_crsf_tx.csv          # 송신 CRSF 채널(roll/pitch/thr/yaw/arm/mode/kill)
    arms_mission_state.csv, arms_detector_status.csv, arms_command.csv ...
  target_plane.png            # 화면(정규화 좌표) 상 표적 위치 — raw vs KF
  attitude_cmd.png            # roll/pitch/yaw 자세 + 제어명령 6개 시계열
  <bag>.mp4                    # UI 화면 영상(H.264)
```

옵션:

```bash
--outdir DIR        # 결과 폴더 (기본 <bag>_viz)
--storage mcap|sqlite3   # 기본: bag/metadata.yaml 에서 자동감지
--video-topic /arms/ui_image/compressed
--fps N             # 영상 fps (0=자동)
--aspect 16:9       # target_plane 화면비
--no-video / --no-csv / --no-plots   # 단계 건너뛰기
```

## 그래프

- **target_plane.png** — 정규화 좌표(0~1)의 표적 위치를 화면비(기본 16:9)로 그린다.
  `detections_raw`(주황 점)와 KF 적용값(`detections`, 파란 선+점)을 함께 → KF 평활/외삽
  효과를 한눈에. y축은 이미지 좌표(위=0)로 뒤집혀 있고 중앙(0.5,0.5) 십자선 표시.
- **attitude_cmd.png** — 2×3 subplot 6개.
  - 1행: 자세(수신) `roll_deg` / `pitch_deg` / `yaw_deg` (`/arms/crsf_rx`)
  - 2행: 제어명령(송신) roll / pitch / yaw — CRSF 채널(CH1/CH2/CH4)을 [-1,1]로 정규화
    (`/arms/crsf_tx`)

## 개별 스크립트 (따로도 실행 가능)

```bash
# 1) bag → CSV
python3 bag_to_csv.py <bag> [--storage ...] [--outdir DIR]

# 2) CSV → 그래프 (ROS 불필요, CSV 만 있으면 어디서나)
python3 plot_overview.py <csv_dir> [--outdir DIR] [--aspect 16:9]

# 3) bag → mp4 (UI 영상)
python3 bag_to_mp4.py <bag> [--topic ...] [--out ...] [--fps N] [--storage ...]
```

`bag_to_csv.py` 는 영상 토픽(CompressedImage/Image)을 빼고 나머지 메시지를 평탄화해
토픽별 CSV 로 쓴다. 각 행은 `t_ns`(원본 수신시각), `t_rel`(첫 메시지 기준 초)로 시작한다.

## 필요 패키지

| 단계 | 필요 |
| ---- | ---- |
| CSV (`bag_to_csv`) | `ros2`(rosbag2 python) |
| 그래프 (`plot_overview`) | `matplotlib`, `pandas` (ROS 불필요) |
| 영상 (`bag_to_mp4`) | `ffmpeg`(libx264), `ros2` |

```bash
pip3 install matplotlib pandas
sudo apt install ffmpeg
```

## 녹화 (참고)

`arms.launch.py auto_save:=true` 로 자동 녹화되며, 수동으로는:

```bash
ros2 bag record --storage mcap -o run1 \
  /arms/ui_image/compressed /arms/detections_raw /arms/detections \
  /arms/crsf_tx /arms/crsf_rx /arms/mission_state /arms/command
```

전체 절차는 `SW/docs/SETUP.md` 의 "UI 화면 녹화 / 재생 / mp4 변환" 절 참고.
