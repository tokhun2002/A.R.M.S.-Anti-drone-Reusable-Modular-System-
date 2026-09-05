# bag_to_video

UI 화면 스트림(`/arms/ui_image/compressed`)을 rosbag 으로 녹화하고, 그 bag 을
**mp4(H.264) 동영상으로 변환**하는 도구.

## 배경
- UI 노드는 화면을 `/arms/ui_image`(raw) + `/arms/ui_image/compressed`(JPEG)로 발행한다.
- **compressed** 를 녹화하면 경량이라 오래 저장/전송하기 좋다.
- rqt 는 압축 토픽을 직접 표시 못 하므로, 볼 때는 raw 로 풀거나(재생) mp4 로 변환한다.

## bag_to_mp4.py
rosbag 안의 `CompressedImage`(JPEG) 프레임을 **ffmpeg 로 파이프해 H.264 mp4** 로 만든다.
- 재생(`ros2 bag play`) 없이 bag 을 직접 읽어 변환 → 라이브 토픽과 충돌 없음.
- fps 는 bag 타임스탬프에서 자동 계산.
- OpenCV VideoWriter 의 `mp4v`(mpeg4, 일부 플레이어 재생 불가) 대신 **libx264 + yuv420p**
  로 인코딩해 브라우저·폰·기본 플레이어 어디서나 재생된다.

필요: `ffmpeg`(libx264), `ros2`(rosbag2 python).

```bash
# 기본: <bag>.mp4 생성
python3 SW/scripts/bag_to_video/bag_to_mp4.py ui_rec

# 옵션
python3 SW/scripts/bag_to_video/bag_to_mp4.py ui_rec --out my_flight.mp4
python3 SW/scripts/bag_to_video/bag_to_mp4.py ui_rec --fps 15          # fps 강제(기본 자동)
python3 SW/scripts/bag_to_video/bag_to_mp4.py ui_rec --storage mcap    # mcap 으로 녹화한 경우
python3 SW/scripts/bag_to_video/bag_to_mp4.py ui_rec --topic /arms/ui_image/compressed
```

## 녹화 / 재생 (참고)
```bash
# 녹화 (compressed, 경량)
ros2 bag record -o ui_rec /arms/ui_image/compressed

# 재생하며 화면으로 보기 (compressed → raw 로 풀어서 rqt)
ros2 bag play ui_rec --loop
ros2 run image_transport republish compressed raw \
  --ros-args -r in/compressed:=/arms/ui_image/compressed -r out:=/arms/ui_view
ros2 run rqt_image_view rqt_image_view /arms/ui_view
```

전체 절차는 `SW/docs/SETUP.md` 의 "UI 화면 녹화 / 재생 / mp4 변환" 절 참고.
