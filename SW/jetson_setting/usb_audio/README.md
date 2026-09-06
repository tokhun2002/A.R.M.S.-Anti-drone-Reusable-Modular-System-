# USB 스피커 기본 출력 고정 (부팅 레이스 우회)

Jetson에 USB 스피커(C-Media USB Audio)를 연결했는데 **부팅 후엔 소리가 안 나고,
USB를 뽑았다 다시 끼면 소리가 나는** 문제를 해결한다.

## 증상과 원인

- 프로그램 효과음/비프는 `ffplay`(SDL) → **PulseAudio 기본 싱크**로 나간다.
- 부팅 시 PulseAudio 기본 싱크가 **USB 스피커가 아니라 내장(`platform-sound`)** 으로
  잡혀서, 소리가 스피커 없는 내장 출력으로 빠진다 → 무음.
- USB 오디오가 **느린 공유 USB2 허브**(영상 캡처 동글·기타 장치와 같은 허브)에 물려 있어
  PulseAudio 시작보다 **늦게** 잡힌다.
- PulseAudio `module-switch-on-connect`는 **부팅 초기에 등장한 장치엔 안 걸리고, 부팅 후
  수동 재플러그(핫플러그)에만** 걸린다. → 그래서 재플러그하면 그때만 기본이 USB로 바뀌어
  소리가 났던 것. `module-default-device-restore`(저장된 기본값 복원)도 이 레이스에 밀린다.

## 해결

로그인 후 **USB 싱크가 나타날 때까지 최대 60초 대기했다가 기본 싱크로 강제**하는
systemd **유저 서비스**를 실행한다. 대기 재시도가 있어 PA/USB 준비 타이밍이 어긋나도 안전하다.

- `arms-usb-audio-default.sh` — 대기·설정 스크립트. USB 싱크를 **장치명 기반**으로 찾으므로
  (`usb-C-Media_Electronics_Inc._USB_Audio_Device`) USB 재열거로 번호가 바뀌어도 안정.
  기본 싱크 지정 + 이미 열린 스트림(부팅 효과음 등)도 USB로 이동.
- `arms-usb-audio-default.service` — 위 스크립트를 `default.target`(그래픽 세션)에서 oneshot 실행.

> 스피커가 다른 USB 오디오면 스크립트의 `SINK_MATCH` 를 해당 장치명으로 바꾼다.
> 이름 확인: `pactl list short sinks`

## 설치

```bash
# 1) 스크립트 배치 (실행권한 포함)
mkdir -p ~/.local/bin
install -m 755 arms-usb-audio-default.sh ~/.local/bin/arms-usb-audio-default.sh

# 2) 유저 서비스 배치 + 등록
mkdir -p ~/.config/systemd/user
cp arms-usb-audio-default.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now arms-usb-audio-default.service
```

## 검증

```bash
# 부팅 상황 재현: 기본을 내장으로 되돌린 뒤 서비스 실행 → USB 로 바뀌면 정상
pactl set-default-sink alsa_output.platform-sound.analog-stereo
systemctl --user start arms-usb-audio-default.service
pactl info | grep 'Default Sink'   # ...usb-C-Media... 이어야 함
```

가장 확실한 검증은 **재부팅 후 재플러그 없이 소리가 나는지** 확인.

## 문제 해결

```bash
systemctl --user status arms-usb-audio-default.service   # 언제 돌았는지
```

- 그래도 무음이면 유저 매니저가 부팅 시 안 뜨는 경우일 수 있다(자동 로그인 아님):
  `loginctl enable-linger $USER` 로 linger 를 켠다(단, PA 가 세션에 묶여 있으면 오히려
  안 뜰 수 있으니 자동 로그인 환경에선 불필요).

## 수동 즉시 전환

```bash
pactl set-default-sink alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo
```

## 제거

```bash
systemctl --user disable --now arms-usb-audio-default.service
rm ~/.config/systemd/user/arms-usb-audio-default.service ~/.local/bin/arms-usb-audio-default.sh
systemctl --user daemon-reload
```
