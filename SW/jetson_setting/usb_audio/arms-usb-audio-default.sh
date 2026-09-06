#!/bin/bash
# USB 스피커(C-Media)를 PulseAudio 기본 싱크로 강제한다.
#
# 문제: USB 오디오가 느린 공유 USB2 허브에 물려 있어 부팅 시 PulseAudio 시작보다
#   늦게 잡힌다. switch-on-connect 는 '부팅 초기 등장' 장치엔 안 걸리고 수동 재플러그
#   (핫플러그)에만 걸려서, 부팅 후엔 기본 싱크가 내장(platform-sound)으로 남아 무음.
# 해결: 로그인 후 이 스크립트가 USB 싱크가 나타날 때까지 최대 60초 기다렸다가
#   기본 싱크로 지정하고, 이미 재생 중인 스트림도 그쪽으로 옮긴다.

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

# 장치명 기반이라 /dev/videoN 번호처럼 바뀌지 않는다(USB 재열거에도 안정).
SINK_MATCH="usb-C-Media_Electronics_Inc._USB_Audio_Device"

for _ in $(seq 1 60); do
    sink="$(pactl list short sinks 2>/dev/null | awk '{print $2}' | grep -m1 "$SINK_MATCH")"
    if [ -n "$sink" ]; then
        pactl set-default-sink "$sink"
        # 이미 열린 스트림(부팅 효과음 등)도 USB 로 이동.
        for si in $(pactl list short sink-inputs 2>/dev/null | awk '{print $1}'); do
            pactl move-sink-input "$si" "$sink" 2>/dev/null || true
        done
        echo "[arms-usb-audio] default sink → $sink"
        exit 0
    fi
    sleep 1
done

echo "[arms-usb-audio] USB 싱크를 60초 내 못 찾음 — 스킵" >&2
exit 0
