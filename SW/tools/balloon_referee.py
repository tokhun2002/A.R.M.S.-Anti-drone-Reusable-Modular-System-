#!/usr/bin/env python3
"""
balloon_referee.py — 풍선을 "비행"시키고 "명중"을 판정/연출하는 SITL 심판 노드

하는 일:
  1) 타이머로 red_ball 모델 위치를 gz set_pose 서비스로 갱신 → 풍선이 하늘에서 떠다님
  2) /arms/mission_state 를 구독해서 상태가 FIRE 가 되면
     풍선을 멀리(지하)로 치워서 "명중(터짐)" 연출 + 로그 출력
  3) (옵션) 드론-풍선 거리 추정은 control 쪽 ray 센서가 담당하므로 여기선 연출만 함

전제:
  - gz (Gazebo Harmonic) CLI 가 PATH 에 있어야 함  (`gz service` 사용)
  - 월드 이름 = arms_sitl, 모델 이름 = red_ball

실행:
  source /opt/ros/humble/setup.bash
  source <arms_ws>/install/setup.bash
  python3 balloon_referee.py
"""

import math
import random
import subprocess
import rclpy
from rclpy.node import Node
from arms_msgs.msg import MissionState

WORLD = "arms_sitl"
MODEL = "red_ball"

# 비행 패턴: 실제 적 드론처럼 상공에서 곡선 기동하며 가로지르고, 화면 밖이면 재등장
ALT = 50.0          # 기본 비행 고도 [m] (높은 상공)
ALT_VAR = 8.0       # 고도 출렁임 폭 [m] (오르락내리락 → 드론 같은 움직임)
SPAN = 28.0         # 가로지르는 거리 [m] (고도 30m 기준, 낮으면 비례 축소)
CROSS = 8.0         # 가로(y) 오프셋 폭 [m]
ALT_REF = 30.0      # 위 SPAN/CROSS/WEAVE 가 설정된 기준 고도 [m]
FOV_FRAC = 0.45     # 화각 안에 두는 비율 (가로 반경 ≈ 고도 × 이 값). 작을수록 더 가운데로
SPEED = 6.0         # 진행 속도 [m/s]
WEAVE = 4.0         # 좌우 위빙 폭 [m] (직선 아니라 흔들며 비행)
WEAVE_HZ = 0.4      # 위빙 주파수 [Hz]
BOB_HZ = 0.3        # 고도 출렁임 주파수 [Hz]
RATE_HZ = 30.0      # 위치 갱신 주기
PAUSE_SEC = 1.5     # 한 번 지나간 뒤 재등장까지 대기 [s]


def set_pose(x, y, z):
    """gz set_pose 서비스로 모델을 순간이동(teleport)시킨다.

    blocking(run) 방식: 한 번에 하나씩만 보내고 응답을 기다린다.
    Popen 으로 비동기로 쏟으면 gz 서비스 큐가 터져서 Gazebo 가 죽으므로 금지.
    """
    req = f'name: "{MODEL}", position: {{x: {x}, y: {y}, z: {z}}}'
    subprocess.run(
        ["gz", "service", "-s", f"/world/{WORLD}/set_pose",
         "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
         "--timeout", "300", "--req", req],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


class BalloonReferee(Node):
    def __init__(self):
        super().__init__("balloon_referee")
        # enabled=false 면 자동 비행 정지 (패널에서 수동 공 생성 시 사용)
        self.declare_parameter("enabled", True)
        # alt = 비행 고도 [m] (패널 슬라이더로 실시간 조절)
        self.declare_parameter("alt", ALT)
        self.hit = False
        self.pos = -SPAN / 2.0          # 가로지르는 진행 위치 [m]
        self.cross = 0.0                # 이번 패스의 y 오프셋
        self.heading = 1.0              # +1: 왼→오, -1: 오→왼 (번갈아)
        self.pause_until = 0.0          # 재등장 대기 종료 시각
        self.phase = 0.0                # 위빙/고도 위상
        self._new_pass()
        self.create_subscription(MissionState, "/arms/mission_state",
                                 self.cb_state, 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info(
            f"balloon_referee ready. 고도 ~{ALT}m 상공에서 적 드론처럼 "
            f"곡선 기동(위빙+고도변화)하며 비행, 화면 밖이면 재등장.")

    def _new_pass(self):
        """새 패스: 가로(y) 오프셋 랜덤, 방향 번갈아. 고도에 맞춰 반경 축소."""
        self.heading *= -1.0
        try:
            alt = self.get_parameter("alt").value
        except Exception:
            alt = ALT
        scale = max(0.15, min(1.0, alt / ALT_REF))
        span = SPAN * scale
        cross = CROSS * scale
        self.pos = -self.heading * span / 2.0   # 진행 방향 반대편 끝에서 시작
        self.cross = random.uniform(-cross, cross)

    def cb_state(self, msg: MissionState):
        if msg.state == "FIRE" and not self.hit:
            self.hit = True
            set_pose(0.0, 0.0, -100.0)   # 풍선 제거(명중 연출)
            self.get_logger().info("🎯 명중! 잠시 후 재등장.")
            now = self.get_clock().now().nanoseconds * 1e-9
            self.pause_until = now + 3.0      # 명중 후 3초 뒤 재등장
            self._rearm_after = now + 1.0
            self._new_pass()

    def tick(self):
        # enabled=false 면 자동 비행 멈춤 (패널이 수동으로 공 위치 제어)
        if not self.get_parameter("enabled").value:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        # 명중 후 일정 시간 지나면 다시 추적 가능하게
        if self.hit:
            if now >= getattr(self, "_rearm_after", 0.0):
                self.hit = False
            else:
                return

        # 재등장 대기 중이면 풍선 숨김
        if now < self.pause_until:
            return

        alt = self.get_parameter("alt").value   # 패널 슬라이더로 바뀌는 고도

        # 고도에 따라 비행 반경 스케일: 카메라 화각 안에 들어오게.
        #   가까이(저고도) = 좁게(드론 주변만), 멀리(고고도) = 넓게.
        #   화면 안 가로반경 ≈ alt × FOV_FRAC. 기준고도 대비 비율로 SPAN/CROSS/WEAVE 축소.
        scale = max(0.15, min(1.0, (alt * FOV_FRAC) / (ALT_REF * FOV_FRAC)))
        span  = SPAN * scale
        cross = CROSS * scale
        weave_amp = WEAVE * scale

        # 진행 (스케일된 span 기준 속도도 비례시켜 너무 빠르지 않게)
        self.pos += self.heading * SPEED * scale * (1.0 / RATE_HZ)
        self.phase += 1.0 / RATE_HZ

        # 적 드론처럼: 직선이 아니라 좌우 위빙 + 고도 출렁임
        weave = weave_amp * math.sin(2.0 * math.pi * WEAVE_HZ * self.phase)
        bob   = min(ALT_VAR, alt * 0.25) * math.sin(2.0 * math.pi * BOB_HZ * self.phase)
        x = self.pos
        # cross 오프셋도 스케일 범위로 제한 (저고도면 가운데로)
        y = max(-cross, min(cross, self.cross)) + weave
        z = alt + bob
        set_pose(x, y, z)

        # 끝까지 가로질렀으면(=화면 밖) 잠깐 쉬고 반대편에서 재등장
        if abs(self.pos) > span / 2.0:
            set_pose(0.0, 0.0, -100.0)   # 일단 숨김
            self.pause_until = now + PAUSE_SEC
            self._new_pass()


def main():
    rclpy.init()
    node = BalloonReferee()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
