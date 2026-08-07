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

# gz in-process transport:
#   매 프레임 gz service CLI(≈0.38s/회)를 새로 띄우면 초당 ~2.6회밖에 못 보내 공이
#   뚝뚝 순간이동한다. 영속 transport 노드로 직접 서비스를 호출하면 수십 Hz 로 매끄럽게
#   pose 를 보낼 수 있다. 바인딩이 없으면 아래 set_pose 가 CLI 로 폴백한다.
try:
    from gz.transport13 import Node as _GzNode
    from gz.msgs10.pose_pb2 import Pose as _GzPose
    from gz.msgs10.boolean_pb2 import Boolean as _GzBool
    _gz_node = _GzNode()
    _GZ_OK = True
except Exception:
    _gz_node = None
    _GZ_OK = False

# 비행 패턴: 실제 적 드론처럼 상공에서 곡선 기동하며 가로지르고, 화면 밖이면 재등장
ALT = 42.0          # 기본 비행 고도 [m] (드론 도달가능·추락방지, 진짜요격)
# 광역 비행: 카메라 화각 밖 먼 지점에서 시작 → 대각선으로 시야 통과 → 반대편 먼 곳으로 이탈.
# 화각 제약 scale 을 제거하고 SPAN 을 기존(28) 대비 ~2.5배로 키움.
SPAN = 100.0         # 대각선 가로지르는 총 거리 [m] (먼곳→먼곳). x/y 각각 ±SPAN/2 이동
SPEED = 1.6        # 진행 속도 [m/s] (단일 속도, 패널 슬라이더로 실시간 조절)
RATE_HZ = 60.0      # 위치 갱신 주기
PAUSE_SEC = 1.5     # 한 번 지나간 뒤 재등장까지 대기 [s]
HIDE_Z = -100.0     # 숨길 때 보내는 지하 z [m] (화면에서 안 보임)


def set_pose(x, y, z):
    """모델을 지정 위치로 순간이동(teleport)시킨다.

    gz.transport 바인딩이 있으면 in-process 서비스 호출(빠름 → 부드러운 이동).
    없으면 gz service CLI 로 폴백(느림). 둘 다 블로킹 1회씩만 보내므로 큐 폭주 없음.
    """
    if _GZ_OK:
        req = _GzPose()
        req.name = MODEL
        req.position.x = float(x)
        req.position.y = float(y)
        req.position.z = float(z)
        # gz 미준비 등으로 실패해도 다음 프레임에 갱신되므로 결과는 무시.
        try:
            _gz_node.request(f"/world/{WORLD}/set_pose", req, _GzPose, _GzBool, 100)
        except Exception:
            pass
        return
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
        # enabled=false(기본) 면 풍선을 숨긴 채 대기. 패널 [공 발사] 버튼을 눌러야
        # 그제서야 먼 곳에서 등장해 비행 시작. (버튼 누르기 전엔 화면에 안 보임)
        self.declare_parameter("enabled", False)
        # alt = 비행 고도 [m] (패널 슬라이더로 실시간 조절)
        self.declare_parameter("alt", ALT)
        # speed = 진행 속도 [m/s] (단일, 패널 슬라이더로 실시간 조절)
        self.declare_parameter("speed", SPEED)
        self.hit = False
        self.t = 0.0                    # 대각선 진행도 [0→1]
        self.pause_until = 0.0          # 재등장 대기 종료 시각
        # _prev_enabled=True 로 시작 → 첫 tick(비활성)에서 공을 1회 숨겨 초기 스폰 위치를 치움.
        self._prev_enabled = True
        # 한 번이라도 발사됐는지. 발사 전(초기 대기)엔 숨기고, 발사 후 정지는 제자리 멈춤.
        self._launched_once = False
        self._new_pass()
        self.create_subscription(MissionState, "/arms/mission_state",
                                 self.cb_state, 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info(
            f"balloon_referee ready. 기본 대기(숨김) 상태. [공 발사] 누르면 "
            f"먼 곳({SPAN/2:.0f}m)에서 대각선으로 광역 비행 시작.")

    def _new_pass(self):
        """대각선: 진행도 t 리셋 (오른쪽아래->왼쪽위)."""
        self.t = 0.0

    def cb_state(self, msg: MissionState):
        if msg.state == "FIRE" and not self.hit:
            self.hit = True
            set_pose(0.0, 0.0, HIDE_Z)   # 풍선 제거(명중 연출)
            self.get_logger().info("🎯 명중! 잠시 후 재등장.")
            now = self.get_clock().now().nanoseconds * 1e-9
            self.pause_until = now + 3.0      # 명중 후 3초 뒤 재등장
            self._rearm_after = now + 1.0
            self._new_pass()

    def tick(self):
        enabled = self.get_parameter("enabled").value
        now = self.get_clock().now().nanoseconds * 1e-9

        # ---- 비활성 상태 처리 ----
        #   · 초기 대기(발사 전): 공을 지하로 숨겨 화면에 안 보이게.
        #   · 발사 후 정지: 현재 위치에 그대로 멈춤(일시정지). set_pose 를 안 보내면
        #     Gazebo 가 마지막 위치를 유지하므로 사라지지 않고 제자리에 정지.
        if not enabled:
            if self._prev_enabled and not self._launched_once:
                set_pose(0.0, 0.0, HIDE_Z)   # 발사 전 초기 스폰만 숨김
            self._prev_enabled = False
            return

        # ---- 비활성→활성 상승엣지(=비행시작/재개) ----
        #   정지했던 그 자리(t)에서 이어서 재개한다. (t 를 리셋하지 않음)
        #   최초 발사는 __init__ 에서 t=0 이라 자동으로 먼 시작점부터 시작.
        if not self._prev_enabled:
            self.hit = False
            self.pause_until = 0.0
            self._launched_once = True
            self.get_logger().info(f"▶ 풍선 재개 (진행도 t={self.t:.2f})")
        self._prev_enabled = True

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

        # 광역 대각선: 화각 제약 scale 없이 SPAN 전체를 이동.
        #   시작 x=+SPAN/2(화면 아래), y=-SPAN/2(화면 오른쪽) = 화면 오른쪽아래 먼 곳
        #   끝   x=-SPAN/2(화면 위),   y=+SPAN/2(화면 왼쪽)   = 화면 왼쪽위 먼 곳
        #   중간(t=0.5)에 (0,0) 카메라 중심을 통과 → 시야 관통.
        speed = self.get_parameter("speed").value   # 단일 속도 (패널 슬라이더)
        # 상공에서 대각선으로 시야 관통
        span = SPAN
        self.t += (speed / max(span, 0.1)) * (1.0 / RATE_HZ)
        f = 1.0 - 2.0 * self.t
        x = (span / 2.0) * f
        y = -(span / 2.0) * f
        z = alt
        set_pose(x, y, z)
        if self.t >= 1.0:
            set_pose(0.0, 0.0, HIDE_Z)       # 반대편 먼 곳 통과 후 숨김
            self.pause_until = now + PAUSE_SEC
            self._new_pass()                 # 재등장: 다시 먼 시작점부터


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
