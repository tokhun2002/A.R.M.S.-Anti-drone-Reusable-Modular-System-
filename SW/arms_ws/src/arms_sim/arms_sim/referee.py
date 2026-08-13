#!/usr/bin/env python3
"""
referee.py — 표적(풍선/드론)을 "비행"시키고 "명중"을 판정/연출하는 SITL 심판 노드  [arms_sim]

하는 일:
  1) 타이머로 target_ball 모델 위치를 gz set_pose 서비스로 갱신 → 풍선이 하늘에서 떠다님
  2) /arms/mission_state 를 구독해서 상태가 FIRE 가 되면
     풍선을 멀리(지하)로 치워서 "명중(터짐)" 연출 + 로그 출력
  3) (옵션) 드론-풍선 거리 추정은 control 쪽 ray 센서가 담당하므로 여기선 연출만 함

전제:
  - gz (Gazebo Harmonic) CLI 가 PATH 에 있어야 함  (`gz service` 사용)
  - 월드 이름 = arms_sitl, 모델 이름 = target_ball

실행: ros2 run arms_sim referee   (node name = balloon_referee)
"""

import math
import os
import random
import subprocess

# gz.msgs10 의 _pb2 파일은 protobuf 3 세대 protoc 로 생성된 것인데, 이 시스템의
# protobuf 는 7.x 라 기본(upb) 구현에서 "Descriptors cannot be created directly"
# 로 import 자체가 터진다. 순수 파이썬 구현으로 내리면 그대로 동작한다.
#   ★ google.protobuf 가 처음 import 되기 전에 설정돼야 한다 → 파일 최상단.
#   증상: 이 줄이 없으면 gz 바인딩이 통째로 죽어서 set_pose 가 CLI 폴백
#         (≈0.38s/회 → 공이 뚝뚝 끊김) 이 되고 접촉 판정이 아예 안 온다.
#   대안은 protobuf 다운그레이드지만 시스템 전역을 건드리므로 여기서 격리한다.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

WORLD = "arms_sitl"
MODEL_BALLOON = "target_ball"   # 표적 A: 풍선(빨간 구)
MODEL_DRONE = "target_uav"      # 표적 B: 적 드론(x500 외형). 이름에 drone/x500 없음 → 자기드론(arms_drone) 식별과 안 겹침
MODEL = MODEL_BALLOON        # (하위호환/기본)

# gz in-process transport:
#   매 프레임 gz service CLI(≈0.38s/회)를 새로 띄우면 초당 ~2.6회밖에 못 보내 공이
#   뚝뚝 순간이동한다. 영속 transport 노드로 직접 서비스를 호출하면 수십 Hz 로 매끄럽게
#   pose 를 보낼 수 있다. 바인딩이 없으면 아래 set_pose 가 CLI 로 폴백한다.
_GZ_IMPORT_ERR = ""
try:
    from gz.transport13 import Node as _GzNode
    from gz.msgs10.pose_pb2 import Pose as _GzPose
    from gz.msgs10.boolean_pb2 import Boolean as _GzBool
    from gz.msgs10.pose_v_pb2 import Pose_V as _GzPoseV
    from gz.msgs10.contacts_pb2 import Contacts as _GzContacts
    _gz_node = _GzNode()
    _GZ_OK = True
except Exception as _e:                     # 무엇이 왜 실패했는지 남긴다
    _GZ_IMPORT_ERR = f"{type(_e).__name__}: {_e}"
    _gz_node = None
    _GzPose = _GzBool = _GzPoseV = _GzContacts = None
    _GZ_OK = False

# 비행 패턴: 실제 적 드론처럼 상공에서 곡선 기동하며 가로지르고, 화면 밖이면 재등장
ALT = 42.0          # 기본 비행 고도 [m] (드론 도달가능·추락방지, 진짜요격)
# 광역 비행: 카메라 화각 밖 먼 지점에서 시작 → 대각선으로 시야 통과 → 반대편 먼 곳으로 이탈.
# 화각 제약 scale 을 제거하고 SPAN 을 기존(28) 대비 ~2.5배로 키움.
SPAN = 100.0         # 대각선 가로지르는 총 거리 [m] (먼곳→먼곳). x/y 각각 ±SPAN/2 이동
SPEED = 3.6        # 진행 속도 [m/s] (3.6m/s 명중 검증값, 패널 슬라이더로 실시간 조절)
RATE_HZ = 60.0      # 위치 갱신 주기
PAUSE_SEC = 1.5     # 한 번 지나간 뒤 재등장까지 대기 [s]
HIDE_Z = -100.0     # 숨길 때 보내는 지하 z [m] (화면에서 안 보임)
# 직격 판정은 Gazebo 접촉 센서가 한다 (world arms_sitl.sdf 의 contact_sensor).
#   예전에는 여기서 중심거리 < BALLOON_RADIUS + DRONE_RADIUS 로 쟀는데, 그
#   DRONE_RADIUS=0.3 은 비구형 드론을 구로 근사한 손어림값이었고 패널 슬라이더로
#   10 m 까지 올릴 수 있었다 = 2 m 넘게 빗나가도 "명중"이 되는 노브였다.
#   이제 판정은 설정값이 아니라 물리다.


def set_pose(model, x, y, z):
    """지정 모델을 지정 위치로 순간이동(teleport)시킨다.

    gz.transport 바인딩이 있으면 in-process 서비스 호출(빠름 → 부드러운 이동).
    없으면 gz service CLI 로 폴백(느림). 둘 다 블로킹 1회씩만 보내므로 큐 폭주 없음.
    """
    if _GZ_OK:
        req = _GzPose()
        req.name = model
        req.position.x = float(x)
        req.position.y = float(y)
        req.position.z = float(z)
        # gz 미준비 등으로 실패해도 다음 프레임에 갱신되므로 결과는 무시.
        try:
            _gz_node.request(f"/world/{WORLD}/set_pose", req, _GzPose, _GzBool, 100)
        except Exception:
            pass
        return
    req = f'name: "{model}", position: {{x: {x}, y: {y}, z: {z}}}'
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
        # target = "balloon"(기본) | "drone" : UI 토글로 표적 종류 선택. 선택된 모델만 비행, 나머진 숨김.
        self.declare_parameter("target", "balloon")
        self.target_model = MODEL_BALLOON
        self.hit = False
        self.t = 0.0                    # 대각선 진행도 [0→1]
        self.pause_until = 0.0          # 재등장 대기 종료 시각
        self._ball_pos = (0.0, 0.0, HIDE_Z)   # 풍선 현재 위치(직격 판정용)
        self._drone_pos = None                # 드론 실제 위치(gz 구독)
        self._drone_seen = False
        # 드론 실제 위치 구독 → 진짜 충돌(직격) 판정. (gz dynamic pose)
        if _GZ_OK and _GzPoseV is not None:
            try:
                _gz_node.subscribe(_GzPoseV, f"/world/{WORLD}/dynamic_pose/info",
                                   self._on_poses)
            except Exception as e:
                self.get_logger().warn(f"드론 위치 구독 실패(접근거리 로그 비활성): {e}")
        # 실제 접촉 구독 → 명중 판정. 두 표적 모두 구독해 두고, 콜백에서
        # '지금 비행 중인 표적' 인지만 확인한다 (숨겨둔 표적의 접촉은 무시).
        if _GZ_OK and _GzContacts is not None:
            for model in (MODEL_BALLOON, MODEL_DRONE):
                topic = (f"/world/{WORLD}/model/{model}/link/link"
                         f"/sensor/contact_sensor/contact")
                try:
                    _gz_node.subscribe(
                        _GzContacts, topic,
                        lambda msg, m=model: self._on_contact(msg, m))
                except Exception as e:
                    self.get_logger().warn(f"접촉 센서 구독 실패 ({model}): {e}")
        else:
            self.get_logger().error(
                "gz 바인딩 없음 — 접촉 판정 불가, 명중이 영원히 안 뜬다. "
                f"원인: {_GZ_IMPORT_ERR or '알 수 없음'}")
        # _prev_enabled=True 로 시작 → 첫 tick(비활성)에서 공을 1회 숨겨 초기 스폰 위치를 치움.
        self._prev_enabled = True
        # 한 번이라도 발사됐는지. 발사 전(초기 대기)엔 숨기고, 발사 후 정지는 제자리 멈춤.
        self._launched_once = False
        self._new_pass()
        # /arms/mission_state 는 구독하지 않는다. 예전에는 state=="FIRE" 를 보고
        # 명중을 선언했는데, 그러면 심판이 응시자의 답을 보고 채점하는 셈이라
        # "영상이 발사 판정했다" 와 "실제로 닿았다" 가 같은 값이 되어버린다.
        # 이제 둘은 독립이고, 어긋나는 것 자체가 정보다.
        # /arms/hit 은 통지용 — arms_control 은 이걸 로그로만 쓴다.
        self._hit_pub = self.create_publisher(Empty, "/arms/hit", 10)
        self._last_dist_log = 0.0     # 접근거리 로그 스로틀
        self._min_dist = 999.0        # 이번 패스 최소 접근거리
        self._last_move_t = 0.0       # 직전 이동 시각(실제 dt 계산용)
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info(
            f"balloon_referee ready. 기본 대기(숨김) 상태. [공 발사] 누르면 "
            f"먼 곳({SPAN/2:.0f}m)에서 대각선으로 광역 비행 시작.")

    def _new_pass(self):
        """대각선: 진행도 t 리셋 (오른쪽아래->왼쪽위). 판정도 패스 단위로 리셋."""
        self.t = 0.0
        self.hit = False
        self._min_dist = 999.0

    def _on_poses(self, msg):
        """gz dynamic pose 구독 콜백 → 드론 실제 위치 추적 (직격 판정용)."""
        if not self._drone_seen and not getattr(self, "_names_logged", False):
            self._names_logged = True
            self.get_logger().info(
                f"gz dynamic pose 모델들: {[p.name for p in msg.pose]}")
        for p in msg.pose:
            n = p.name
            if ("drone" in n or "x500" in n) and n not in (MODEL_BALLOON, MODEL_DRONE):
                self._drone_pos = (p.position.x, p.position.y, p.position.z)
                if not self._drone_seen:
                    self._drone_seen = True
                    self.get_logger().info(f"직격 판정 활성화 (드론='{n}' 위치 추적 중)")
                return

    @staticmethod
    def _is_our_drone(collision_name):
        """접촉 상대가 우리 요격 드론인가.

        표적 두 개는 이름에 drone/x500 이 안 들어가게 지어져 있다
        (target_ball / target_uav) — 자기드론 식별과 겹치지 않으려고 일부러
        그렇게 뒀고, _on_poses 도 같은 규칙을 쓴다.
        """
        n = collision_name
        return (("drone" in n or "x500" in n)
                and not n.startswith(MODEL_BALLOON)
                and not n.startswith(MODEL_DRONE))

    def _on_contact(self, msg, model):
        """Gazebo 접촉 센서 콜백 — 우리 드론과 닿았을 때만 명중으로 친다.

        상대를 반드시 확인해야 한다. 비활성 표적은 둘 다 HIDE_Z(0,0,-100)
        에 주차되는데 반경 1.0 구가 서로 겹쳐서 **접촉이 상시 발생**한다
        (실측 확인). 지면(ground_plane)도 마찬가지로 접촉 상대가 될 수 있다.
        상대를 안 보면 발사 전부터 "명중" 이 뜬다.
        """
        if model != self.target_model or self.hit:
            return          # 숨겨둔 표적의 센서이거나 이미 이번 패스에서 보고함
        for c in msg.contact:
            for name in (c.collision1.name, c.collision2.name):
                if self._is_our_drone(name):
                    self._declare_hit(f"접촉: {name}")
                    return

    def _declare_hit(self, reason):
        """명중 보고. 상태 변화는 없다 — 로그와 /arms/hit 통지뿐.

        예전에는 여기서 표적을 튕겨 날리는 연출을 하고, /arms/hit 이
        arms_control 의 상태머신을 FIRE 로 밀었다. 둘 다 없앴다:
          · 미션 전이는 영상(τ) 판정 하나로만 일어난다. 심판이 대신 눌러주면
            검출이 고장나도 미션이 성공으로 끝나 성능 측정이 오염된다.
          · 튕김은 물리가 한다. 표적은 kinematic(무한질량)이라 밀리지 않고,
            부딪힌 드론이 튕겨 나간다 — 실제로 일어날 일 그대로다.
        self.hit 은 한 패스에 한 번만 로그하려는 중복 방지 플래그일 뿐이다.
        """
        if self.hit:
            return
        self.hit = True
        self._hit_pub.publish(Empty())    # 제어노드는 이걸 로그로만 쓴다
        extra = ""
        if self._drone_pos is not None:
            dx = self._ball_pos[0] - self._drone_pos[0]
            dy = self._ball_pos[1] - self._drone_pos[1]
            dz = self._ball_pos[2] - self._drone_pos[2]
            extra = f"  중심거리 {math.sqrt(dx*dx + dy*dy + dz*dz):.2f}m"
        self.get_logger().info(
            f"🎯 명중 — {reason}{extra}  (이번 패스 최소접근 {self._min_dist:.2f}m)")

    def tick(self):
        enabled = self.get_parameter("enabled").value
        now = self.get_clock().now().nanoseconds * 1e-9

        # ---- 표적 종류 전환 (UI 토글: target 파라미터) ----
        #   선택된 모델만 비행시키고, 직전 표적은 지하로 숨긴다. (명중은 거리기반이라 무손상)
        tgt = str(self.get_parameter("target").value).lower()
        want = MODEL_DRONE if tgt == "drone" else MODEL_BALLOON
        if want != self.target_model:
            old = self.target_model
            self.target_model = want
            set_pose(old, 0.0, 0.0, HIDE_Z)              # 이전 표적 숨김
            bx, by, bz = self._ball_pos
            set_pose(self.target_model, bx, by, bz)      # 새 표적을 현재 위치로 (비행 중이면 즉시 교체 보임)
            self.get_logger().info(
                f"표적 전환 → {'드론' if want == MODEL_DRONE else '풍선'} ({want})")

        # ---- 비활성 상태 처리 ----
        #   · 초기 대기(발사 전): 두 표적 모두 지하로 숨겨 화면에 안 보이게.
        #   · 발사 후 정지: 현재 위치에 그대로 멈춤(일시정지). set_pose 를 안 보내면
        #     Gazebo 가 마지막 위치를 유지하므로 사라지지 않고 제자리에 정지.
        if not enabled:
            if self._prev_enabled and not self._launched_once:
                set_pose(MODEL_BALLOON, 0.0, 0.0, HIDE_Z)   # 발사 전 초기 스폰: 둘 다 숨김
                set_pose(MODEL_DRONE, 0.0, 0.0, HIDE_Z)
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

        # 명중해도 표적은 원래 궤적을 그대로 계속 난다. 인위적으로 튕겨내던
        # 연출은 없앴다 — 표적은 kinematic(무한질량)이라 물리적으로 밀리지 않고,
        # 부딪힌 쪽인 드론이 튕겨 나가는 게 실제로 일어나는 일이다.
        # self.hit 은 한 패스에 한 번만 로그하려는 플래그로만 남는다.

        # 재등장 대기 중이면 풍선 숨김
        if now < self.pause_until:
            return

        alt = self.get_parameter("alt").value   # 패널 슬라이더로 바뀌는 고도

        # 광역 대각선: 화각 제약 scale 없이 SPAN 전체를 이동.
        #   시작 x=+SPAN/2(화면 아래), y=-SPAN/2(화면 오른쪽) = 화면 오른쪽아래 먼 곳
        #   끝   x=-SPAN/2(화면 위),   y=+SPAN/2(화면 왼쪽)   = 화면 왼쪽위 먼 곳
        #   중간(t=0.5)에 (0,0) 카메라 중심을 통과 → 시야 관통.
        speed = self.get_parameter("speed").value   # 단일 속도 (패널 슬라이더)
        # ★ 실제 경과시간(dt)으로 진행 → 타이머 지터/렉에도 '일정한' 속도 유지.
        #   (기존 1/RATE_HZ 고정은 틱이 늦게 오면 그만큼 느려/빨라 보였음 = 렉처럼 들쭉날쭉)
        dt = now - self._last_move_t
        self._last_move_t = now
        dt = max(0.0, min(dt, 3.0 / RATE_HZ))   # 정지→재개 등 큰 점프 방지
        span = SPAN
        self.t += (speed / max(span, 0.1)) * dt
        f = 1.0 - 2.0 * self.t
        x = (span / 2.0) * f
        y = -(span / 2.0) * f
        z = alt
        set_pose(self.target_model, x, y, z)
        self._ball_pos = (x, y, z)

        # ── 접근거리 로그 (판정 아님) ──────────────────────────────────────
        #   명중 판정은 Gazebo 접촉 센서(_on_contact)가 한다. 여기서 재는 거리는
        #   "실제로 몇 m 까지 갔는가" 를 보기 위한 진단일 뿐, 어떤 결정도 하지 않는다.
        if self._drone_pos is not None:
            dx = x - self._drone_pos[0]
            dy = y - self._drone_pos[1]
            dz = z - self._drone_pos[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            self._min_dist = min(self._min_dist, dist)
            if dist < 8.0 and (now - self._last_dist_log) > 0.4:
                self._last_dist_log = now
                self.get_logger().info(
                    f"접근거리 {dist:.2f}m (이번 패스 최소 {self._min_dist:.2f}m)")
        elif (now - self._last_dist_log) > 2.0:
            self._last_dist_log = now
            self.get_logger().warn("드론 위치 미수신 — 접근거리 로그 비활성 (gz 구독 확인)")

        if self.t >= 1.0:
            set_pose(self.target_model, 0.0, 0.0, HIDE_Z)   # 반대편 먼 곳 통과 후 숨김
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
