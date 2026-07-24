#!/usr/bin/env python3
"""
arms_panel.py — A.R.M.S. SITL 컨트롤 패널 (시작하자마자 뜨는 인터페이스)

기능:
  - 미션 상태 표시 (IDLE/SEARCH/LOCK/TRACK/FIRE/RTL)
  - [LAUNCH]            : /arms/launch_cmd 발행 (수동 TRACK 전환)
  - [YOLO/CV/BOTH]      : fusion_detector 검출 모드 토글
  - [Roll/Pitch 부호]   : 제어 방향 뒤집기 (재빌드 X)
  - [시작/최대 P + 증가시간]: TRACK 진입 후 시간에 따라 P 를 시작→최대로 램프
  - [kd 슬라이더]       : roll/pitch D 게인 실시간
  - [상승 추력] 슬라이더 : track_throttle
  - [공 생성]           : red_ball 입력 위치/속도로 (referee 자동 정지)

확정 환경: 월드=arms_sitl, 드론=arms_drone, 표적=red_ball
실행: python3 arms_panel.py  (런치에서 자동 실행)
"""

import queue
import subprocess
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from arms_msgs.msg import MissionState

CTRL_NODE = "/arms_control_node"
FUSION_NODE = "/fusion_detector"
REFEREE_NODE = "/balloon_referee"

WORLD = "arms_sitl"
DRONE = "arms_drone_0"   # PX4가 스폰 시 인스턴스 번호 _0 을 붙임 (Entity Tree 확인)
BALL = "red_ball"


STATE_COLOR = {
    "IDLE": "#888888", "SEARCH": "#f5a623", "LOCK": "#f57c00",
    "BOOST": "#ff6d00", "TRACK": "#d0021b", "FIRE": "#ff1744", "RTL": "#2979ff",
}


def _bg(fn):
    """외부 프로세스 호출을 백그라운드로 → tkinter 메인스레드 freeze 방지."""
    threading.Thread(target=fn, daemon=True).start()


def ros_param_set(name, value):
    _bg(lambda: subprocess.run(
        ["ros2", "param", "set", CTRL_NODE, name, str(value)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))


def ros_param_set_node(node, name, value):
    _bg(lambda: subprocess.run(
        ["ros2", "param", "set", node, name, str(value)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))










class PanelNode(Node):
    def __init__(self, q):
        super().__init__("arms_panel")
        self.q = q
        self.launch_pub = self.create_publisher(Empty, "/arms/launch_cmd", 10)
        self.create_subscription(MissionState, "/arms/mission_state", self._cb, 10)

    def _cb(self, msg):
        self.q.put(msg)

    def fire_launch(self):
        self.launch_pub.publish(Empty())



class PanelGUI:
    def __init__(self, root, node, q):
        self.node = node
        self.q = q
        self.roll_sign = 1.0
        self.pitch_sign = -1.0   # 확정값(yaml)과 일치
        self.ball_flying = False  # 시작 시 풍선 대기(숨김) — referee enabled 기본 False 와 일치

        root.title("A.R.M.S. Control Panel")
        root.configure(bg="#1e1e1e")
        root.geometry("400x1130")

        tk.Label(root, text="MISSION STATE", fg="#aaaaaa", bg="#1e1e1e",
                 font=("Arial", 11)).pack(pady=(10, 2))
        self.state_lbl = tk.Label(root, text="—", fg="white", bg="#888888",
                                   font=("Arial", 26, "bold"), width=12)
        self.state_lbl.pack(pady=3, ipady=7)
        self.info_lbl = tk.Label(root, text="error: -, -\nlock: 0.0s",
                                 fg="#dddddd", bg="#1e1e1e", font=("Consolas", 11))
        self.info_lbl.pack(pady=3)

        tk.Button(root, text="🚀 LAUNCH (→TRACK)", font=("Arial", 12, "bold"),
                  bg="#d0021b", fg="white", activebackground="#ff1744",
                  command=self.node.fire_launch).pack(pady=5, fill="x", padx=20)
        # 적기(풍선) 발사 — 누르면 먼 곳에서 대각선으로 등장해 광역 비행.
        # 이미 비행 중이어도 항상 시작점(먼 곳)부터 새로 발사(enabled false→true 리셋).
        self.ball_fly_btn = tk.Button(root, text="🚀 공 발사 (적기)", font=("Arial", 12, "bold"),
                  bg="#c0392b", fg="white", activebackground="#e74c3c",
                  command=self.launch_balloon)
        self.ball_fly_btn.pack(pady=5, fill="x", padx=20)
        self.ball_stop_btn = tk.Button(root, text="⏹ 공 정지 (숨김)", font=("Arial", 9),
                  command=self.stop_balloon)
        self.ball_stop_btn.pack(pady=2, fill="x", padx=40)

        tk.Label(root, text="DETECTION MODE (다중 선택)", fg="#aaaaaa", bg="#1e1e1e",
                 font=("Arial", 10)).pack(pady=(8, 2))
        mfrm = tk.Frame(root, bg="#1e1e1e")
        mfrm.pack()
        # 3개 독립 토글. 여러 개 켜면 다 적용(융합). param 이름 = use_hsv/use_yolo/use_absdiff
        # 기본: hsv ON, yolo OFF, absdiff ON
        self.det_on = {"hsv": True, "yolo": False, "absdiff": True}
        self.det_btns = {}
        for key, label, col in [("hsv", "HSV", "#2e7d32"),
                                ("yolo", "YOLO", "#6a1b9a"),
                                ("absdiff", "ABSDIFF", "#0277bd")]:
            b = tk.Button(mfrm, text=label, width=9, font=("Arial", 9, "bold"),
                          command=lambda k=key: self.toggle_det(k))
            b.grid(row=0, column=len(self.det_btns), padx=2)
            self.det_btns[key] = (b, col)
        self._refresh_det_btns()
        tk.Label(root, text="눌린 것 = ON (초록/보라/파랑). 여러 개 켜면 융합 검출.",
                 fg="#777777", bg="#1e1e1e", font=("Arial", 8)).pack()

        sfrm = tk.Frame(root, bg="#1e1e1e")
        sfrm.pack(pady=6)
        self.roll_btn = tk.Button(sfrm, text="Roll sign: +", width=14, command=self.flip_roll)
        self.roll_btn.grid(row=0, column=0, padx=4)
        self.pitch_btn = tk.Button(sfrm, text="Pitch sign: -", width=14, command=self.flip_pitch)
        self.pitch_btn.grid(row=0, column=1, padx=4)

        tk.Label(root, text="P 게인 (시간 램프)", fg="#aaaaaa",
                 bg="#1e1e1e", font=("Arial", 10)).pack(pady=(8, 0))
        self.kp_start = tk.Scale(root, from_=0, to=150, resolution=1, orient="horizontal",
                                 length=300, bg="#1e1e1e", fg="white", label="시작 P (약하게)",
                                 highlightthickness=0, troughcolor="#444")
        self.kp_start.set(8)
        self.kp_start.pack()
        self.kp_start.bind("<ButtonRelease-1>",
                           lambda e: ros_param_set("control.kp_start", float(self.kp_start.get())))
        self.kp_max = tk.Scale(root, from_=0, to=160, resolution=1, orient="horizontal",
                               length=300, bg="#1e1e1e", fg="white", label="최대 P (램프 끝)",
                               highlightthickness=0, troughcolor="#444")
        self.kp_max.set(25)
        self.kp_max.pack()
        self.kp_max.bind("<ButtonRelease-1>",
                         lambda e: ros_param_set("control.kp_max", float(self.kp_max.get())))
        self.kp_ramp = tk.Scale(root, from_=0, to=15, resolution=0.5, orient="horizontal",
                                length=300, bg="#1e1e1e", fg="white", label="P 증가 시간 [s]",
                                highlightthickness=0, troughcolor="#444")
        self.kp_ramp.set(5)
        self.kp_ramp.pack()
        self.kp_ramp.bind("<ButtonRelease-1>",
                          lambda e: ros_param_set("control.kp_ramp_sec", float(self.kp_ramp.get())))
        self.kd = tk.Scale(root, from_=0, to=2, resolution=0.05, orient="horizontal",
                           length=300, bg="#1e1e1e", fg="white", label="kd (roll/pitch)",
                           highlightthickness=0, troughcolor="#444")
        self.kd.set(0.6)
        self.kd.pack()
        self.kd.bind("<ButtonRelease-1>", lambda e: self.set_pid_kd(self.kd.get()))

        self.ki = tk.Scale(root, from_=0.0, to=2.0, resolution=0.1, orient="horizontal",
                           length=300, bg="#1e1e1e", fg="white", label="ki (적분 - 중앙오차 제거)",
                           highlightthickness=0, troughcolor="#444")
        self.ki.set(0.8)
        self.ki.pack()
        self.ki.bind("<ButtonRelease-1>", lambda e: self.set_pid_ki(self.ki.get()))

        self.maxang = tk.Scale(root, from_=30, to=120, resolution=5, orient="horizontal",
                               length=300, bg="#1e1e1e", fg="white", label="최대 각도 [deg]",
                               highlightthickness=0, troughcolor="#444")
        self.maxang.set(30)
        self.maxang.pack()
        self.maxang.bind("<ButtonRelease-1>", lambda e: self.set_max_angle(self.maxang.get()))

        tk.Label(root, text="상승 추력 (track_throttle)", fg="#aaaaaa",
                 bg="#1e1e1e").pack(pady=(6, 0))
        self.thr = tk.Scale(root, from_=0.50, to=0.95, resolution=0.01,
                            orient="horizontal", length=300, bg="#1e1e1e",
                            fg="white", highlightthickness=0, troughcolor="#444")
        self.thr.set(0.85)
        self.thr.pack()
        self.thr.bind("<ButtonRelease-1>",
                      lambda e: ros_param_set("control.track_throttle", self.thr.get()))

        tk.Label(root, text="풍선(red_ball) 상태 — 발사/정지는 위 버튼 사용", fg="#aaaaaa",
                 bg="#1e1e1e", font=("Arial", 10)).pack(pady=(8, 2))
        # 현재 풍선 상태 표시 라벨 (위 [공 발사]/[공 정지] 버튼과 연동)
        self.ball_state_lbl = tk.Label(root, text="풍선 상태: 대기 (숨김)",
                                       fg="white", bg="#2e7d32", font=("Arial", 11, "bold"),
                                       width=30)
        self.ball_state_lbl.pack(pady=(2, 0), ipady=4)
        self._refresh_ball_state()

        # 풍선 고도 슬라이더 (referee alt 파라미터 실시간 변경)
        self.alt = tk.Scale(root, from_=2, to=100, resolution=1, orient="horizontal",
                            length=300, bg="#1e1e1e", fg="white", label="풍선 고도 [m]",
                            highlightthickness=0, troughcolor="#444")
        self.alt.set(50)
        self.alt.pack(pady=(2, 0))
        self.alt.bind("<ButtonRelease-1>",
                      lambda e: ros_param_set_node(REFEREE_NODE, "alt", float(self.alt.get())))

        # 풍선 속도 슬라이더 (referee speed 파라미터 실시간 변경)
        self.ball_speed = tk.Scale(root, from_=2, to=40, resolution=1, orient="horizontal",
                                   length=300, bg="#1e1e1e", fg="white", label="풍선 속도 [m/s]",
                                   highlightthickness=0, troughcolor="#444")
        self.ball_speed.set(12)
        self.ball_speed.pack(pady=(2, 0))
        self.ball_speed.bind("<ButtonRelease-1>",
                             lambda e: ros_param_set_node(REFEREE_NODE, "speed", float(self.ball_speed.get())))

        self._poll()

    def toggle_det(self, key):
        # 해당 검출기 on/off 뒤집고 fusion_detector 의 use_<key> 파라미터 갱신
        self.det_on[key] = not self.det_on[key]
        ros_param_set_node(FUSION_NODE, f"use_{key}", str(self.det_on[key]).lower())
        self._refresh_det_btns()

    def _refresh_det_btns(self):
        for key, (b, col) in self.det_btns.items():
            if self.det_on[key]:
                b.config(bg=col, fg="white", relief="sunken")      # ON
            else:
                b.config(bg="#333333", fg="#cccccc", relief="raised")  # OFF

    def launch_balloon(self):
        # 발사: enabled false→true 상승엣지 → referee 가 항상 먼 시작점부터 새 패스로 등장.
        # (이미 비행 중이어도 다시 시작점부터 발사)
        ros_param_set_node(REFEREE_NODE, "enabled", "false")
        threading.Timer(0.3, lambda: ros_param_set_node(REFEREE_NODE, "enabled", "true")).start()
        self.ball_flying = True
        self._refresh_ball_state()

    def stop_balloon(self):
        # 정지: referee 가 공을 지하로 숨김 → 화면에서 사라짐
        ros_param_set_node(REFEREE_NODE, "enabled", "false")
        self.ball_flying = False
        self._refresh_ball_state()

    def flip_roll(self):
        self.roll_sign *= -1.0
        ros_param_set("control.roll_sign", self.roll_sign)
        self.roll_btn.config(text=f"Roll sign: {'+' if self.roll_sign > 0 else '-'}")

    def flip_pitch(self):
        self.pitch_sign *= -1.0
        ros_param_set("control.pitch_sign", self.pitch_sign)
        self.pitch_btn.config(text=f"Pitch sign: {'+' if self.pitch_sign > 0 else '-'}")

    def set_pid_kd(self, v):
        ros_param_set("control.roll_pid.kd", float(v))
        ros_param_set("control.pitch_pid.kd", float(v))

    def set_pid_ki(self, v):
        ros_param_set("control.roll_pid.ki", float(v))
        ros_param_set("control.pitch_pid.ki", float(v))

    def set_max_angle(self, v):
        ros_param_set("control.roll_pid.output_limit", float(v))
        ros_param_set("control.pitch_pid.output_limit", float(v))

    def _refresh_ball_state(self):
        # 현재 풍선 상태에 맞게 상단 [공 발사]/[공 정지] 버튼 강조 + 라벨 갱신
        flying = getattr(self, "ball_flying", False)
        if flying:
            self.ball_state_lbl.config(text="풍선 상태: 비행 중 (적기 접근)", bg="#c0392b")
            self.ball_fly_btn.config(relief="sunken")
            self.ball_stop_btn.config(bg="#333333", fg="#cccccc", relief="raised")
        else:
            self.ball_state_lbl.config(text="풍선 상태: 대기 (숨김)", bg="#2e7d32")
            self.ball_fly_btn.config(relief="raised")
            self.ball_stop_btn.config(bg="#2e7d32", fg="white", relief="sunken")

    def _poll(self):
        try:
            while True:
                msg = self.q.get_nowait()
                color = STATE_COLOR.get(msg.state, "#888888")
                self.state_lbl.config(text=msg.state or "—", bg=color)
                self.info_lbl.config(
                    text=f"error: {msg.error_x:+.2f}, {msg.error_y:+.2f}\n"
                         f"lock: {msg.lock_elapsed_sec:.1f}s   현재 P: {msg.kp_now:.0f}")
        except queue.Empty:
            pass
        self.state_lbl.after(100, self._poll)


def main():
    rclpy.init()
    q = queue.Queue()
    node = PanelNode(q)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    root = tk.Tk()
    PanelGUI(root, node, q)
    try:
        root.mainloop()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
