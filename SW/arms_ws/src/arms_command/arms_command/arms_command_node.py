#!/usr/bin/env python3
"""
arms_command_node.py — A.R.M.S. SITL 컨트롤 패널 (tkinter GUI)

기능:
  - 미션 상태 표시 (IDLE/SEARCH/LOCK/TRACK/FIRE/RTL)
  - [LAUNCH]            : /arms/command buttons[3]=1 발행 (수동 TRACK 전환)
  - [YOLO/HSV/ABSDIFF]  : arms_detection_node 검출 모드 토글
  - [Roll/Pitch 부호]   : 제어 방향 뒤집기 (재빌드 X)
  - [시작/최대 P + 증가시간]: TRACK 진입 후 시간에 따라 P 를 시작→최대로 램프
  - [kd/ki 슬라이더]    : roll/pitch D/I 게인 실시간
  - [상승 추력] 슬라이더 : track_throttle
  - [풍선 비행/정지]    : balloon_referee 파라미터로 표적 자동 비행 제어

  별도 창 (ControllerGUI):
  - [CONTROLLER INPUT]  : 스틱 드래그 → /arms/command axes 발행
                          스위치 클릭 → /arms/command buttons 발행

확정 환경: 월드=arms_sitl, 드론=arms_drone, 표적=red_ball
실행: ros2 run arms_command arms_command_node
"""

import math
import queue
import subprocess
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from arms_msgs.msg import MissionState

CTRL_NODE    = "/arms_control_node"
FUSION_NODE  = "/arms_detection_node"
REFEREE_NODE = "/balloon_referee"

WORLD = "arms_sitl"
DRONE = "arms_drone_0"
BALL  = "red_ball"

STATE_COLOR = {
    "IDLE":   "#888888",
    "SEARCH": "#f5a623",
    "LOCK":   "#f57c00",
    "TRACK":  "#d0021b",
    "FIRE":   "#ff1744",
    "RTL":    "#2979ff",
}

STICK_SIZE = 80
STICK_R    = 36
DOT_R      = 8


def _bg(fn):
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
        super().__init__("arms_command_node")
        self.q = q
        self._joy_pub = self.create_publisher(Joy, "/arms/command", 10)
        self._axes = [0.0, 0.0, 0.0, 0.0]
        self._buttons = [0, 0, 0, 0]
        self.create_subscription(MissionState, "/arms/mission_state", self._cb_state, 10)
        self.create_timer(0.05, self._publish_joy)  # 20 Hz

    def _cb_state(self, msg):
        self.q.put(msg)

    def _publish_joy(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = list(self._axes)
        msg.buttons = list(self._buttons)
        self._joy_pub.publish(msg)

    def set_axis(self, idx, value):
        self._axes[idx] = float(value)

    def toggle_button(self, idx):
        self._buttons[idx] ^= 1
        return self._buttons[idx]

    def fire_launch(self):
        self._buttons[3] = 1

    def release_launch(self):
        self._buttons[3] = 0


# ---------------------------------------------------------------------------
# 메인 컨트롤 패널
# ---------------------------------------------------------------------------

class PanelGUI:
    def __init__(self, root, node, q):
        self.root = root
        self.node = node
        self.q = q
        self.roll_sign  = 1.0
        self.pitch_sign = 1.0

        root.title("A.R.M.S. Control Panel")
        root.configure(bg="#1e1e1e")
        root.geometry("740x580")

        # ── 상단: 미션 상태 (전체 폭) ──────────────────────────────────────
        top = tk.Frame(root, bg="#1e1e1e")
        top.pack(fill="x", padx=12, pady=(10, 4))

        tk.Label(top, text="MISSION STATE", fg="#aaaaaa", bg="#1e1e1e",
                 font=("Arial", 11)).pack()
        self.state_lbl = tk.Label(top, text="—", fg="white", bg="#888888",
                                   font=("Arial", 24, "bold"), width=14)
        self.state_lbl.pack(pady=3, ipady=6)
        self.info_lbl = tk.Label(top, text="error: -, -\nlock: 0.0s",
                                 fg="#dddddd", bg="#1e1e1e", font=("Consolas", 10))
        self.info_lbl.pack()
        tk.Button(top, text="LAUNCH (→TRACK)", font=("Arial", 12, "bold"),
                  bg="#d0021b", fg="white", activebackground="#ff1744",
                  command=self._on_launch).pack(pady=(5, 2), fill="x", padx=40)
        tk.Button(top, text="RESET (드론 원점 복귀)", font=("Arial", 10),
                  bg="#37474f", fg="white", activebackground="#546e7a",
                  command=self._on_reset).pack(pady=(0, 4), fill="x", padx=40)

        # ── 2열 레이아웃 ────────────────────────────────────────────────────
        cols = tk.Frame(root, bg="#1e1e1e")
        cols.pack(fill="both", expand=True, padx=(12, 4), pady=4)

        left = tk.Frame(cols, bg="#1e1e1e")
        left.grid(row=0, column=0, sticky="nw", padx=(0, 16))

        sep = tk.Frame(cols, bg="#444444", width=1)
        sep.grid(row=0, column=1, sticky="ns", padx=4)

        right = tk.Frame(cols, bg="#1e1e1e")
        right.grid(row=0, column=2, sticky="nw")

        # ── 왼쪽: 검출 모드 ─────────────────────────────────────────────────
        tk.Label(left, text="DETECTION MODE (다중 선택)", fg="#aaaaaa", bg="#1e1e1e",
                 font=("Arial", 10)).pack(pady=(4, 2))
        mfrm = tk.Frame(left, bg="#1e1e1e")
        mfrm.pack()
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
        tk.Label(left, text="눌린 것 = ON. 여러 개 켜면 융합 검출.",
                 fg="#777777", bg="#1e1e1e", font=("Arial", 8)).pack()

        sfrm = tk.Frame(left, bg="#1e1e1e")
        sfrm.pack(pady=4)
        self.roll_btn = tk.Button(sfrm, text="Roll sign: +", width=13, command=self.flip_roll)
        self.roll_btn.grid(row=0, column=0, padx=3)
        self.pitch_btn = tk.Button(sfrm, text="Pitch sign: +", width=13, command=self.flip_pitch)
        self.pitch_btn.grid(row=0, column=1, padx=3)

        # ── 왼쪽: PID 입력 ──────────────────────────────────────────────────
        def make_pid_group(parent, title, defaults, params):
            """(P, I, D) 입력 행 + 적용 버튼. returns (ep, ei, ed) Entry widgets."""
            tk.Label(parent, text=title, fg="#aaaaaa", bg="#1e1e1e",
                     font=("Arial", 10, "bold")).pack(anchor="w", pady=(8, 2))
            row = tk.Frame(parent, bg="#1e1e1e")
            row.pack(anchor="w")
            entries = []
            for col, (lbl, val) in enumerate(zip(["P", "I", "D"], defaults)):
                tk.Label(row, text=lbl, fg="white", bg="#1e1e1e",
                         width=2).grid(row=0, column=col * 2, padx=(4, 0))
                e = tk.Entry(row, width=7, bg="#2e2e2e", fg="white",
                             insertbackground="white", relief="flat")
                e.insert(0, str(val))
                e.grid(row=0, column=col * 2 + 1, padx=(2, 6))
                entries.append(e)
            ep, ei, ed = entries

            def apply():
                try:
                    ros_param_set(params[0], float(ep.get()))
                    ros_param_set(params[1], float(ei.get()))
                    ros_param_set(params[2], float(ed.get()))
                except ValueError:
                    pass

            tk.Button(row, text="적용", bg="#455a64", fg="white",
                      activebackground="#607d8b", relief="flat",
                      command=apply).grid(row=0, column=6, padx=(2, 0))
            return ep, ei, ed

        self.roll_kp, self.roll_ki, self.roll_kd = make_pid_group(
            left, "Roll PID", [20, 0.8, 0.65],
            ["control.roll_pid.kp", "control.roll_pid.ki", "control.roll_pid.kd"])

        self.pitch_kp, self.pitch_ki, self.pitch_kd = make_pid_group(
            left, "Pitch PID", [20, 0.8, 0.65],
            ["control.pitch_pid.kp", "control.pitch_pid.ki", "control.pitch_pid.kd"])

        # 추력 슬라이더는 아날로그 느낌이 있어서 유지
        tk.Label(left, text="상승 추력 (track_throttle)", fg="#aaaaaa",
                 bg="#1e1e1e", font=("Arial", 9)).pack(anchor="w", pady=(10, 0))
        thr_row = tk.Frame(left, bg="#1e1e1e")
        thr_row.pack(anchor="w")
        self.thr_entry = tk.Entry(thr_row, width=7, bg="#2e2e2e", fg="white",
                                  insertbackground="white", relief="flat")
        self.thr_entry.insert(0, "0.77")
        self.thr_entry.grid(row=0, column=0, padx=(4, 6))
        tk.Button(thr_row, text="적용", bg="#455a64", fg="white",
                  activebackground="#607d8b", relief="flat",
                  command=lambda: ros_param_set(
                      "control.track_throttle", float(self.thr_entry.get()))
                  ).grid(row=0, column=1)

        # ── 오른쪽: 풍선 ────────────────────────────────────────────────────
        tk.Label(right, text="풍선 (red_ball)", fg="#aaaaaa", bg="#1e1e1e",
                 font=("Arial", 11, "bold")).pack(pady=(4, 6))

        bbtn = tk.Frame(right, bg="#1e1e1e")
        bbtn.pack(pady=2)
        self.ball_fly_btn = tk.Button(bbtn, text="비행 시작", width=12,
                                      font=("Arial", 10, "bold"),
                                      command=self.ball_start)
        self.ball_fly_btn.grid(row=0, column=0, padx=3, pady=2)
        self.ball_stop_btn = tk.Button(bbtn, text="정지", width=8,
                                       command=self.ball_stop)
        self.ball_stop_btn.grid(row=0, column=1, padx=3, pady=2)

        self.ball_state_lbl = tk.Label(right, text="상태: ? (버튼으로 설정)",
                                       fg="white", bg="#555555",
                                       font=("Arial", 10, "bold"), width=26)
        self.ball_state_lbl.pack(pady=(2, 6), ipady=4)
        self._refresh_ball_btns()

        self.alt = tk.Scale(right, from_=2, to=100, resolution=1, orient="horizontal",
                            length=150, bg="#1e1e1e", fg="white", label="풍선 고도 [m]",
                            highlightthickness=0, troughcolor="#444")
        self.alt.set(50)
        self.alt.pack(pady=(2, 0))
        self.alt.bind("<ButtonRelease-1>",
                      lambda e: ros_param_set_node(REFEREE_NODE, "alt", float(self.alt.get())))

        self._poll()

    # ------------------------------------------------------------------
    # LAUNCH / RESET 버튼
    # ------------------------------------------------------------------
    def _on_launch(self):
        self.node.fire_launch()
        self.root.after(300, self._release_launch)

    def _release_launch(self):
        self.node.release_launch()

    def _on_reset(self):
        # 1) arms_control → IDLE
        _bg(lambda: subprocess.run(
            ["ros2", "topic", "pub", "--once", "/arms/reset_cmd", "std_msgs/msg/Empty", "{}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        # 2) Gazebo 드론 원점 복귀
        pose_args = (
            "position: {x: 0.0, y: 0.0, z: 0.2}, "
            "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
        )
        _bg(lambda: subprocess.run([
            "gz", "service",
            "-s", f"/world/{WORLD}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", f"name: '{DRONE}', {pose_args}",
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

    # ------------------------------------------------------------------
    def toggle_det(self, key):
        self.det_on[key] = not self.det_on[key]
        ros_param_set_node(FUSION_NODE, f"use_{key}", str(self.det_on[key]).lower())
        self._refresh_det_btns()

    def _refresh_det_btns(self):
        for key, (b, col) in self.det_btns.items():
            if self.det_on[key]:
                b.config(bg=col, fg="white", relief="sunken")
            else:
                b.config(bg="#333333", fg="#cccccc", relief="raised")

    def flip_roll(self):
        self.roll_sign *= -1.0
        ros_param_set("control.roll_sign", self.roll_sign)
        self.roll_btn.config(text=f"Roll sign: {'+' if self.roll_sign > 0 else '-'}")

    def flip_pitch(self):
        self.pitch_sign *= -1.0
        ros_param_set("control.pitch_sign", self.pitch_sign)
        self.pitch_btn.config(text=f"Pitch sign: {'+' if self.pitch_sign > 0 else '-'}")

    def ball_start(self):
        ros_param_set_node(REFEREE_NODE, "enabled", "true")
        self.ball_flying = True
        self._refresh_ball_btns()

    def ball_stop(self):
        ros_param_set_node(REFEREE_NODE, "enabled", "false")
        self.ball_flying = False
        self._refresh_ball_btns()

    def _refresh_ball_btns(self):
        flying = getattr(self, "ball_flying", None)
        if flying is True:
            self.ball_state_lbl.config(text="풍선 상태: 비행 중 (움직임)", bg="#9c27b0")
            self.ball_fly_btn.config(bg="#9c27b0", fg="white", relief="sunken")
            self.ball_stop_btn.config(bg="#333333", fg="#cccccc", relief="raised")
        elif flying is False:
            self.ball_state_lbl.config(text="풍선 상태: 정지 (멈춤)", bg="#2e7d32")
            self.ball_fly_btn.config(bg="#333333", fg="#cccccc", relief="raised")
            self.ball_stop_btn.config(bg="#2e7d32", fg="white", relief="sunken")
        else:
            self.ball_state_lbl.config(text="풍선 상태: ? (버튼을 눌러 설정)", bg="#555555")
            self.ball_fly_btn.config(bg="#9c27b0", fg="white", relief="raised")
            self.ball_stop_btn.config(bg="#333333", fg="#cccccc", relief="raised")

    def _poll(self):
        try:
            while True:
                msg = self.q.get_nowait()
                if isinstance(msg, MissionState):
                    color = STATE_COLOR.get(msg.state, "#888888")
                    self.state_lbl.config(text=msg.state or "—", bg=color)
                    self.info_lbl.config(
                        text=f"error: {msg.error_x:+.2f}, {msg.error_y:+.2f}\n"
                             f"lock: {msg.lock_elapsed_sec:.1f}s   현재 P: {msg.kp_now:.0f}")
        except queue.Empty:
            pass
        self.state_lbl.after(50, self._poll)


# ---------------------------------------------------------------------------
# 조종기 입력 패널 (별도 창)
# ---------------------------------------------------------------------------

class ControllerGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node

        root.title("Controller Input")
        root.configure(bg="#1e1e1e")
        root.geometry("280x230")

        tk.Label(root, text="CONTROLLER INPUT", fg="#aaaaaa", bg="#1e1e1e",
                 font=("Arial", 10)).pack(pady=(8, 0))
        tk.Label(root, text="스틱: 드래그 (놓으면 중앙 복귀) / 스위치: 클릭 토글",
                 fg="#666666", bg="#1e1e1e", font=("Arial", 8)).pack()

        stick_frm = tk.Frame(root, bg="#1e1e1e")
        stick_frm.pack(pady=6)

        # 왼쪽 스틱 (ax0=X, ax1=Y)
        lfrm = tk.Frame(stick_frm, bg="#1e1e1e")
        lfrm.grid(row=0, column=0, padx=16)
        tk.Label(lfrm, text="L-STICK", fg="#888888", bg="#1e1e1e",
                 font=("Arial", 8)).pack()
        self.l_canvas = tk.Canvas(lfrm, width=STICK_SIZE, height=STICK_SIZE,
                                  bg="#2a2a2a", highlightthickness=0, cursor="crosshair")
        self.l_canvas.pack()
        tk.Label(lfrm, text="ax0, ax1", fg="#666666", bg="#1e1e1e",
                 font=("Arial", 7)).pack()
        self._draw_stick_bg(self.l_canvas)
        self.l_dot = self.l_canvas.create_oval(
            *self._dot_coords(0, 0), fill="#e53935", outline="")
        self._bind_stick(self.l_canvas, self.l_dot, 0, 1)

        # 오른쪽 스틱 (ax2=X, ax3=Y)
        rfrm = tk.Frame(stick_frm, bg="#1e1e1e")
        rfrm.grid(row=0, column=1, padx=16)
        tk.Label(rfrm, text="R-STICK", fg="#888888", bg="#1e1e1e",
                 font=("Arial", 8)).pack()
        self.r_canvas = tk.Canvas(rfrm, width=STICK_SIZE, height=STICK_SIZE,
                                  bg="#2a2a2a", highlightthickness=0, cursor="crosshair")
        self.r_canvas.pack()
        tk.Label(rfrm, text="ax2, ax3", fg="#666666", bg="#1e1e1e",
                 font=("Arial", 7)).pack()
        self._draw_stick_bg(self.r_canvas)
        self.r_dot = self.r_canvas.create_oval(
            *self._dot_coords(0, 0), fill="#e53935", outline="")
        self._bind_stick(self.r_canvas, self.r_dot, 2, 3)

        # 스위치 (buttons[0..3])
        SW_COLORS = ["#c62828", "#1565c0", "#2e7d32", "#e65100"]
        SW_NAMES  = ["KILL", "LAND", "MODE", "LAUNCH"]
        sw_frm = tk.Frame(root, bg="#1e1e1e")
        sw_frm.pack(pady=4)
        self.sw_labels = []
        for i, (name, col_on) in enumerate(zip(SW_NAMES, SW_COLORS)):
            lbl = tk.Label(sw_frm, text=f"{name}: OFF", width=9,
                           font=("Arial", 9, "bold"), fg="white",
                           bg="#444444", relief="groove", cursor="hand2")
            lbl.grid(row=0, column=i, padx=2)
            lbl.bind("<Button-1>", lambda e, idx=i, c=col_on, n=name: self._toggle_switch(idx, c, n))
            self.sw_labels.append(lbl)

    def _bind_stick(self, canvas, dot_id, ax_idx, ay_idx):
        def _move(event):
            cx = cy = STICK_SIZE // 2
            dx = (event.x - cx) / STICK_R
            dy = (cy - event.y) / STICK_R
            mag = math.hypot(dx, dy)
            if mag > 1.0:
                dx /= mag
                dy /= mag
            self.node.set_axis(ax_idx, dx)
            self.node.set_axis(ay_idx, dy)
            canvas.coords(dot_id, *self._dot_coords(dx, dy))

        def _release(event):
            self.node.set_axis(ax_idx, 0.0)
            self.node.set_axis(ay_idx, 0.0)
            canvas.coords(dot_id, *self._dot_coords(0.0, 0.0))

        canvas.bind("<Button-1>", _move)
        canvas.bind("<B1-Motion>", _move)
        canvas.bind("<ButtonRelease-1>", _release)

    def _toggle_switch(self, idx, col_on, name):
        val = self.node.toggle_button(idx)
        lbl = self.sw_labels[idx]
        SW_NAMES = ["KILL", "LAND", "MODE", "LAUNCH"]
        if val:
            lbl.config(text=f"{SW_NAMES[idx]}: ON", bg=col_on)
        else:
            lbl.config(text=f"{SW_NAMES[idx]}: OFF", bg="#444444")

    def _draw_stick_bg(self, canvas):
        cx = cy = STICK_SIZE // 2
        r = STICK_R
        canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                           outline="#555555", width=1, fill="#1a1a1a")
        canvas.create_line(cx - r, cy, cx + r, cy, fill="#444444")
        canvas.create_line(cx, cy - r, cx, cy + r, fill="#444444")

    def _dot_coords(self, ax, ay):
        cx = cy = STICK_SIZE // 2
        x = cx + ax * STICK_R
        y = cy - ay * STICK_R
        return x - DOT_R, y - DOT_R, x + DOT_R, y + DOT_R


def main(args=None):
    rclpy.init(args=args)
    q = queue.Queue()
    node = PanelNode(q)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    PanelGUI(root, node, q)

    ctrl_win = tk.Toplevel(root)
    ControllerGUI(ctrl_win, node)

    try:
        root.mainloop()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
