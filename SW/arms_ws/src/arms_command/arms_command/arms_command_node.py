#!/usr/bin/env python3
"""
arms_command_node.py — A.R.M.S. SITL 가상 조종기 (tkinter GUI)

실기체의 물리 조종기(ESP32 → arms_command_hw_node)에 대응하는 SITL 쌍둥이.
오직 /arms/command (sensor_msgs/Joy) 만 발행한다:
  - 스틱 드래그        → axes  (roll/pitch/throttle/yaw)
  - KILL/ARM/MODE 버튼 → buttons[0..2]
  - LAUNCH 버튼        → buttons[3] (수동 TRACK 전환, 순간 눌림)

튜닝/심판 콘솔은 arms_sim 패키지의 panel 노드로 분리됨.
(발행자는 이 노드 하나뿐 — 두 노드가 /arms/command 를 동시에 쏘면 레이스가 남)

실행: ros2 run arms_command arms_command_node
"""

import math
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# 스틱 캔버스 크기 (크게)
STICK_SIZE = 120
STICK_R    = 52
DOT_R      = 12


class ControllerNode(Node):
    """가상 조종기 노드 — /arms/command (Joy) 만 발행."""

    def __init__(self):
        super().__init__("arms_command_node")
        self._joy_pub = self.create_publisher(Joy, "/arms/command", 10)
        self._axes = [0.0, 0.0, 0.0, 0.0]
        self._buttons = [0, 0, 0, 0]
        self.create_timer(0.05, self._publish_joy)  # 20 Hz

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

    def set_button(self, idx, val):
        self._buttons[idx] = 1 if val else 0
        return self._buttons[idx]

    def fire_launch(self):
        self._buttons[3] = 1

    def release_launch(self):
        self._buttons[3] = 0


class ControllerGUI:
    """가상 조종기 창 — 스틱 2개 + KILL/ARM/MODE 버튼 + LAUNCH."""

    def __init__(self, root, node):
        self.root = root
        self.node = node

        root.title("Controller Input")
        root.configure(bg="#1e1e1e")
        root.geometry("460x560")

        tk.Label(root, text="CONTROLLER INPUT", fg="#dddddd", bg="#1e1e1e",
                 font=("Arial", 15, "bold")).pack(pady=(12, 0))
        tk.Label(root, text="스틱: 드래그 (놓으면 중앙 복귀 / throttle 은 그 자리 유지)",
                 fg="#777777", bg="#1e1e1e", font=("Arial", 9)).pack()

        # ── 스틱 2개 ──────────────────────────────────────────────────────────
        stick_frm = tk.Frame(root, bg="#1e1e1e")
        stick_frm.pack(pady=12)

        # 왼쪽 스틱 (ax0=X=yaw, ax1=Y=throttle)
        #   throttle 초기값 = 중앙(0.0): PX4 Altitude 모드에서 중앙 = 고도유지(호버).
        #   → 자동에서 수동으로 넘겨도 스틱 안 건드리면 그 자리 호버.
        self._thr_hold = 0.0
        self.node.set_axis(1, 0.0)
        lfrm = tk.Frame(stick_frm, bg="#1e1e1e")
        lfrm.grid(row=0, column=0, padx=24)
        tk.Label(lfrm, text="L-STICK", fg="#999999", bg="#1e1e1e",
                 font=("Arial", 11, "bold")).pack()
        self.l_canvas = tk.Canvas(lfrm, width=STICK_SIZE, height=STICK_SIZE,
                                  bg="#2a2a2a", highlightthickness=0, cursor="crosshair")
        self.l_canvas.pack()
        tk.Label(lfrm, text="yaw, throttle", fg="#666666", bg="#1e1e1e",
                 font=("Arial", 8)).pack()
        self._draw_stick_bg(self.l_canvas)
        self.l_dot = self.l_canvas.create_oval(
            *self._dot_coords(0, self._thr_hold), fill="#e53935", outline="")
        self._bind_throttle_stick(self.l_canvas, self.l_dot, 0, 1)

        # 오른쪽 스틱 (ax2=X=roll, ax3=Y=pitch)
        rfrm = tk.Frame(stick_frm, bg="#1e1e1e")
        rfrm.grid(row=0, column=1, padx=24)
        tk.Label(rfrm, text="R-STICK", fg="#999999", bg="#1e1e1e",
                 font=("Arial", 11, "bold")).pack()
        self.r_canvas = tk.Canvas(rfrm, width=STICK_SIZE, height=STICK_SIZE,
                                  bg="#2a2a2a", highlightthickness=0, cursor="crosshair")
        self.r_canvas.pack()
        tk.Label(rfrm, text="roll, pitch", fg="#666666", bg="#1e1e1e",
                 font=("Arial", 8)).pack()
        self._draw_stick_bg(self.r_canvas)
        self.r_dot = self.r_canvas.create_oval(
            *self._dot_coords(0, 0), fill="#e53935", outline="")
        self._bind_stick(self.r_canvas, self.r_dot, 2, 3)

        # ── 스위치 (큼직하게): KILL / ARM / MODE ──────────────────────────────
        sw_frm = tk.Frame(root, bg="#1e1e1e")
        sw_frm.pack(pady=(8, 4))
        self.kill_btn = tk.Button(sw_frm, text="KILL", width=8, height=2,
                                  font=("Arial", 14, "bold"), bg="#444444", fg="#cccccc",
                                  activebackground="#c62828", command=self._toggle_kill)
        self.kill_btn.grid(row=0, column=0, padx=5)
        self.arm_btn = tk.Button(sw_frm, text="DISARM", width=8, height=2,
                                 font=("Arial", 14, "bold"), bg="#444444", fg="#cccccc",
                                 activebackground="#1565c0", command=self._toggle_arm)
        self.arm_btn.grid(row=0, column=1, padx=5)
        self.mode_btn = tk.Button(sw_frm, text="모드: 자동", width=10, height=2,
                                  font=("Arial", 14, "bold"), bg="#444444", fg="#cccccc",
                                  activebackground="#2e7d32", command=self._toggle_mode)
        self.mode_btn.grid(row=0, column=2, padx=5)

        # ── LAUNCH (3버튼 아래, 크게) ─────────────────────────────────────────
        self.launch_btn = tk.Button(root, text="🚀 LAUNCH (→TRACK)", height=2,
                                    font=("Arial", 17, "bold"), bg="#d0021b", fg="white",
                                    activebackground="#ff1744", command=self._on_launch)
        self.launch_btn.pack(pady=(8, 14), fill="x", padx=28)

    # ---- 스위치 핸들러 (토글 + 색/텍스트 갱신) ----
    def _toggle_kill(self):
        on = self.node.toggle_button(0)
        self.kill_btn.config(text="■ KILL" if on else "KILL",
                             bg="#000000" if on else "#444444",
                             fg="#ff4444" if on else "#cccccc")

    def _toggle_arm(self):
        on = self.node.toggle_button(1)
        self.arm_btn.config(text="● ARM" if on else "DISARM",
                            bg="#d0021b" if on else "#444444",
                            fg="white" if on else "#cccccc")

    def _toggle_mode(self):
        on = self.node.toggle_button(2)
        self.mode_btn.config(text="모드: 수동" if on else "모드: 자동",
                             bg="#1565c0" if on else "#444444",
                             fg="white" if on else "#cccccc")

    def _on_launch(self):
        # 순간 눌림: buttons[3]=1 잠깐 → 300ms 후 해제 (arms_control LOCK→TRACK 트리거)
        self.node.fire_launch()
        self.root.after(300, self.node.release_launch)

    # ---- 스틱 바인딩/그리기 ----
    def _bind_throttle_stick(self, canvas, dot_id, ax_x, ay_thr):
        """왼쪽 스틱: 가로(yaw)는 놓으면 중앙 복귀, 세로(throttle)는 그 자리 유지."""
        def _move(event):
            cx = cy = STICK_SIZE // 2
            dx = max(-1.0, min(1.0, (event.x - cx) / STICK_R))
            dy = max(-1.0, min(1.0, (cy - event.y) / STICK_R))
            self.node.set_axis(ax_x, dx)
            self.node.set_axis(ay_thr, dy)
            self._thr_hold = dy
            canvas.coords(dot_id, *self._dot_coords(dx, dy))

        def _release(event):
            self.node.set_axis(ax_x, 0.0)   # yaw 만 중앙 복귀
            canvas.coords(dot_id, *self._dot_coords(0.0, self._thr_hold))

        canvas.bind("<Button-1>", _move)
        canvas.bind("<B1-Motion>", _move)
        canvas.bind("<ButtonRelease-1>", _release)

    def _bind_stick(self, canvas, dot_id, ax_idx, ay_idx):
        """오른쪽 스틱: 놓으면 중앙 복귀."""
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
    node = ControllerNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    ControllerGUI(root, node)
    try:
        root.mainloop()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
