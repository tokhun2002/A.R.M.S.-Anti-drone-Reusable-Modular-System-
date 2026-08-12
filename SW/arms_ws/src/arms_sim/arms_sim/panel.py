#!/usr/bin/env python3
"""
panel.py — A.R.M.S. SITL 튜닝/심판 콘솔 (tkinter GUI)  [arms_sim]

SITL 전용 개발 콘솔. /arms/command 는 발행하지 않는다 (그건 가상 조종기 =
arms_command 노드 담당). 여기서 하는 일은 전부 파라미터/서비스 세팅뿐:
  - 미션 상태 표시 (IDLE/SEARCH/LOCK/TRACK/FIRE/RTL) — 읽기전용 구독
  - 검출 모드(YOLO/HSV/ABSDIFF) 토글 → arms_detection_node 파라미터
  - Roll/Pitch 부호, PID, τ, 추력, PN 게인 등 → arms_control_node 파라미터
  - 표적(풍선/드론) 비행·정지·고도·속도·접촉반경(hit_radius) → referee 파라미터
  - RESET(추후 디벨롭) → gz 서비스 + /arms/reset_cmd

실행: ros2 run arms_sim panel
"""

import queue
import subprocess
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from arms_msgs.msg import MissionState

CTRL_NODE    = "/arms_control_node"
FUSION_NODE  = "/arms_detection_node"
REFEREE_NODE = "/balloon_referee"

WORLD = "arms_sitl"
DRONE = "arms_drone_0"
BALL  = "target_ball"

STATE_COLOR = {
    "IDLE":   "#888888",
    "SEARCH": "#f5a623",
    "LOCK":   "#f57c00",
    "TRACK":  "#d0021b",
    "FIRE":   "#ff1744",
    "RTL":    "#2979ff",
}

def _bg(fn):
    threading.Thread(target=fn, daemon=True).start()


def group(parent, title):
    """제목 달린 묶음 상자. tkinter 의 기본 그룹 위젯이 LabelFrame 이다.

    패널이 커지면서 슬라이더가 한 줄로 주욱 늘어서 어느 게 어느 계통인지
    읽히지 않았다. 여기서 묶는 기준은 '무엇을 튜닝하느냐'가 아니라
    '어느 노드의 파라미터냐'다 — 제어기(arms_control), 검출기
    (arms_detection), 심판/표적(referee). 잘못된 노드에 값을 쏘면 조용히
    아무 일도 안 일어나므로, 그 경계가 눈에 보이는 편이 낫다.
    """
    f = tk.LabelFrame(parent, text=title, fg="#9fc3e8", bg="#1e1e1e",
                      font=("Arial", 10, "bold"), bd=1, relief="groove",
                      labelanchor="nw", padx=8, pady=6)
    f.pack(fill="x", pady=(8, 0))
    return f


def ros_param_set(name, value):
    _bg(lambda: subprocess.run(
        ["ros2", "param", "set", CTRL_NODE, name, str(value)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))


def ros_param_set_node(node, name, value):
    _bg(lambda: subprocess.run(
        ["ros2", "param", "set", node, name, str(value)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))


class PanelNode(Node):
    """콘솔 노드 — 미션 상태만 구독(표시용). /arms/command 는 발행하지 않는다."""

    def __init__(self, q):
        super().__init__("arms_panel_node")
        self.q = q
        self.create_subscription(MissionState, "/arms/mission_state", self._cb_state, 10)

    def _cb_state(self, msg):
        self.q.put(msg)


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
        root.geometry("760x880")

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
        # LAUNCH/arm/kill/mode 는 가상 조종기(arms_command)로 이동 — /arms/command 발행자는
        # 조종기 하나뿐이어야 한다(두 노드가 동시에 쏘면 레이스). 여긴 튜닝/심판만.

        # ── 2열 레이아웃 ────────────────────────────────────────────────────
        cols = tk.Frame(root, bg="#1e1e1e")
        cols.pack(fill="both", expand=True, padx=(12, 4), pady=4)

        left = tk.Frame(cols, bg="#1e1e1e")
        left.grid(row=0, column=0, sticky="nw", padx=(0, 16))

        sep = tk.Frame(cols, bg="#444444", width=1)
        sep.grid(row=0, column=1, sticky="ns", padx=4)

        right = tk.Frame(cols, bg="#1e1e1e")
        right.grid(row=0, column=2, sticky="nw")

        # ── [검출] arms_detection_node ──────────────────────────────────────
        g_det = group(left, "검출  ·  arms_detection")
        mfrm = tk.Frame(g_det, bg="#1e1e1e")
        mfrm.pack()
        # 노드 쪽 declare_parameter 기본값과 일치시킨다 (셋 다 True). 예전엔 여기서만
        # yolo=False 라 패널은 OFF 로 그려놓고 노드는 켜져 있었고, YOLO 버튼을 처음
        # 누르면 켜지는 게 아니라 꺼졌다. _push_defaults 가 시작할 때 실제로 밀어준다.
        self.det_on = {"hsv": True, "yolo": True, "absdiff": True}
        self.det_btns = {}
        for key, label, col in [("hsv", "HSV", "#2e7d32"),
                                 ("yolo", "YOLO", "#6a1b9a"),
                                 ("absdiff", "ABSDIFF", "#0277bd")]:
            b = tk.Button(mfrm, text=label, width=9, font=("Arial", 9, "bold"),
                          command=lambda k=key: self.toggle_det(k))
            b.grid(row=0, column=len(self.det_btns), padx=2)
            self.det_btns[key] = (b, col)
        self._refresh_det_btns()
        tk.Label(g_det, text="눌린 것 = ON. 여러 개 켜면 융합 검출.",
                 fg="#777777", bg="#1e1e1e", font=("Arial", 8)).pack()

        # ── [제어] 축 부호 + PID ────────────────────────────────────────────
        g_pid = group(left, "제어 PID  ·  arms_control")
        sfrm = tk.Frame(g_pid, bg="#1e1e1e")
        sfrm.pack(pady=(0, 2))
        self.roll_btn = tk.Button(sfrm, text="Roll sign: +", width=13, command=self.flip_roll)
        self.roll_btn.grid(row=0, column=0, padx=3)
        self.pitch_btn = tk.Button(sfrm, text="Pitch sign: +", width=13, command=self.flip_pitch)
        self.pitch_btn.grid(row=0, column=1, padx=3)

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
            g_pid, "Roll PID  [px→deg/s]", [455, 0.0, 3.5],
            ["control.roll_pid.kp", "control.roll_pid.ki", "control.roll_pid.kd"])

        self.pitch_kp, self.pitch_ki, self.pitch_kd = make_pid_group(
            g_pid, "Pitch PID  [px→deg/s]", [455, 0.0, 3.5],
            ["control.pitch_pid.kp", "control.pitch_pid.ki", "control.pitch_pid.kd"])

        # ── [제어] 추력 ─────────────────────────────────────────────────────
        g_thr = group(left, "추력  ·  arms_control")
        tk.Label(g_thr, text="상승 추력 (track_throttle)", fg="#aaaaaa",
                 bg="#1e1e1e", font=("Arial", 9)).pack(anchor="w")
        thr_row = tk.Frame(g_thr, bg="#1e1e1e")
        thr_row.pack(anchor="w")
        self.thr_entry = tk.Entry(thr_row, width=7, bg="#2e2e2e", fg="white",
                                  insertbackground="white", relief="flat")
        self.thr_entry.insert(0, "0.85")   # yaml track_throttle 과 일치 (1.05kg)
        self.thr_entry.grid(row=0, column=0, padx=(4, 6))
        tk.Button(thr_row, text="적용", bg="#455a64", fg="white",
                  activebackground="#607d8b", relief="flat",
                  command=lambda: ros_param_set(
                      "control.track_throttle", float(self.thr_entry.get()))
                  ).grid(row=0, column=1)

        # ── [표적] referee ──────────────────────────────────────────────────
        g_tgt = group(right, "표적  ·  referee")

        # 표적 종류 토글 (풍선 ⇄ 드론) → referee 'target' 파라미터. 선택된 것만 비행.
        self.target_is_drone = False   # 기본: 풍선
        self.target_btn = tk.Button(g_tgt, text="종류: 풍선", width=20,
                                    font=("Arial", 10, "bold"), bg="#8d6e63", fg="white",
                                    command=self.toggle_target)
        self.target_btn.pack(pady=(0, 6))

        bbtn = tk.Frame(g_tgt, bg="#1e1e1e")
        bbtn.pack(pady=2)
        self.ball_fly_btn = tk.Button(bbtn, text="비행 시작", width=12,
                                      font=("Arial", 10, "bold"),
                                      command=self.ball_start)
        self.ball_fly_btn.grid(row=0, column=0, padx=3, pady=2)
        self.ball_stop_btn = tk.Button(bbtn, text="정지", width=8,
                                       command=self.ball_stop)
        self.ball_stop_btn.grid(row=0, column=1, padx=3, pady=2)

        self.ball_state_lbl = tk.Label(g_tgt, text="상태: ? (버튼으로 설정)",
                                       fg="white", bg="#555555",
                                       font=("Arial", 10, "bold"), width=26)
        self.ball_state_lbl.pack(pady=(2, 6), ipady=4)
        self._refresh_ball_btns()

        self.alt = tk.Scale(g_tgt, from_=5, to=100, resolution=1, orient="horizontal",
                            length=180, bg="#1e1e1e", fg="white", label="타겟 고도 [m]",
                            highlightthickness=0, troughcolor="#444")
        self.alt.set(42)
        self.alt.pack(pady=(2, 0))
        self.alt.bind("<ButtonRelease-1>",
                      lambda e: ros_param_set_node(REFEREE_NODE, "alt", float(self.alt.get())))

        # 타겟 속도 슬라이더 (근접/먼거리 통일 — 단일 속도)
        self.ball_speed = tk.Scale(g_tgt, from_=0.2, to=40, resolution=0.2, orient="horizontal",
                                   length=180, bg="#1e1e1e", fg="white", label="타겟 속도 [m/s]",
                                   highlightthickness=0, troughcolor="#444")
        self.ball_speed.set(1.6)
        self.ball_speed.pack(pady=(2, 0))
        self.ball_speed.bind("<ButtonRelease-1>",
                             lambda e: ros_param_set_node(REFEREE_NODE, "speed", float(self.ball_speed.get())))

        # ── [유도] arms_control ─────────────────────────────────────────────
        g_guid = group(right, "유도  ·  arms_control")
        # 0=기본 추적(각속도 PID 추종), 1=PN(비례항법). 기본은 기본 추적.
        self._guidance_mode = 0
        self.guid_btn = tk.Button(g_guid, text="방식: 기본 추적", font=("Arial", 11, "bold"),
                                  bg="#37474f", fg="white", activebackground="#546e7a",
                                  command=self.toggle_guidance)
        self.guid_btn.pack(pady=(0, 4), fill="x")

        # 예측 조준(lead) — 움직이는 공의 미래 위치를 겨냥 (control.lead_gain)
        self.lead = tk.Scale(g_guid, from_=0.0, to=1.5, resolution=0.05, orient="horizontal",
                             length=180, bg="#1e1e1e", fg="white", label="예측 조준 lead (기본 추적)",
                             highlightthickness=0, troughcolor="#444")
        self.lead.set(0.0)
        self.lead.pack(pady=(2, 0))
        self.lead.bind("<ButtonRelease-1>",
                       lambda e: ros_param_set("control.lead_gain", float(self.lead.get())))

        self.pn_nav = tk.Scale(g_guid, from_=0, to=525, resolution=5, orient="horizontal",
                               length=180, bg="#1e1e1e", fg="white", label="PN 항법이득 (PN 일때)",
                               highlightthickness=0, troughcolor="#444")
        self.pn_nav.set(175)
        self.pn_nav.pack(pady=(2, 0))
        self.pn_nav.bind("<ButtonRelease-1>",
                         lambda e: ros_param_set("control.pn_nav_gain", float(self.pn_nav.get())))
        self.pn_center = tk.Scale(g_guid, from_=0.0, to=350.0, resolution=0.5, orient="horizontal",
                                  length=180, bg="#1e1e1e", fg="white", label="PN 중심유지 (PN 일때)",
                                  highlightthickness=0, troughcolor="#444")
        self.pn_center.set(52.5)
        self.pn_center.pack(pady=(2, 0))
        self.pn_center.bind("<ButtonRelease-1>",
                            lambda e: ros_param_set("control.pn_center_gain", float(self.pn_center.get())))

        # ── [충돌 판정] 발사 조건 + 심판 ────────────────────────────────────
        g_fire = group(right, "충돌 판정")
        # 발사 τ = 충돌까지 시간 임계(비전 looming). 작을수록 접촉 직전에 발사.
        self.tau_fire = tk.Scale(g_fire, from_=0.1, to=1.0, resolution=0.05, orient="horizontal",
                                 length=180, bg="#1e1e1e", fg="white", label="발사 τ [s] (충돌까지)",
                                 highlightthickness=0, troughcolor="#444")
        self.tau_fire.set(0.3)
        self.tau_fire.pack()
        self.tau_fire.bind("<ButtonRelease-1>",
                           lambda e: ros_param_set("mission.tau_fire_sec", float(self.tau_fire.get())))
        # 접촉반경 = 심판 직격(kinetic) 판정 거리. 드론-표적 중심거리<이 값 → 명중(/arms/hit).
        #   그물 포획이 아니라 직격이라 실제 표면접촉(풍선1.0+드론0.3≈1.3m). 빠른 표적은 살짝↑.
        self.hit_radius = tk.Scale(right, from_=0.5, to=5, resolution=0.1, orient="horizontal",
                                   length=150, bg="#1e1e1e", fg="white", label="접촉반경 [m] (직격 판정)",
                                   highlightthickness=0, troughcolor="#444")
        self.hit_radius.set(1.3)
        self.hit_radius.pack(pady=(6, 0))
        self.hit_radius.bind("<ButtonRelease-1>",
                             lambda e: ros_param_set_node(REFEREE_NODE, "hit_radius", float(self.hit_radius.get())))

        self._push_defaults()
        self._poll()

    def _push_defaults(self):
        """패널이 그려놓은 초기 상태를 실제 노드로 한 번 밀어준다.

        이게 없으면 패널은 '표시'만 하고 노드는 자기 기본값대로 돌아, 화면과
        실제가 조용히 어긋난다. 실제로 그랬다: 패널은 YOLO OFF 로 그렸는데
        arms_detection_node 의 use_yolo 기본값은 True 였다.
        토글 버튼 계열(검출 3종, 유도 방식)만 민다 — 슬라이더/입력란은 사용자가
        '적용'을 누르거나 드래그를 놓을 때 나가는 게 맞다.
        """
        for key, on in self.det_on.items():
            ros_param_set_node(FUSION_NODE, f"use_{key}", str(on).lower())
        ros_param_set("control.guidance_mode", self._guidance_mode)

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

    # ---- 표적 종류 전환 (풍선 ↔ 드론) → referee 'target' 파라미터 ----
    def toggle_target(self):
        self.target_is_drone = not self.target_is_drone
        if self.target_is_drone:
            self.target_btn.config(text="종류: 드론", bg="#37474f")
            ros_param_set_node(REFEREE_NODE, "target", "drone")
        else:
            self.target_btn.config(text="종류: 풍선", bg="#8d6e63")
            ros_param_set_node(REFEREE_NODE, "target", "balloon")

    # ---- 유도 방식 전환 (기본 추적 ↔ PN) ----
    def toggle_guidance(self):
        self._guidance_mode = 1 if self._guidance_mode == 0 else 0
        ros_param_set("control.guidance_mode", self._guidance_mode)
        if self._guidance_mode == 1:
            self.guid_btn.config(text="방식: PN(비례항법)", bg="#6a1b9a")
        else:
            self.guid_btn.config(text="방식: 기본 추적", bg="#37474f")

    def ball_start(self):
        # enabled=true → referee 가 정지했던 자리(t)에서 재개
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
            self.ball_state_lbl.config(text="타겟 상태: 비행 중 (움직임)", bg="#9c27b0")
            self.ball_fly_btn.config(bg="#9c27b0", fg="white", relief="sunken")
            self.ball_stop_btn.config(bg="#333333", fg="#cccccc", relief="raised")
        elif flying is False:
            self.ball_state_lbl.config(text="타겟 상태: 정지 (제자리, 재개가능)", bg="#2e7d32")
            self.ball_fly_btn.config(bg="#333333", fg="#cccccc", relief="raised")
            self.ball_stop_btn.config(bg="#2e7d32", fg="white", relief="sunken")
        else:
            self.ball_state_lbl.config(text="타겟 상태: ? (버튼을 눌러 설정)", bg="#555555")
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
                             f"lock: {msg.lock_elapsed_sec:.1f}s")
        except queue.Empty:
            pass
        self.state_lbl.after(50, self._poll)


def main(args=None):
    rclpy.init(args=args)
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
