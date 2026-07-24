#!/usr/bin/env python3
"""balloon_launcher_panel.py — 공(적기) 발사 전용 미니 패널"""
import subprocess
import threading
import tkinter as tk

REFEREE = "/balloon_referee"

def set_enabled(val: bool):
    def _run():
        subprocess.run(
            ["ros2", "param", "set", REFEREE, "enabled", "true" if val else "false"],
            capture_output=True)
    threading.Thread(target=_run, daemon=True).start()

def launch_ball():
    set_enabled(False)
    root.after(300, lambda: set_enabled(True))
    status.config(text="🚀 발사됨!", fg="#00ff88")

def stop_ball():
    set_enabled(False)
    status.config(text="⏹ 정지됨", fg="#ff8888")

root = tk.Tk()
root.title("공 발사")
root.configure(bg="#1e1e1e")
root.geometry("260x200")

tk.Label(root, text="적기(공) 발사", font=("Arial", 14, "bold"),
         fg="white", bg="#1e1e1e").pack(pady=(16, 10))

tk.Button(root, text="🚀 공 발사", font=("Arial", 13, "bold"),
          bg="#c0392b", fg="white", activebackground="#e74c3c",
          command=launch_ball, height=2).pack(pady=6, fill="x", padx=24)

tk.Button(root, text="⏹ 공 정지", font=("Arial", 11),
          bg="#444", fg="white", activebackground="#666",
          command=stop_ball, height=1).pack(pady=6, fill="x", padx=24)

status = tk.Label(root, text="대기 중", font=("Arial", 11),
                  fg="#aaaaaa", bg="#1e1e1e")
status.pack(pady=12)

root.mainloop()
