#!/usr/bin/env python3
"""Simple Windows GUI for the 5 FPS local MJPEG recorder."""
from __future__ import annotations

import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from windows_mjpeg_recorder import MjpegRecorder


class RecorderWindow(ttk.Frame):
    def __init__(self, master: tk.Tk) -> None:
        super().__init__(master, padding=22)
        self.master = master
        self.recorder: MjpegRecorder | None = None
        self.thread: threading.Thread | None = None
        self.url = tk.StringVar(value="http://树莓派IP:5000/video_feed")
        self.output = tk.StringVar(value=str(Path.home() / "Desktop" / "Robitics" / "data" / "captures"))
        self.status = tk.StringVar(value="请先填写树莓派 IP，并选择保存位置。")
        self.grid(sticky="nsew")
        self._build()
        self.after(400, self._refresh)

    def _build(self) -> None:
        self.columnconfigure(0, weight=1)
        ttk.Label(self, text="Robitics 本地 JPG 记录器", font=("Microsoft YaHei UI", 18, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(self, text="默认每秒保存 5 张；图片只写入本机，不保存到树莓派。", foreground="#555555").grid(row=1, column=0, sticky="w", pady=(3, 18))
        ttk.Label(self, text="树莓派视频流地址").grid(row=2, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.url, width=58).grid(row=3, column=0, sticky="ew", pady=(4, 16))
        ttk.Label(self, text="保存位置（重要）", font=("Microsoft YaHei UI", 11, "bold")).grid(row=4, column=0, sticky="w")
        path_row = ttk.Frame(self); path_row.grid(row=5, column=0, sticky="ew", pady=(5, 18)); path_row.columnconfigure(0, weight=1)
        ttk.Entry(path_row, textvariable=self.output, state="readonly").grid(row=0, column=0, sticky="ew", padx=(0, 10))
        ttk.Button(path_row, text="选择保存位置…", command=self._choose).grid(row=0, column=1)
        controls = ttk.Frame(self); controls.grid(row=6, column=0, sticky="w")
        self.start_button = ttk.Button(controls, text="开始保存（5 FPS）", command=self._start)
        self.start_button.grid(row=0, column=0, padx=(0, 10))
        self.stop_button = ttk.Button(controls, text="停止保存", command=self._stop, state="disabled")
        self.stop_button.grid(row=0, column=1)
        ttk.Separator(self).grid(row=7, column=0, sticky="ew", pady=18)
        ttk.Label(self, textvariable=self.status, wraplength=520).grid(row=8, column=0, sticky="w")

    def _choose(self) -> None:
        selected = filedialog.askdirectory(title="选择 JPG 保存位置", initialdir=self.output.get())
        if selected:
            self.output.set(selected)
            self.status.set(f"已选择保存位置：{selected}")

    def _start(self) -> None:
        url = self.url.get().strip()
        if not url.startswith(("http://", "https://")) or "/video_feed" not in url:
            messagebox.showerror("视频流地址不正确", "请输入例如：http://100.80.46.54:5000/video_feed")
            return
        self.recorder = MjpegRecorder(url, Path(self.output.get()), fps=5.0, min_free_gb=5.0)
        self.thread = threading.Thread(target=self.recorder.run, name="mjpeg-recorder", daemon=True)
        self.thread.start()
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.status.set("正在连接视频流并保存到本机……")

    def _stop(self) -> None:
        if self.recorder:
            self.recorder.stop_event.set()
        self.status.set("正在停止记录器……")

    def _refresh(self) -> None:
        if self.recorder:
            if self.thread and self.thread.is_alive():
                detail = self.recorder.last_error or "正在接收视频流"
                self.status.set(f"{detail}；已保存 {self.recorder.saved} 张 JPG。")
            else:
                self.status.set(f"记录已停止；本次保存 {self.recorder.saved} 张 JPG。")
                self.start_button.configure(state="normal")
                self.stop_button.configure(state="disabled")
                self.recorder = None
        self.after(400, self._refresh)


def main() -> None:
    root = tk.Tk()
    root.title("Robitics 本地 JPG 记录器（5 FPS）")
    root.minsize(620, 360)
    RecorderWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
