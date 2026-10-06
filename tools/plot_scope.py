#!/usr/bin/env python3
"""
plot_scope.py —— 四轴实时示波器（上位机侧的"数字示波器"）。

四条曲线：位置 / 速度 / 电流 / 母线电压。数据来自 /hw/motor_states，
用滚动窗口显示最近 N 秒，可随时暂停、导出 PNG。

用法：
    python3 tools/plot_scope.py                  # 默认 10 秒窗口
    python3 tools/plot_scope.py --window 30 --save run1.png
    python3 tools/plot_scope.py --signals velocity,current

为什么不用 rqt_plot / PlotJuggler：
它们都要额外依赖且启动后要手点话题与字段；这个脚本把本项目关心的四个量
直接开箱画好，现场调试少一步就少一个出错机会。
"""

import argparse
import collections
import sys
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from hw_msgs.msg import MotorStateArray
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError as e:
    sys.exit(f"缺少依赖：{e}\n  需要 rclpy（ROS2 环境）与 matplotlib（pip install matplotlib）")


SIGNAL_KEYS = ["position", "velocity", "current", "vbus"]
SIGNAL_LABELS = {
    "position": "位置 (rad)",
    "velocity": "速度 (rad/s)",
    "current": "电流 (A)",
    "vbus": "母线电压 (V)",
}
MOTOR_COLORS = ["#d62728", "#1f77b4", "#2ca02c", "#ff7f0e"]   # M0红 M1蓝 M2绿 M3橙


class ScopeNode(Node):
    def __init__(self, window, fs):
        super().__init__("plot_scope")
        self.window = window
        self.maxlen = int(window * fs) + 10
        self.fs = fs
        self.lock = threading.Lock()
        self.buf = {k: {i: collections.deque(maxlen=self.maxlen) for i in range(4)}
                    for k in SIGNAL_KEYS}
        self.t = collections.deque(maxlen=self.maxlen)
        self.t0 = time.monotonic()
        self.count = 0
        self.create_subscription(MotorStateArray, "/hw/motor_states", self.on_state, 10)

    def on_state(self, msg):
        if len(msg.motors) < 4:
            return
        now = time.monotonic() - self.t0
        with self.lock:
            self.t.append(now)
            for i in range(4):
                m = msg.motors[i]
                self.buf["position"][i].append(m.position)
                self.buf["velocity"][i].append(m.velocity)
                self.buf["current"][i].append(m.current)
            self.buf["vbus"][0].append(msg.board.vbus)
            self.count += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--window", type=float, default=10.0, help="滚动窗口秒数")
    ap.add_argument("--fs", type=float, default=100.0, help="预期采样率（用于定缓冲区大小）")
    ap.add_argument("--signals", default="position,velocity,current,vbus")
    ap.add_argument("--save", help="退出前把当前画面存成 PNG")
    args = ap.parse_args()

    signals = [s.strip() for s in args.signals.split(",") if s.strip() in SIGNAL_KEYS]
    if not signals:
        sys.exit(f"--signals 里没有有效项，可选：{', '.join(SIGNAL_KEYS)}")

    rclpy.init()
    node = ScopeNode(args.window, args.fs)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    n = len(signals)
    fig, axes = plt.subplots(n, 1, sharex=True, figsize=(11, 2.4 * n), squeeze=False)
    axes = [a[0] for a in axes]
    fig.suptitle("HWB-MC4G4 四轴示波器 —— 数据源 /hw/motor_states", fontsize=11)

    paused = {"v": False}

    def on_key(event):
        if event.key == " ":
            paused["v"] = not paused["v"]
            fig.suptitle(
                ("⏸ 已暂停  " if paused["v"] else "")
                + "HWB-MC4G4 四轴示波器 —— 数据源 /hw/motor_states", fontsize=11)
    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        with node.lock:
            ts = list(node.t)
            data = {s: {i: list(node.buf[s][i]) for i in range(4)} for s in signals}
            vbus = list(node.buf["vbus"][0])

        for ax, sig in zip(axes, signals):
            if not paused["v"]:
                ax.clear()
                if sig == "vbus":
                    ax.plot(ts[-len(vbus):], vbus, color="#7f7f7f", lw=1.2,
                            label="母线电压")
                    ax.legend(loc="upper right", fontsize=8)
                else:
                    for i in range(4):
                        ax.plot(ts, data[sig][i], color=MOTOR_COLORS[i],
                                lw=1.0, label=f"M{i}")
                    if sig == "current":
                        ax.axhline(0, color="k", lw=0.4, alpha=0.4)
                    ax.legend(loc="upper right", fontsize=8, ncol=4)
                ax.set_ylabel(SIGNAL_LABELS[sig], fontsize=9)
                ax.grid(alpha=0.25)
                if ts:
                    ax.set_xlim(max(0.0, ts[-1] - node.window), ts[-1] + 0.05)
        axes[-1].set_xlabel("时间 (s)", fontsize=9)

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.tight_layout()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass

    if args.save:
        fig.savefig(args.save, dpi=130)
        print(f"已保存 {args.save}")

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
