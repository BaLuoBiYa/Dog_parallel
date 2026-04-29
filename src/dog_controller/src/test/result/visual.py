#!/usr/bin/env python3
"""
从 CSV 读取轨迹数据并绘制动画，保存为 webp 格式。
直接在脚本中修改 CSV_PATH 即可使用。
"""
import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

# ====== 在这里修改 CSV 文件路径 ======
CSV_PATH = "/home/luo/Project/Horse/Dog_parallel/src/dog_controller/src/test/result/walkTrajectory_0.5.csv"
# ====================================

def main():
    if not os.path.exists(CSV_PATH):
        raise FileNotFoundError(f"CSV not found: {CSV_PATH}")

    stem = os.path.splitext(os.path.basename(CSV_PATH))[0]
    out_path = os.path.join("test", "trajectory", f"{stem}.webp")

    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    df = pd.read_csv(CSV_PATH)

    required_cols = ["t_s", "pos_x", "pos_y", "pos_z", "vel_x", "vel_z"]
    for c in required_cols:
        if c not in df.columns:
            raise ValueError(f"Missing required column: {c}")

    t = df["t_s"].to_numpy()
    x = df["pos_x"].to_numpy()
    z = df["pos_z"].to_numpy()
    vx = df["vel_x"].to_numpy()
    vz = df["vel_z"].to_numpy()

    n = len(df)
    if n < 2:
        raise ValueError("Not enough points to animate")

    # ---------- 固定 100fps 输出 ----------
    TARGET_FPS = 100
    TARGET_INTERVAL_MS = 1000.0 / TARGET_FPS       # 10ms/帧
    duration = t[-1] - t[0]
    target_frames = max(2, int(round(duration * TARGET_FPS)))
    # stride: 从原始数据中每 N 个点取 1 个作为输出帧
    save_stride = max(1, n // target_frames)
    save_indices = list(range(0, n, save_stride))
    if save_indices[-1] != n - 1:
        save_indices.append(n - 1)
    save_n = len(save_indices)
    print(f"Data: {n} pts @{n/duration:.0f}Hz, duration={duration:.3f}s")
    print(f"Saving: stride={save_stride}, {save_n} frames @{TARGET_FPS}fps")

    # 降采样后的时间、位置、速度
    t_save = t[save_indices]
    x_save = x[save_indices]
    z_save = z[save_indices]
    vx_save = vx[save_indices]
    vz_save = vz[save_indices]

    # ---------- 创建图形（较小尺寸，默认 DPI，避免内存爆炸） ----------
    fig, (ax_path, ax_time) = plt.subplots(1, 2, figsize=(10, 4))
    fig.suptitle(stem + " Animation")

    # 左图：X-Z 平面轨迹
    ax_path.set_title("Path in X-Z Plane")
    ax_path.set_xlabel("pos_x")
    ax_path.set_ylabel("pos_z")
    ax_path.grid(True, alpha=0.3)

    margin_x = (x.max() - x.min()) * 0.1 + 1e-6
    margin_z = (z.max() - z.min()) * 0.1 + 1e-6
    ax_path.set_xlim(x.min() - margin_x, x.max() + margin_x)
    ax_path.set_ylim(z.min() - margin_z, z.max() + margin_z)
    ax_path.invert_yaxis()

    line_path, = ax_path.plot([], [], lw=2, label="trail")
    point_path, = ax_path.plot([], [], "ro", ms=6, label="current")
    ax_path.legend(loc="best")

    # 右图：x/z 方向速度随时间变化
    ax_time.set_title("Velocity vs Time")
    ax_time.set_xlabel("time (s)")
    ax_time.set_ylabel("velocity")
    ax_time.grid(True, alpha=0.3)

    ymin = min(vx.min(), vz.min())
    ymax = max(vx.max(), vz.max())
    margin_v = (ymax - ymin) * 0.1 + 1e-6

    ax_time.set_xlim(t.min(), t.max())
    ax_time.set_ylim(ymin - margin_v, ymax + margin_v)

    line_vx, = ax_time.plot([], [], label="vel_x")
    line_vz, = ax_time.plot([], [], label="vel_z")
    cursor = ax_time.axvline(x=t[0], color="k", linestyle="--", alpha=0.6)
    ax_time.legend(loc="best")

    def show_frame(i, xi, zi, vxi, vzi, ti, line_p, point_p, line_vx_p, line_vz_p, cursor_p, ax):
        line_p.set_data(xi, zi)
        point_p.set_data([xi[-1]], [zi[-1]])
        line_vx_p.set_data(ti, vxi)
        line_vz_p.set_data(ti, vzi)
        cursor_p.set_xdata([ti[-1], ti[-1]])
        ax.set_title(f"Path in X-Z Plane (t={ti[-1]:.3f}s)")

    # ---------- 保存 webp（降采样帧，不使用 blit 避免额外缓存） ----------
    def init_save():
        line_path.set_data([], [])
        point_path.set_data([], [])
        line_vx.set_data([], [])
        line_vz.set_data([], [])
        cursor.set_xdata([t[0], t[0]])
        return line_path, point_path, line_vx, line_vz, cursor

    def update_save(idx):
        i = save_indices[idx]
        show_frame(i, x[: i + 1], z[: i + 1], vx[: i + 1], vz[: i + 1],
                   t[: i + 1], line_path, point_path, line_vx, line_vz, cursor, ax_path)
        if idx % 50 == 0:
            print(f"Saving frame {idx + 1}/{save_n}...")
        return line_path, point_path, line_vx, line_vz, cursor

    ani = FuncAnimation(
        fig, update_save, frames=save_n, init_func=init_save,
        interval=TARGET_INTERVAL_MS, blit=False,
    )
    ani.save(out_path, writer=PillowWriter(fps=TARGET_FPS))
    print(f"Saved animation: {out_path}")

    # ---------- 交互实时播放（关闭之前的 figure 释放内存） ----------
    plt.close(fig)
    # print("Starting interactive real-time playback...")

    # fig2, (ax_p2, ax_t2) = plt.subplots(1, 2, figsize=(10, 4))
    # fig2.suptitle(stem + " Animation (Real-time)")

    # ax_p2.set_title("Path in X-Z Plane")
    # ax_p2.set_xlabel("pos_x")
    # ax_p2.set_ylabel("pos_z")
    # ax_p2.grid(True, alpha=0.3)
    # ax_p2.set_xlim(x.min() - margin_x, x.max() + margin_x)
    # ax_p2.set_ylim(z.min() - margin_z, z.max() + margin_z)
    # ax_p2.invert_yaxis()
    # line_p2, = ax_p2.plot([], [], lw=2, label="trail")
    # point_p2, = ax_p2.plot([], [], "ro", ms=6, label="current")
    # ax_p2.legend(loc="best")

    # ax_t2.set_title("Velocity vs Time")
    # ax_t2.set_xlabel("time (s)")
    # ax_t2.set_ylabel("velocity")
    # ax_t2.grid(True, alpha=0.3)
    # ax_t2.set_xlim(t.min(), t.max())
    # ax_t2.set_ylim(ymin - margin_v, ymax + margin_v)
    # line_vx2, = ax_t2.plot([], [], label="vel_x")
    # line_vz2, = ax_t2.plot([], [], label="vel_z")
    # cursor2 = ax_t2.axvline(x=t[0], color="k", linestyle="--", alpha=0.6)
    # ax_t2.legend(loc="best")

    # # 交互播放使用原始帧（不降采样），但限制最大帧数避免 timer 爆炸
    # play_n = n
    # play_indices = list(range(0, n, max(1, n // 3000)))  # 最多 3000 帧交互
    # if play_indices[-1] != n - 1:
    #     play_indices.append(n - 1)
    # play_t = t[play_indices]

    # def schedule_play(play_idx):
    #     i = play_indices[play_idx]
    #     show_frame(i, x[: i + 1], z[: i + 1], vx[: i + 1], vz[: i + 1],
    #                t[: i + 1], line_p2, point_p2, line_vx2, line_vz2, cursor2, ax_p2)
    #     fig2.canvas.draw_idle()
    #     if play_idx + 1 < len(play_indices):
    #         next_i = play_indices[play_idx + 1]
    #         interval_ms = max(1.0, (t[next_i] - t[i]) * 1000.0)
    #         timer = fig2.canvas.new_timer(interval=int(interval_ms))
    #         timer.add_callback(lambda j=play_idx + 1: schedule_play(j))
    #         timer.start()

    # schedule_play(0)
    # plt.show()

if __name__ == "__main__":
    main()