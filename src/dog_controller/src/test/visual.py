#!/usr/bin/env python3
"""
从 CSV 读取 WalkTrajectory 多周期在线重规划数据并绘制动画/静态图。

CSV 列（由 walkTest.cpp 输出）:
    t_s, pos_x, pos_z, vel_x, vel_z, cycle, stepLength_mm, totalTime_s

用法：修改 CSV_PATH 后直接运行。
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

# ====== 在这里修改 CSV 文件路径 ======
CSV_PATH = "/tmp/walk_test_output.csv"
# ====================================

def main():
    if not os.path.exists(CSV_PATH):
        raise FileNotFoundError(f"CSV not found: {CSV_PATH}")

    stem = os.path.splitext(os.path.basename(CSV_PATH))[0]
    out_path = os.path.join("test", "trajectory",f"{stem}.webp")
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    df = pd.read_csv(CSV_PATH)

    # ---- 验证列 ----
    required = ["t_s", "pos_x", "pos_z", "vel_x", "vel_z", "cycle", "stepLength_mm", "totalTime_s"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Missing columns: {missing}. Got: {list(df.columns)}")

    t = df["t_s"].to_numpy()
    x = df["pos_x"].to_numpy()
    z = df["pos_z"].to_numpy()
    vx = df["vel_x"].to_numpy()
    vz = df["vel_z"].to_numpy()
    cycle = df["cycle"].to_numpy().astype(int)
    step_len = df["stepLength_mm"].to_numpy()

    n = len(df)
    if n < 2:
        raise ValueError("Not enough points to animate")

    # ---- 检测参数变化时刻 ----
    change_mask = np.diff(step_len, prepend=step_len[0]) != 0
    change_times = np.unique(t[change_mask])  # 参数变化时间点

    duration = t[-1] - t[0]
    print(f"Data: {n} pts, duration={duration:.3f}s, cycles={cycle.max()}, "
          f"param changes={len(change_times)}")

    # ========== 静态对比图（多周期轨迹总览） ==========
    fig_static, axes_static = plt.subplots(2, 2, figsize=(14, 8))
    fig_static.suptitle(f"{stem} — Online Replanning Test ({cycle.max()} cycles)")

    # 左上：X-Z 轨迹（按周期着色）
    ax_xz = axes_static[0, 0]
    ax_xz.set_title("Foot Trajectory (X-Z Plane, colored by cycle)")
    ax_xz.set_xlabel("pos_x (mm)")
    ax_xz.set_ylabel("pos_z (mm)")
    ax_xz.grid(True, alpha=0.3)

    unique_cycles = np.unique(cycle)
    cmap = plt.cm.viridis # type: ignore
    for c in unique_cycles:
        mask_c = cycle == c
        ax_xz.plot(x[mask_c], z[mask_c], lw=1.2, alpha=0.8,
                   color=cmap(c / max(1, unique_cycles.max())),
                   label=f"cycle {c}")
    # 标记起终点
    ax_xz.scatter(x[0], z[0], c="green", s=60, zorder=5, label="start")
    ax_xz.scatter(x[-1], z[-1], c="red", s=60, zorder=5, label="end")
    ax_xz.invert_yaxis()
    ax_xz.legend(fontsize=7, ncol=3, loc="lower left")

    # 右上：X 位置随时间变化
    ax_xt = axes_static[0, 1]
    ax_xt.set_title("pos_x vs Time")
    ax_xt.set_xlabel("time (s)")
    ax_xt.set_ylabel("pos_x (mm)")
    ax_xt.plot(t, x, lw=0.8, color="steelblue")
    ax_xt.grid(True, alpha=0.3)
    # 标参变竖线
    for ct in change_times:
        ax_xt.axvline(x=ct, color="red", linestyle="--", alpha=0.5, lw=1)

    # 左下：Z 位置随时间变化
    ax_zt = axes_static[1, 0]
    ax_zt.set_title("pos_z vs Time")
    ax_zt.set_xlabel("time (s)")
    ax_zt.set_ylabel("pos_z (mm)")
    ax_zt.plot(t, z, lw=0.8, color="forestgreen")
    ax_zt.grid(True, alpha=0.3)
    for ct in change_times:
        ax_zt.axvline(x=ct, color="red", linestyle="--", alpha=0.5, lw=1)

    # 右下：速度 + 步长随时间变化
    ax_v = axes_static[1, 1]
    ax_v.set_title("Velocity & Step Length vs Time")
    ax_v.set_xlabel("time (s)")
    ax_v.plot(t, vx, lw=0.6, alpha=0.7, color="steelblue", label="vel_x")
    ax_v.plot(t, vz, lw=0.6, alpha=0.7, color="forestgreen", label="vel_z")

    # 双Y轴：步长
    ax_sl = ax_v.twinx()
    ax_sl.plot(t, step_len, lw=0.5, alpha=0.5, color="gray", linestyle="dotted",
               label="stepLength")
    ax_sl.set_ylabel("stepLength (mm)", color="gray")
    ax_sl.tick_params(axis="y", colors="gray")

    ax_v.set_ylabel("velocity (mm/s)")
    ax_v.grid(True, alpha=0.3)
    for ct in change_times:
        ax_v.axvline(x=ct, color="red", linestyle="--", alpha=0.5, lw=1)
    # 合并图例
    lines_v, labels_v = ax_v.get_legend_handles_labels()
    lines_sl, labels_sl = ax_sl.get_legend_handles_labels()
    ax_v.legend(lines_v + lines_sl, labels_v + labels_sl, fontsize=7)

    plt.tight_layout()
    static_png = os.path.join("test", "trajectory", f"{stem}_static.png")
    fig_static.savefig(static_png, dpi=150)
    print(f"Static overview saved to: {static_png}")
    plt.close(fig_static)

    # ========== 动画（降采样 100fps） ==========
    TARGET_FPS = 100
    TARGET_INTERVAL_MS = 1000.0 / TARGET_FPS
    target_frames = max(2, int(round(duration * TARGET_FPS)))
    save_stride = max(1, n // target_frames)
    save_indices = list(range(0, n, save_stride))
    if save_indices[-1] != n - 1:
        save_indices.append(n - 1)
    save_n = len(save_indices)
    print(f"Saving animation: stride={save_stride}, {save_n} frames @{TARGET_FPS}fps")

    t_save = t[save_indices]
    x_save = x[save_indices]
    z_save = z[save_indices]
    vx_save = vx[save_indices]
    vz_save = vz[save_indices]
    cycle_save = cycle[save_indices]
    sl_save = step_len[save_indices]

    # ---- 图形 ----
    fig_anim, (ax_p, ax_tv) = plt.subplots(1, 2, figsize=(12, 5))
    fig_anim.suptitle(f"{stem} — Online Replanning ({cycle.max()} cycles, "
                      f"stepLen={np.unique(step_len).tolist()})")

    # 左：X-Z 轨迹（当前周期高亮）
    ax_p.set_title("Path in X-Z Plane")
    ax_p.set_xlabel("pos_x (mm)")
    ax_p.set_ylabel("pos_z (mm)")
    ax_p.grid(True, alpha=0.3)

    mx = (x.max() - x.min()) * 0.1 + 1e-6
    mz = (z.max() - z.min()) * 0.1 + 1e-6
    ax_p.set_xlim(x.min() - mx, x.max() + mx)
    ax_p.set_ylim(z.min() - mz, z.max() + mz)
    ax_p.invert_yaxis()

    trail_line, = ax_p.plot([], [], lw=1.2, color="steelblue", alpha=0.6, label="trail")
    cur_cycle_line, = ax_p.plot([], [], lw=2, color="orange", label="current cycle")
    dot, = ax_p.plot([], [], "ro", ms=8, zorder=5, label="now")
    ax_p.legend(loc="lower left", fontsize=7)

    # 右：速度 + 步长
    ax_tv.set_title("Velocity & Step Length")
    ax_tv.set_xlabel("time (s)")
    ax_tv.set_ylabel("velocity (mm/s)")
    ax_tv.grid(True, alpha=0.3)
    ax_tv.set_xlim(t.min(), t.max())

    ylo = min(vx.min(), vz.min())
    yhi = max(vx.max(), vz.max())
    mv = (yhi - ylo) * 0.1 + 1e-6
    ax_tv.set_ylim(ylo - mv, yhi + mv)

    line_vx, = ax_tv.plot([], [], lw=0.8, color="steelblue", label="vel_x")
    line_vz, = ax_tv.plot([], [], lw=0.8, color="forestgreen", label="vel_z")
    cursor_v = ax_tv.axvline(x=0, color="black", ls="--", alpha=0.6)
    ax_tv.legend(loc="upper left", fontsize=7)

    ax_sl_anim = ax_tv.twinx()
    ax_sl_anim.set_ylabel("stepLength (mm)", color="gray")
    ax_sl_anim.tick_params(axis="y", colors="gray")
    sl_lo = step_len.min() - 5
    sl_hi = step_len.max() + 5
    ax_sl_anim.set_ylim(sl_lo, sl_hi)
    line_sl, = ax_sl_anim.plot([], [], lw=0.5, color="gray", linestyle="dotted",
                                alpha=0.7, label="stepLen")

    # 参数变化标记竖线
    ax_sl_anim.fill_between([], sl_lo, sl_hi, alpha=0.2, color="red", linewidth=0)

    # 合并图例
    h_tv, l_tv = ax_tv.get_legend_handles_labels()
    h_sla, l_sla = ax_sl_anim.get_legend_handles_labels()
    ax_tv.legend(h_tv + h_sla, l_tv + l_sla, loc="upper left", fontsize=7)

    def init_anim():
        trail_line.set_data([], [])
        cur_cycle_line.set_data([], [])
        dot.set_data([], [])
        line_vx.set_data([], [])
        line_vz.set_data([], [])
        cursor_v.set_xdata([t[0], t[0]])
        line_sl.set_data([], [])
        return trail_line, cur_cycle_line, dot, line_vx, line_vz, cursor_v, line_sl

    def update_anim(idx):
        i = save_indices[idx]
        now_cycle = cycle_save[idx]

        # 全部轨迹（低透明）
        trail_line.set_data(x[:i + 1], z[:i + 1])

        # 当前周期高亮
        mask_cur = cycle[:i + 1] == now_cycle
        cx = x[:i + 1][mask_cur]
        cz = z[:i + 1][mask_cur]
        cur_cycle_line.set_data(cx, cz)

        # 当前点
        dot.set_data([x[i]], [z[i]])

        # 速度
        line_vx.set_data(t[:i + 1], vx[:i + 1])
        line_vz.set_data(t[:i + 1], vz[:i + 1])
        cursor_v.set_xdata([t[i], t[i]])

        # 步长
        line_sl.set_data(t[:i + 1], step_len[:i + 1])

        ax_p.set_title(f"Path in X-Z Plane (t={t[i]:.3f}s, cycle={now_cycle}, "
                       f"stepLen={step_len[i]:.0f})")

        if idx % 60 == 0:
            print(f"  Frame {idx + 1}/{save_n}...")
        return trail_line, cur_cycle_line, dot, line_vx, line_vz, cursor_v, line_sl

    ani = FuncAnimation(
        fig_anim, update_anim, frames=save_n, init_func=init_anim,
        interval=TARGET_INTERVAL_MS, blit=False,
    )
    ani.save(out_path, writer=PillowWriter(fps=TARGET_FPS))
    print(f"Animation saved: {out_path}")
    plt.close(fig_anim)
    print("Done.")


if __name__ == "__main__":
    main()