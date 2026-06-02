#!/usr/bin/env python3
"""
通用可视化脚本 — 支持 walkTest / standTest / jumpTest 输出的 CSV。

walkTest / standTest CSV 列格式:
    t_s, pos_x, pos_z, vel_x, vel_z, cycle, stepLength_mm, totalTime_s

jumpTest CSV 列格式:
    t_s, phase, pos_x, pos_z, vel_x, vel_z, kp_x, kp_z, kd_x, kd_z, tau_x, tau_z
    其中 phase 为字符串: CROUCH, PUSH, FLIGHT, ABSORPTION, RECOVER, FINISHED

用法：修改 CSV_PATH 后直接运行。
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

# ====== 在这里修改 CSV 文件路径 ======
CSV_PATH = "/tmp/jumpTest.csv"
# ====================================

JUMP_PHASE_NAMES = ["CROUCH", "PUSH", "FLIGHT", "ABSORPTION", "RECOVER", "FINISHED"]
JUMP_PHASE_COLORS = {
    "CROUCH":     "#1f77b4",  # blue
    "PUSH":       "#ff7f0e",  # orange
    "FLIGHT":     "#2ca02c",  # green
    "ABSORPTION": "#d62728",  # red
    "RECOVER":    "#9467bd",  # purple
    "FINISHED":   "#7f7f7f",  # gray
}

def is_jump_format(df):
    """Return True if CSV is jumpTest format (string phase column)."""
    return "phase" in df.columns and "tau_x" in df.columns

def main():
    if not os.path.exists(CSV_PATH):
        raise FileNotFoundError(f"CSV not found: {CSV_PATH}")

    stem = os.path.splitext(os.path.basename(CSV_PATH))[0]
    out_dir = os.path.join("test", "trajectory")
    os.makedirs(out_dir, exist_ok=True)

    df = pd.read_csv(CSV_PATH)
    n = len(df)
    if n < 2:
        raise ValueError("Not enough points")

    t = df["t_s"].to_numpy()
    x = df["pos_x"].to_numpy()
    z = df["pos_z"].to_numpy()
    vx = df["vel_x"].to_numpy()
    vz = df["vel_z"].to_numpy()

    # ── 检测格式 ──────────────────────────────────────────
    if is_jump_format(df):
        run_jump(df, t, x, z, vx, vz, stem, out_dir)
    else:
        run_legacy(df, t, x, z, vx, vz, stem, out_dir)

# ═══════════════════════════════════════════════════════════════
#  jumpTest 可视化
# ═══════════════════════════════════════════════════════════════
def run_jump(df, t, x, z, vx, vz, stem, out_dir):
    n = len(df)
    duration = t[-1] - t[0]

    phase_raw = df["phase"].to_numpy()
    kp_x = df["kp_x"].to_numpy()
    kp_z = df["kp_z"].to_numpy()
    kd_x = df["kd_x"].to_numpy()
    kd_z = df["kd_z"].to_numpy()
    tau_x = df["tau_x"].to_numpy()
    tau_z = df["tau_z"].to_numpy()

    # 编码 phase 字符串为整数
    phase_map = {name: i for i, name in enumerate(JUMP_PHASE_NAMES)}
    phase_id = np.array([phase_map.get(p, -1) for p in phase_raw], dtype=int)
    unique_phases = sorted(set(phase_id[phase_id >= 0]))

    # 相位边界
    phase_edges = np.where(np.diff(phase_id) != 0)[0]
    phase_times = [t[0]] + [t[i + 1] for i in phase_edges] + [t[-1]]
    phase_labels = [JUMP_PHASE_NAMES[phase_id[0]]]
    for i in phase_edges:
        phase_labels.append(JUMP_PHASE_NAMES[phase_id[i + 1]])

    print(f"[Jump] Data: {n} pts, duration={duration:.3f}s, "
          f"phases={'→'.join(phase_labels)}")

    # ========== 静态总览图 (2×3) ==========
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(f"{stem} — JumpTrajectory: {' → '.join(phase_labels)}",
                 fontsize=13, fontweight="bold")

    # (0,0) X-Z 轨迹（按 phase 着色）
    ax_xz = axes[0, 0]
    ax_xz.set_title("Foot Trajectory (X-Z Plane)")
    ax_xz.set_xlabel("pos_x (mm)")
    ax_xz.set_ylabel("pos_z (mm)")
    ax_xz.grid(True, alpha=0.3)
    for pid in unique_phases:
        mask = phase_id == pid
        name = JUMP_PHASE_NAMES[pid]
        ax_xz.plot(x[mask], z[mask], lw=1.5, alpha=0.85,
                   color=JUMP_PHASE_COLORS[name], label=name)
    ax_xz.scatter(x[0], z[0], c="green", s=80, zorder=5, marker="o", label="start")
    ax_xz.scatter(x[-1], z[-1], c="red", s=80, zorder=5, marker="s", label="end")
    ax_xz.invert_yaxis()
    ax_xz.legend(fontsize=7, ncol=3, loc="lower left")

    # 在 X-Z 图上标出 phase 切换点
    for ei in phase_edges:
        ax_xz.scatter(x[ei], z[ei], c="black", s=25, zorder=4, marker="x")

    # (0,1) pos_x vs time
    ax_xt = axes[0, 1]
    ax_xt.set_title("pos_x vs Time")
    ax_xt.set_xlabel("time (s)")
    ax_xt.set_ylabel("pos_x (mm)")
    ax_xt.grid(True, alpha=0.3)
    _draw_phase_regions(ax_xt, t, phase_id, phase_times, phase_labels)

    # (0,2) pos_z vs time
    ax_zt = axes[0, 2]
    ax_zt.set_title("pos_z vs Time")
    ax_zt.set_xlabel("time (s)")
    ax_zt.set_ylabel("pos_z (mm)")
    ax_zt.grid(True, alpha=0.3)
    _draw_phase_regions(ax_zt, t, phase_id, phase_times, phase_labels)

    # (1,0) Velocity
    ax_v = axes[1, 0]
    ax_v.set_title("Velocity vs Time")
    ax_v.set_xlabel("time (s)")
    ax_v.set_ylabel("velocity (mm/s)")
    ax_v.grid(True, alpha=0.3)
    ax_v.plot(t, vx, lw=0.8, color="steelblue", label="vel_x")
    ax_v.plot(t, vz, lw=0.8, color="forestgreen", label="vel_z")
    ax_v.legend(fontsize=7)
    for pi in phase_edges:
        ax_v.axvline(x=t[pi], color="black", linestyle=":", alpha=0.5, lw=1)

    # (1,1) PD gains
    ax_pd = axes[1, 1]
    ax_pd.set_title("PD Gains vs Time")
    ax_pd.set_xlabel("time (s)")
    ax_pd.set_ylabel("kp")
    ax_pd.grid(True, alpha=0.3)
    ax_pd.plot(t, kp_x, lw=0.8, color="steelblue", label="kp_x")
    ax_pd.plot(t, kp_z, lw=0.8, color="orange", label="kp_z")
    ax_kd = ax_pd.twinx()
    ax_kd.set_ylabel("kd", color="gray")
    ax_kd.plot(t, kd_x, lw=0.6, ls="--", color="lightsteelblue", label="kd_x")
    ax_kd.plot(t, kd_z, lw=0.6, ls="--", color="navajowhite", label="kd_z")
    ax_kd.tick_params(axis="y", colors="gray")
    h1, l1 = ax_pd.get_legend_handles_labels()
    h2, l2 = ax_kd.get_legend_handles_labels()
    ax_pd.legend(h1 + h2, l1 + l2, fontsize=7)
    for pi in phase_edges:
        ax_pd.axvline(x=t[pi], color="black", linestyle=":", alpha=0.5, lw=1)

    # (1,2) Feedforward torque
    ax_tau = axes[1, 2]
    ax_tau.set_title("Feedforward Torque vs Time")
    ax_tau.set_xlabel("time (s)")
    ax_tau.set_ylabel("tau (N)")
    ax_tau.grid(True, alpha=0.3)
    ax_tau.plot(t, tau_x, lw=0.8, color="steelblue", label="tau_x")
    ax_tau.plot(t, tau_z, lw=0.8, color="forestgreen", label="tau_z")
    ax_tau.legend(fontsize=7)
    for pi in phase_edges:
        ax_tau.axvline(x=t[pi], color="black", linestyle=":", alpha=0.5, lw=1)

    plt.tight_layout()
    static_png = os.path.join(out_dir, f"{stem}_static.png")
    fig.savefig(static_png, dpi=150)
    print(f"Static overview saved to: {static_png}")
    plt.close(fig)

    # ========== 动画 ==========
    TARGET_FPS = 100
    TARGET_INTERVAL_MS = 1000.0 / TARGET_FPS
    target_frames = max(2, int(round(duration * TARGET_FPS)))
    save_stride = max(1, n // target_frames)
    save_indices = list(range(0, n, save_stride))
    if save_indices[-1] != n - 1:
        save_indices.append(n - 1)
    save_n = len(save_indices)
    print(f"Saving animation: stride={save_stride}, {save_n} frames @{TARGET_FPS}fps")

    fig_anim, ((ax_p, ax_tau_a), (ax_xz_a, ax_vv)) = plt.subplots(
        2, 2, figsize=(14, 10))
    fig_anim.suptitle(f"{stem} — JumpTrajectory: {' → '.join(phase_labels)}",
                      fontsize=12, fontweight="bold")

    # 左上: X-Z 轨迹动画
    ax_p.set_title("Path in X-Z Plane")
    ax_p.set_xlabel("pos_x (mm)")
    ax_p.set_ylabel("pos_z (mm)")
    ax_p.grid(True, alpha=0.3)
    mx = (x.max() - x.min()) * 0.15 + 5
    mz = (z.max() - z.min()) * 0.15 + 5
    ax_p.set_xlim(x.min() - mx, x.max() + mx)
    ax_p.set_ylim(z.min() - mz, z.max() + mz)
    ax_p.invert_yaxis()
    trail_line, = ax_p.plot([], [], lw=1.2, color="steelblue", alpha=0.5, label="trail")
    cur_phase_line, = ax_p.plot([], [], lw=2.5, label="cur phase")
    dot, = ax_p.plot([], [], "ro", ms=10, zorder=5, label="now")
    ax_p.legend(loc="lower left", fontsize=7)

    # 右上: 力/力矩 (tau) 实时
    ax_tau_a.set_title("Feedforward Torque")
    ax_tau_a.set_xlabel("time (s)")
    ax_tau_a.set_ylabel("tau (N)")
    ax_tau_a.grid(True, alpha=0.3)
    ax_tau_a.set_xlim(t.min(), t.max())
    ty_lo = min(tau_x.min(), tau_z.min()) * 1.3
    ty_hi = max(tau_x.max(), tau_z.max()) * 1.3
    if abs(ty_hi - ty_lo) < 1e-6:
        ty_lo, ty_hi = -1, 1
    ax_tau_a.set_ylim(ty_lo, ty_hi)
    line_tau_x, = ax_tau_a.plot([], [], lw=0.8, color="steelblue", label="tau_x")
    line_tau_z, = ax_tau_a.plot([], [], lw=0.8, color="forestgreen", label="tau_z")
    cursor_tau = ax_tau_a.axvline(x=0, color="black", ls="--", alpha=0.6)
    ax_tau_a.legend(loc="upper left", fontsize=7)

    # 左下: pos vs time 子图 (X + Z 叠加)
    ax_xz_a.set_title("pos vs Time")
    ax_xz_a.set_xlabel("time (s)")
    ax_xz_a.set_ylabel("pos (mm)")
    ax_xz_a.grid(True, alpha=0.3)
    ax_xz_a.set_xlim(t.min(), t.max())
    z_lo = min(x.min(), z.min()) - 5
    z_hi = max(x.max(), z.max()) + 5
    ax_xz_a.set_ylim(z_lo, z_hi)
    line_px, = ax_xz_a.plot([], [], lw=0.8, color="steelblue", label="pos_x")
    line_pz, = ax_xz_a.plot([], [], lw=0.8, color="forestgreen", label="pos_z")
    cursor_xz = ax_xz_a.axvline(x=0, color="black", ls="--", alpha=0.6)
    ax_xz_a.legend(loc="upper left", fontsize=7)

    # 右下: PD gains 实时
    ax_vv.set_title("PD Gains")
    ax_vv.set_xlabel("time (s)")
    ax_vv.set_ylabel("kp")
    ax_vv.grid(True, alpha=0.3)
    ax_vv.set_xlim(t.min(), t.max())
    kp_lo = min(kp_x.min(), kp_z.min()) - 0.1
    kp_hi = max(kp_x.max(), kp_z.max()) + 0.1
    ax_vv.set_ylim(kp_lo, kp_hi)
    line_kpx, = ax_vv.plot([], [], lw=0.8, color="steelblue", label="kp_x")
    line_kpz, = ax_vv.plot([], [], lw=0.8, color="orange", label="kp_z")
    ax_kd_a = ax_vv.twinx()
    ax_kd_a.set_ylabel("kd", color="gray")
    ax_kd_a.set_ylim(-0.1, max(kd_x.max(), kd_z.max()) * 1.2)
    line_kdx, = ax_kd_a.plot([], [], lw=0.6, ls="--", color="lightsteelblue", label="kd_x")
    line_kdz, = ax_kd_a.plot([], [], lw=0.6, ls="--", color="navajowhite", label="kd_z")
    ax_kd_a.tick_params(axis="y", colors="gray")
    h1, l1 = ax_vv.get_legend_handles_labels()
    h2, l2 = ax_kd_a.get_legend_handles_labels()
    ax_vv.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=7)

    def init_anim():
        trail_line.set_data([], [])
        cur_phase_line.set_data([], [])
        dot.set_data([], [])
        line_tau_x.set_data([], [])
        line_tau_z.set_data([], [])
        cursor_tau.set_xdata([t[0], t[0]])
        line_px.set_data([], [])
        line_pz.set_data([], [])
        cursor_xz.set_xdata([t[0], t[0]])
        line_kpx.set_data([], [])
        line_kpz.set_data([], [])
        line_kdx.set_data([], [])
        line_kdz.set_data([], [])
        return (trail_line, cur_phase_line, dot,
                line_tau_x, line_tau_z, cursor_tau,
                line_px, line_pz, cursor_xz,
                line_kpx, line_kpz, line_kdx, line_kdz)

    def update_anim(idx):
        i = save_indices[idx]
        now_id = phase_id[i]
        now_name = JUMP_PHASE_NAMES[now_id] if now_id >= 0 else "?"

        # X-Z trail
        trail_line.set_data(x[:i + 1], z[:i + 1])
        # 当前 phase 段
        mask = phase_id[:i + 1] == now_id
        cur_phase_line.set_data(x[:i + 1][mask], z[:i + 1][mask])
        cur_phase_line.set_color(JUMP_PHASE_COLORS.get(now_name, "gray"))
        dot.set_data([x[i]], [z[i]])
        ax_p.set_title(f"X-Z Plane (t={t[i]:.3f}s, {now_name})")

        # Torque
        line_tau_x.set_data(t[:i + 1], tau_x[:i + 1])
        line_tau_z.set_data(t[:i + 1], tau_z[:i + 1])
        cursor_tau.set_xdata([t[i], t[i]])

        # Position vs time
        line_px.set_data(t[:i + 1], x[:i + 1])
        line_pz.set_data(t[:i + 1], z[:i + 1])
        cursor_xz.set_xdata([t[i], t[i]])

        # PD gains
        line_kpx.set_data(t[:i + 1], kp_x[:i + 1])
        line_kpz.set_data(t[:i + 1], kp_z[:i + 1])
        line_kdx.set_data(t[:i + 1], kd_x[:i + 1])
        line_kdz.set_data(t[:i + 1], kd_z[:i + 1])

        if idx % 60 == 0:
            print(f"  Frame {idx + 1}/{save_n}...")
        return (trail_line, cur_phase_line, dot,
                line_tau_x, line_tau_z, cursor_tau,
                line_px, line_pz, cursor_xz,
                line_kpx, line_kpz, line_kdx, line_kdz)

    ani = FuncAnimation(
        fig_anim, update_anim, frames=save_n, init_func=init_anim,
        interval=TARGET_INTERVAL_MS, blit=False,
    )
    out_webp = os.path.join(out_dir, f"{stem}.webp")
    ani.save(out_webp, writer=PillowWriter(fps=TARGET_FPS))
    print(f"Animation saved: {out_webp}")
    plt.close(fig_anim)
    print("Done.")


def _draw_phase_regions(ax, t, phase_id, phase_times, phase_labels):
    """在 time-series 子图上绘制 phase 背景色带和标签。"""
    for i in range(len(phase_times) - 1):
        t0, t1 = phase_times[i], phase_times[i + 1]
        name = phase_labels[i]
        color = JUMP_PHASE_COLORS.get(name, "#cccccc")
        ax.axvspan(t0, t1, facecolor=color, alpha=0.12, edgecolor="none")
        ax.text((t0 + t1) / 2, ax.get_ylim()[1] * 0.97, name,
                ha="center", va="top", fontsize=7,
                color=color, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.15", facecolor="white", alpha=0.7))


# ═══════════════════════════════════════════════════════════════
#  walkTest / standTest 可视化 (旧格式，保留兼容)
# ═══════════════════════════════════════════════════════════════
JUMP_PHASE_NAMES_LEGACY = {0: "PUSH_OFF", 1: "FLIGHT", 2: "ABSORPTION", 3: "FINISHED"}

def run_legacy(df, t, x, z, vx, vz, stem, out_dir):
    n = len(df)

    cycle = df["cycle"].to_numpy().astype(int)
    step_len = df["stepLength_mm"].to_numpy()
    total_t = df["totalTime_s"].to_numpy()

    change_mask = np.diff(step_len, prepend=step_len[0]) != 0
    change_times = np.unique(t[change_mask])
    unique_cycles = np.unique(cycle)
    duration = t[-1] - t[0]

    # 检测: jump legacy / stand / walk
    if cycle.max() <= 3 and (cycle.max() == 3 and np.all(step_len == step_len[0])):
        mode = "jump_legacy"
        data_label = "Jump (legacy)"
        hint_label = "jumpLength (mm)"
        cycle_label = "phase"
    elif total_t[-1] == 0.0 and step_len[-1] > 100:
        mode = "stand"
        data_label = "Stand"
        hint_label = "targetZ (mm)"
        cycle_label = "segment"
    else:
        mode = "walk"
        data_label = "Walk"
        hint_label = "stepLength (mm)"
        cycle_label = "cycle"

    print(f"[{data_label}] Data: {n} pts, duration={duration:.3f}s, "
          f"segments={unique_cycles.max() + 1}, param changes={len(change_times)}")

    # ========== 静态对比图 ==========
    fig_static, axes_static = plt.subplots(2, 2, figsize=(14, 8))
    fig_static.suptitle(f"{stem} — {data_label} Online Replanning ("
                        f"{unique_cycles.max() + 1} segments)")

    ax_xz = axes_static[0, 0]
    ax_xz.set_title(f"Foot Trajectory (X-Z Plane, colored by {cycle_label})")
    ax_xz.set_xlabel("pos_x (mm)")
    ax_xz.set_ylabel("pos_z (mm)")
    ax_xz.grid(True, alpha=0.3)
    cmap = plt.cm.viridis # type: ignore
    for c in unique_cycles:
        mask = cycle == c
        if mode == "jump_legacy":
            lbl = JUMP_PHASE_NAMES_LEGACY.get(c, f"{cycle_label} {c}")
        else:
            lbl = f"{cycle_label} {c}"
        ax_xz.plot(x[mask], z[mask], lw=1.2, alpha=0.8,
                   color=cmap(c / max(1, unique_cycles.max())), label=lbl)
    ax_xz.scatter(x[0], z[0], c="green", s=60, zorder=5, label="start")
    ax_xz.scatter(x[-1], z[-1], c="red", s=60, zorder=5, label="end")
    ax_xz.invert_yaxis()
    ax_xz.legend(fontsize=7, ncol=3, loc="lower left")

    ax_xt = axes_static[0, 1]
    ax_xt.set_title("pos_x vs Time")
    ax_xt.plot(t, x, lw=0.8, color="steelblue")
    ax_xt.set_xlabel("time (s)")
    ax_xt.set_ylabel("pos_x (mm)")
    ax_xt.grid(True, alpha=0.3)
    for ct in change_times:
        ax_xt.axvline(x=ct, color="red", linestyle="--", alpha=0.5, lw=1)
    if mode == "jump_legacy":
        for pi in np.where(np.diff(cycle) != 0)[0]:
            ax_xt.axvline(x=t[pi], color="orange", linestyle=":", alpha=0.7, lw=1.5)

    ax_zt = axes_static[1, 0]
    ax_zt.set_title("pos_z vs Time")
    ax_zt.plot(t, z, lw=0.8, color="forestgreen")
    ax_zt.set_xlabel("time (s)")
    ax_zt.set_ylabel("pos_z (mm)")
    ax_zt.grid(True, alpha=0.3)
    for ct in change_times:
        ax_zt.axvline(x=ct, color="red", linestyle="--", alpha=0.5, lw=1)
    if mode == "jump_legacy":
        for pi in np.where(np.diff(cycle) != 0)[0]:
            ax_zt.axvline(x=t[pi], color="orange", linestyle=":", alpha=0.7, lw=1.5)

    ax_v = axes_static[1, 1]
    ax_v.set_title(f"Velocity & {hint_label} vs Time")
    ax_v.plot(t, vx, lw=0.6, alpha=0.7, color="steelblue", label="vel_x")
    ax_v.plot(t, vz, lw=0.6, alpha=0.7, color="forestgreen", label="vel_z")
    ax_v.set_xlabel("time (s)")
    ax_v.set_ylabel("velocity (mm/s)")
    ax_v.grid(True, alpha=0.3)
    ax_sl = ax_v.twinx()
    ax_sl.plot(t, step_len, lw=0.5, alpha=0.5, color="gray", linestyle="dotted",
               label=hint_label)
    ax_sl.set_ylabel(hint_label, color="gray")
    ax_sl.tick_params(axis="y", colors="gray")
    for ct in change_times:
        ax_v.axvline(x=ct, color="red", linestyle="--", alpha=0.5, lw=1)
    lines_v, labels_v = ax_v.get_legend_handles_labels()
    lines_sl, labels_sl = ax_sl.get_legend_handles_labels()
    ax_v.legend(lines_v + lines_sl, labels_v + labels_sl, fontsize=7)

    plt.tight_layout()
    static_png = os.path.join(out_dir, f"{stem}_static.png")
    fig_static.savefig(static_png, dpi=150)
    print(f"Static overview saved to: {static_png}")
    plt.close(fig_static)

    # ========== 动画 ==========
    TARGET_FPS = 100
    TARGET_INTERVAL_MS = 1000.0 / TARGET_FPS
    target_frames = max(2, int(round(duration * TARGET_FPS)))
    save_stride = max(1, n // target_frames)
    save_indices = list(range(0, n, save_stride))
    if save_indices[-1] != n - 1:
        save_indices.append(n - 1)
    save_n = len(save_indices)
    print(f"Saving animation: stride={save_stride}, {save_n} frames @{TARGET_FPS}fps")

    fig_anim, (ax_p, ax_tv) = plt.subplots(1, 2, figsize=(12, 5))
    fig_anim.suptitle(f"{stem} — {data_label} ({unique_cycles.max() + 1} segments)")

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
    cur_cycle_line, = ax_p.plot([], [], lw=2, color="orange",
                                label=f"cur {cycle_label}")
    dot, = ax_p.plot([], [], "ro", ms=8, zorder=5, label="now")
    ax_p.legend(loc="lower left", fontsize=7)

    ax_tv.set_title(f"Velocity & {hint_label}")
    ax_tv.set_xlabel("time (s)")
    ax_tv.set_ylabel("velocity (mm/s)")
    ax_tv.grid(True, alpha=0.3)
    ax_tv.set_xlim(t.min(), t.max())
    ylo, yhi = min(vx.min(), vz.min()), max(vx.max(), vz.max())
    mv = (yhi - ylo) * 0.1 + 1e-6
    ax_tv.set_ylim(ylo - mv, yhi + mv)
    line_vx, = ax_tv.plot([], [], lw=0.8, color="steelblue", label="vel_x")
    line_vz, = ax_tv.plot([], [], lw=0.8, color="forestgreen", label="vel_z")
    cursor_v = ax_tv.axvline(x=0, color="black", ls="--", alpha=0.6)
    ax_tv.legend(loc="upper left", fontsize=7)
    ax_sl_anim = ax_tv.twinx()
    ax_sl_anim.set_ylabel(hint_label, color="gray")
    ax_sl_anim.tick_params(axis="y", colors="gray")
    sl_lo, sl_hi = step_len.min() - 5, step_len.max() + 5
    ax_sl_anim.set_ylim(sl_lo, sl_hi)
    line_sl, = ax_sl_anim.plot([], [], lw=0.5, color="gray", linestyle="dotted",
                                alpha=0.7, label=hint_label)
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
        now_cycle = cycle[i]
        trail_line.set_data(x[:i + 1], z[:i + 1])
        mask_cur = cycle[:i + 1] == now_cycle
        cx, cz = x[:i + 1][mask_cur], z[:i + 1][mask_cur]
        cur_cycle_line.set_data(cx, cz)
        dot.set_data([x[i]], [z[i]])
        line_vx.set_data(t[:i + 1], vx[:i + 1])
        line_vz.set_data(t[:i + 1], vz[:i + 1])
        cursor_v.set_xdata([t[i], t[i]])
        line_sl.set_data(t[:i + 1], step_len[:i + 1])

        if mode == "jump_legacy":
            phase_name = JUMP_PHASE_NAMES_LEGACY.get(now_cycle, str(now_cycle))
            ax_p.set_title(f"Path in X-Z Plane (t={t[i]:.3f}s, {phase_name})")
        else:
            ax_p.set_title(f"Path in X-Z Plane (t={t[i]:.3f}s, {cycle_label}={now_cycle})")

        if idx % 60 == 0:
            print(f"  Frame {idx + 1}/{save_n}...")
        return trail_line, cur_cycle_line, dot, line_vx, line_vz, cursor_v, line_sl

    ani = FuncAnimation(
        fig_anim, update_anim, frames=save_n, init_func=init_anim,
        interval=TARGET_INTERVAL_MS, blit=False,
    )
    out_webp = os.path.join(out_dir, f"{stem}.webp")
    ani.save(out_webp, writer=PillowWriter(fps=TARGET_FPS))
    print(f"Animation saved: {out_webp}")
    plt.close(fig_anim)
    print("Done.")


if __name__ == "__main__":
    main()