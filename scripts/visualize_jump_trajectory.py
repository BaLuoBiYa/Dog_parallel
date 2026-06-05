#!/usr/bin/env python3
"""
跳跃轨迹可视化工具
------------------
模拟 JumpTrajectory 的全部阶段（CROUCH → PUSH → FLIGHT → ABSORPTION → RECOVER → FINISHED）
并绘制足端位置、速度、加速度/力随时间的变化曲线。

用法:
    python3 scripts/visualize_jump_trajectory.py [--config PATH] [--par-id ID]

    --config    YAML 配置文件路径（默认: src/dog_bringup/parameters/jump.yaml）
    --par-id    参数 ID（默认: 0）
"""

import argparse
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import yaml

# =============================================================================
# 五次多项式核心计算（与 C++ QuinticPolynomial 一致）
# =============================================================================

def solve_quintic_coeff(p0, v0, a0, pT, vT, aT, T):
    """解析求解五次多项式系数 p(t) = c0 + c1*t + c2*t² + c3*t³ + c4*t⁴ + c5*t⁵"""
    T2 = T * T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T
    T6 = T3 * T3
    T9 = T6 * T3

    c0 = p0
    c1 = v0
    c2 = 0.5 * a0

    # 右端向量 b = [Δp, Δv, Δa]
    b0 = pT - (c0 + c1 * T + c2 * T2)
    b1 = vT - (c1 + 2.0 * c2 * T)
    b2 = aT - (2.0 * c2)

    invDet = 0.5 / T9  # 1/(2T⁹)

    # Cramer 法则
    num3 = 20.0 * b0 * T6 - 8.0 * b1 * T6 * T + b2 * T6 * T2
    num4 = -30.0 * b0 * T5 + 14.0 * b1 * T6 - 2.0 * b2 * T6 * T
    num5 = 12.0 * b0 * T4 - 6.0 * b1 * T5 + b2 * T6

    c3 = num3 * invDet
    c4 = num4 * invDet
    c5 = num5 * invDet

    return np.array([c0, c1, c2, c3, c4, c5])


def eval_pos(coeff, t):
    t2, t3, t4, t5 = t*t, t*t*t, t**4, t**5
    return coeff[0] + coeff[1]*t + coeff[2]*t2 + coeff[3]*t3 + coeff[4]*t4 + coeff[5]*t5


def eval_vel(coeff, t):
    t2, t3, t4 = t*t, t*t*t, t**4
    return coeff[1] + 2*coeff[2]*t + 3*coeff[3]*t2 + 4*coeff[4]*t3 + 5*coeff[5]*t4


def eval_acc(coeff, t):
    t2, t3 = t*t, t*t*t
    return 2*coeff[2] + 6*coeff[3]*t + 12*coeff[4]*t2 + 20*coeff[5]*t3


# =============================================================================
# 跳跃轨迹模拟
# =============================================================================

def simulate_jump(params, dt=0.001):
    """
    模拟完整跳跃轨迹，返回各阶段时间序列数据。

    参数:
        params: dict, YAML 中的参数
        dt:     采样间隔 (s)

    返回:
        dict: {
            't':        时间数组,
            'pos_x':    X 位置 (mm),
            'pos_z':    Z 位置 (mm, 正=向下/远离髋部),
            'vel_x':    X 速度 (mm/s),
            'vel_z':    Z 速度 (mm/s),
            'acc_z':    Z 加速度 (mm/s²),
            'force_z':  单腿 Z 方向力 (N),
            'phase':    阶段标记 (0=CROUCH, 1=PUSH, 2=FLIGHT, 3=ABSORPTION, 4=RECOVER, 5=FINISHED),
            'phase_names': 阶段名称列表,
            'phase_boundaries': 阶段边界时间
        }
    """
    BODY_MASS = 12.0  # kg, 机体总质量
    MASS_PER_LEG = BODY_MASS / 4.0

    # 提取参数
    stand_z = params.get('legLengthStand', 270.0)       # 正常站立腿长 (mm)
    start_z = params.get('startZ', 80.0)                 # 下蹲目标高度 (mm)
    finish_z = params.get('finishZ', 270.0)              # 起跳伸展高度 (mm)
    tuck_min_z = params.get('tuckMinZ', 230.0)

    t_crouch = params.get('crouchDuration', 0.20)        # 下蹲时长
    t_push = params.get('pushDuration', 0.15)            # 蹬地时长
    t_flight = params.get('flightDuration', 0.30)        # 空中时长
    t_absorb = params.get('absorbDuration', 0.10)        # 缓冲时长
    t_recover = params.get('recoverDuration', 0.25)      # 恢复时长

    launch_speed = params.get('launchSpeed', 0.0)         # 起跳速度 (m/s → 转为 mm/s)
    launch_angle_deg = params.get('launchAngle', 0.0)     # 起跳角度 (度)
    launch_angle = np.radians(launch_angle_deg)

    air_tuck_factor = params.get('airTipLevelFactor', 0.1)

    # 单位转换: 速度从 m/s 转为 mm/s
    launch_speed_mm = launch_speed * 1000.0

    cos_a = np.cos(launch_angle)
    sin_a = np.sin(launch_angle)

    vx_launch = launch_speed_mm * cos_a
    vz_launch = launch_speed_mm * sin_a

    # ---- 阶段 0: CROUCH (下蹲) ----
    # 从站立 Z 高度平滑过渡到 startZ，五次多项式，a(0)=a(T)=0
    n_crouch = max(1, int(t_crouch / dt))
    t_crouch_arr = np.linspace(0, t_crouch, n_crouch)

    coeff_crouch = solve_quintic_coeff(
        p0=stand_z, v0=0.0, a0=0.0,
        pT=start_z, vT=0.0, aT=0.0,
        T=t_crouch
    )
    crouch_z = np.array([eval_pos(coeff_crouch, t) for t in t_crouch_arr])
    crouch_vz = np.array([eval_vel(coeff_crouch, t) for t in t_crouch_arr])
    crouch_az = np.array([eval_acc(coeff_crouch, t) for t in t_crouch_arr])
    crouch_x = np.zeros(n_crouch)
    crouch_vx = np.zeros(n_crouch)

    # ---- 阶段 1: PUSH (蹬地) ----
    # 使用 PushPlanner: 五次多项式，从当前状态到起跳目标
    n_push = max(1, int(t_push / dt))
    t_push_arr = np.linspace(0, t_push, n_push)

    push_start_x = 0.0
    push_start_z = start_z
    push_start_vx = vx_launch * 0.0  # 蹬地开始时 X 速度从 0 开始
    push_start_vz = 0.0

    push_target_x = push_start_x + 0.5 * vx_launch * t_push  # 近似 X 位移
    push_target_z = finish_z
    push_target_vx = vx_launch
    push_target_vz = vz_launch

    coeff_push_x = solve_quintic_coeff(
        p0=push_start_x, v0=push_start_vx, a0=0.0,
        pT=push_target_x, vT=push_target_vx, aT=0.0,
        T=t_push
    )
    coeff_push_z = solve_quintic_coeff(
        p0=push_start_z, v0=push_start_vz, a0=0.0,
        pT=push_target_z, vT=push_target_vz, aT=0.0,
        T=t_push
    )

    push_x = np.array([eval_pos(coeff_push_x, t) for t in t_push_arr])
    push_z = np.array([eval_pos(coeff_push_z, t) for t in t_push_arr])
    push_vx = np.array([eval_vel(coeff_push_x, t) for t in t_push_arr])
    push_vz = np.array([eval_vel(coeff_push_z, t) for t in t_push_arr])
    push_az = np.array([eval_acc(coeff_push_z, t) for t in t_push_arr])

    # ---- 阶段 2: FLIGHT (空中收腿) ----
    # 五次多项式收腿: 从当前 Z 到 tuck 位置
    n_flight = max(1, int(t_flight / dt))
    t_flight_arr = np.linspace(0, t_flight, n_flight)

    flight_start_x = push_x[-1]
    flight_start_z = push_z[-1]
    flight_start_vx = push_vx[-1]
    flight_start_vz = push_vz[-1]

    # 计算收腿目标 Z
    tuck_delta = max(0.0, flight_start_z - start_z) * air_tuck_factor
    flight_target_z = max(tuck_min_z, flight_start_z - tuck_delta)
    # X 保持当前速度滑行
    flight_target_x = flight_start_x + flight_start_vx * t_flight

    coeff_flight_x = solve_quintic_coeff(
        p0=flight_start_x, v0=flight_start_vx, a0=0.0,
        pT=flight_target_x, vT=flight_start_vx, aT=0.0,
        T=t_flight
    )
    coeff_flight_z = solve_quintic_coeff(
        p0=flight_start_z, v0=flight_start_vz, a0=0.0,
        pT=flight_target_z, vT=0.0, aT=0.0,
        T=t_flight
    )

    flight_x = np.array([eval_pos(coeff_flight_x, t) for t in t_flight_arr])
    flight_z = np.array([eval_pos(coeff_flight_z, t) for t in t_flight_arr])
    flight_vx = np.array([eval_vel(coeff_flight_x, t) for t in t_flight_arr])
    flight_vz = np.array([eval_vel(coeff_flight_z, t) for t in t_flight_arr])
    flight_az = np.array([eval_acc(coeff_flight_z, t) for t in t_flight_arr])

    # ---- 阶段 3: ABSORPTION (缓冲) ----
    # 被动阻尼阶段，位置保持不动，速度为零
    n_absorb = max(1, int(t_absorb / dt))
    t_absorb_arr = np.linspace(0, t_absorb, n_absorb)

    absorb_x = np.full(n_absorb, flight_x[-1])
    absorb_z = np.full(n_absorb, flight_z[-1])
    absorb_vx = np.zeros(n_absorb)
    absorb_vz = np.zeros(n_absorb)
    absorb_az = np.zeros(n_absorb)

    # ---- 阶段 4: RECOVER (恢复站立) ----
    # 从当前 Z 平滑过渡回站立高度
    n_recover = max(1, int(t_recover / dt))
    t_recover_arr = np.linspace(0, t_recover, n_recover)

    recover_start_z = absorb_z[-1]
    recover_start_x = absorb_x[-1]

    coeff_recover_x = solve_quintic_coeff(
        p0=recover_start_x, v0=0.0, a0=0.0,
        pT=0.0, vT=0.0, aT=0.0,  # X 回到 0
        T=t_recover
    )
    coeff_recover_z = solve_quintic_coeff(
        p0=recover_start_z, v0=0.0, a0=0.0,
        pT=stand_z, vT=0.0, aT=0.0,
        T=t_recover
    )

    recover_x = np.array([eval_pos(coeff_recover_x, t) for t in t_recover_arr])
    recover_z = np.array([eval_pos(coeff_recover_z, t) for t in t_recover_arr])
    recover_vx = np.array([eval_vel(coeff_recover_x, t) for t in t_recover_arr])
    recover_vz = np.array([eval_vel(coeff_recover_z, t) for t in t_recover_arr])
    recover_az = np.array([eval_acc(coeff_recover_z, t) for t in t_recover_arr])

    # ---- 阶段 5: FINISHED (完成) ----
    # 稳态保持
    t_hold = 0.2
    n_finished = max(1, int(t_hold / dt))
    t_finished_arr = np.linspace(0, t_hold, n_finished)

    finished_x = np.zeros(n_finished)
    finished_z = np.full(n_finished, stand_z)
    finished_vx = np.zeros(n_finished)
    finished_vz = np.zeros(n_finished)
    finished_az = np.zeros(n_finished)

    # ---- 拼接所有阶段 ----
    phases = [
        ('CROUCH',     t_crouch_arr,   crouch_x,   crouch_z,   crouch_vx,   crouch_vz,   crouch_az),
        ('PUSH',       t_push_arr,     push_x,     push_z,     push_vx,     push_vz,     push_az),
        ('FLIGHT',     t_flight_arr,   flight_x,   flight_z,   flight_vx,   flight_vz,   flight_az),
        ('ABSORPTION', t_absorb_arr,   absorb_x,   absorb_z,   absorb_vx,   absorb_vz,   absorb_az),
        ('RECOVER',    t_recover_arr,  recover_x,  recover_z,  recover_vx,  recover_vz,  recover_az),
        ('FINISHED',   t_finished_arr, finished_x, finished_z, finished_vx, finished_vz, finished_az),
    ]

    t_offset = 0.0
    all_t = []
    all_x, all_z = [], []
    all_vx, all_vz = [], []
    all_az = []
    all_phase = []
    phase_boundaries = [0.0]

    for i, (name, t_arr, x, z, vx, vz, az) in enumerate(phases):
        all_t.extend(t_arr + t_offset)
        all_x.extend(x)
        all_z.extend(z)
        all_vx.extend(vx)
        all_vz.extend(vz)
        all_az.extend(az)
        all_phase.extend([i] * len(t_arr))
        t_offset += t_arr[-1] + dt if len(t_arr) > 0 else 0
        phase_boundaries.append(t_offset)

    all_t = np.array(all_t)
    all_x = np.array(all_x)
    all_z = np.array(all_z)
    all_vx = np.array(all_vx)
    all_vz = np.array(all_vz)
    all_az = np.array(all_az)
    all_phase = np.array(all_phase)

    # 计算单腿力 F_per_leg(t) = (M/4) * a_z(t)
    all_force_z = MASS_PER_LEG * all_az / 1000.0  # mm/s² → m/s², 再乘质量

    return {
        't': all_t,
        'pos_x': all_x,
        'pos_z': all_z,
        'vel_x': all_vx,
        'vel_z': all_vz,
        'acc_z': all_az,
        'force_z': all_force_z,
        'phase': all_phase,
        'phase_names': [p[0] for p in phases],
        'phase_boundaries': phase_boundaries,
    }


# =============================================================================
# 可视化
# =============================================================================

PHASE_COLORS = {
    0: '#4C72B0',  # CROUCH - blue
    1: '#DD8452',  # PUSH - orange
    2: '#55A868',  # FLIGHT - green
    3: '#C44E52',  # ABSORPTION - red
    4: '#937860',  # RECOVER - brown
    5: '#8C8C8C',  # FINISHED - gray
}


def plot_trajectory(data, params, save_path=None):
    """绘制完整的跳跃轨迹分析图"""
    t = data['t']
    phase = data['phase']
    phase_names = data['phase_names']
    boundaries = data['phase_boundaries']

    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    fig.suptitle('Jump Trajectory Visualization', fontsize=16, fontweight='bold')

    # ---- 子图 1: Z 位置 (足端高度) ----
    ax = axes[0, 0]
    for i, name in enumerate(phase_names):
        mask = phase == i
        if mask.any():
            ax.plot(t[mask], data['pos_z'][mask], color=PHASE_COLORS[i], lw=1.5, label=name)
    # 标注关键高度
    stand_z = params.get('legLengthStand', 270)
    start_z = params.get('startZ', 80)
    finish_z = params.get('finishZ', 270)
    ax.axhline(y=stand_z, color='gray', ls='--', lw=1, alpha=0.6, label=f'Stand Z={stand_z}mm')
    ax.axhline(y=start_z, color='blue', ls=':', lw=1, alpha=0.5, label=f'Crouch Z={start_z}mm')
    ax.axhline(y=finish_z, color='orange', ls=':', lw=1, alpha=0.5, label=f'Extend Z={finish_z}mm')
    ax.set_ylabel('Z Position (mm)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Foot Tip Z Position (positive = downward from hip)')
    ax.legend(fontsize=7, loc='best')
    ax.grid(True, alpha=0.3)

    # 添加阶段背景色
    for i in range(len(boundaries) - 1):
        ax.axvspan(boundaries[i], boundaries[i+1], alpha=0.06, color=PHASE_COLORS.get(i, '#ccc'))

    # ---- 子图 2: X 位置 (前进方向) ----
    ax = axes[0, 1]
    for i, name in enumerate(phase_names):
        mask = phase == i
        if mask.any():
            ax.plot(t[mask], data['pos_x'][mask], color=PHASE_COLORS[i], lw=1.5, label=name)
    ax.set_ylabel('X Position (mm)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Foot Tip X Position (forward direction)')
    ax.legend(fontsize=7, loc='best')
    ax.grid(True, alpha=0.3)
    for i in range(len(boundaries) - 1):
        ax.axvspan(boundaries[i], boundaries[i+1], alpha=0.06, color=PHASE_COLORS.get(i, '#ccc'))

    # ---- 子图 3: Z 速度 ----
    ax = axes[1, 0]
    for i, name in enumerate(phase_names):
        mask = phase == i
        if mask.any():
            ax.plot(t[mask], data['vel_z'][mask], color=PHASE_COLORS[i], lw=1.5, label=name)
    ax.axhline(y=0, color='black', ls='-', lw=0.5)
    ax.set_ylabel('Z Velocity (mm/s)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Foot Tip Z Velocity')
    ax.legend(fontsize=7, loc='best')
    ax.grid(True, alpha=0.3)
    for i in range(len(boundaries) - 1):
        ax.axvspan(boundaries[i], boundaries[i+1], alpha=0.06, color=PHASE_COLORS.get(i, '#ccc'))

    # ---- 子图 4: X 速度 ----
    ax = axes[1, 1]
    for i, name in enumerate(phase_names):
        mask = phase == i
        if mask.any():
            ax.plot(t[mask], data['vel_x'][mask], color=PHASE_COLORS[i], lw=1.5, label=name)
    ax.axhline(y=0, color='black', ls='-', lw=0.5)
    ax.set_ylabel('X Velocity (mm/s)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Foot Tip X Velocity')
    ax.legend(fontsize=7, loc='best')
    ax.grid(True, alpha=0.3)
    for i in range(len(boundaries) - 1):
        ax.axvspan(boundaries[i], boundaries[i+1], alpha=0.06, color=PHASE_COLORS.get(i, '#ccc'))

    # ---- 子图 5: 单腿足端力 ----
    ax = axes[2, 0]
    for i, name in enumerate(phase_names):
        mask = phase == i
        if mask.any():
            ax.plot(t[mask], data['force_z'][mask], color=PHASE_COLORS[i], lw=1.5, label=name)
    ax.axhline(y=0, color='black', ls='-', lw=0.5)
    # 自重参考线
    body_weight_per_leg = 12.0 * 9.81 / 4.0
    ax.axhline(y=-body_weight_per_leg, color='red', ls='--', lw=1, alpha=0.5,
               label=f'Body weight/leg = {body_weight_per_leg:.1f}N')
    ax.set_ylabel('Force per leg (N)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Foot Tip Z Force (single leg, F = M/4 · a)')
    ax.legend(fontsize=7, loc='best')
    ax.grid(True, alpha=0.3)
    for i in range(len(boundaries) - 1):
        ax.axvspan(boundaries[i], boundaries[i+1], alpha=0.06, color=PHASE_COLORS.get(i, '#ccc'))

    # ---- 子图 6: 2D 轨迹 (X vs Z) ----
    ax = axes[2, 1]
    for i, name in enumerate(phase_names):
        mask = phase == i
        if mask.any():
            ax.plot(data['pos_x'][mask], data['pos_z'][mask], color=PHASE_COLORS[i], lw=1.5, label=name)
    # 标注起点和终点
    ax.scatter([data['pos_x'][0]], [data['pos_z'][0]], c='green', s=80, zorder=5, marker='o', label='Start')
    ax.scatter([data['pos_x'][-1]], [data['pos_z'][-1]], c='red', s=80, zorder=5, marker='s', label='End')
    # 添加箭头表示运动方向
    step = max(1, len(data['pos_x']) // 20)
    ax.quiver(data['pos_x'][::step], data['pos_z'][::step],
              np.diff(data['pos_x'][::step], append=data['pos_x'][-1]),
              np.diff(data['pos_z'][::step], append=data['pos_z'][-1]),
              angles='xy', scale_units='xy', scale=1, alpha=0.3, width=0.003)
    ax.set_xlabel('X Position (mm)')
    ax.set_ylabel('Z Position (mm)')
    ax.set_title('2D Foot Trajectory (X-Z plane)')
    ax.legend(fontsize=7, loc='best')
    ax.grid(True, alpha=0.3)
    ax.invert_yaxis()  # Z+ is downward, invert so 'up' is visually up
    ax.set_aspect('equal')

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Trajectory plot saved: {save_path}")
    else:
        plt.show()
    plt.close(fig)


def plot_phase_summary(data, params, save_path=None):
    """绘制紧凑的阶段摘要图"""
    t = data['t']
    phase = data['phase']
    phase_names = data['phase_names']
    boundaries = data['phase_boundaries']

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle('Jump Trajectory — Phase Summary', fontsize=14, fontweight='bold')

    titles = ['Z Position (mm)', 'X Position (mm)', 'Z Velocity (mm/s)', 'Force per Leg (N)']
    keys = ['pos_z', 'pos_x', 'vel_z', 'force_z']

    for ax, title, key in zip(axes, titles, keys):
        for i, name in enumerate(phase_names):
            mask = phase == i
            if mask.any():
                ax.plot(t[mask], data[key][mask], color=PHASE_COLORS[i], lw=1.2, label=name)
        ax.set_ylabel(title)
        ax.legend(fontsize=7, loc='upper right', ncol=6)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='black', ls='-', lw=0.5, alpha=0.3)
        for i in range(len(boundaries) - 1):
            ax.axvspan(boundaries[i], boundaries[i+1], alpha=0.06, color=PHASE_COLORS.get(i, '#ccc'))

    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Summary saved: {save_path}")
    else:
        plt.show()
    plt.close(fig)


def print_stats(data, params):
    """Print key statistics"""
    print("\n" + "=" * 60)
    print("  Jump Trajectory Key Statistics")
    print("=" * 60)

    phase_names = data['phase_names']
    phase = data['phase']
    t = data['t']

    for i, name in enumerate(phase_names):
        mask = phase == i
        if mask.any():
            t_phase = t[mask]
            z_phase = data['pos_z'][mask]
            v_phase = data['vel_z'][mask]
            f_phase = data['force_z'][mask]

            duration = t_phase[-1] - t_phase[0]
            z_min, z_max = z_phase.min(), z_phase.max()
            z_delta = z_phase[-1] - z_phase[0]
            v_max = np.abs(v_phase).max()
            f_max = np.abs(f_phase).max()

            print(f"\n  [{name}]")
            print(f"    Duration:       {duration:.3f} s")
            print(f"    Z range:        {z_min:.1f} ~ {z_max:.1f} mm  (delta={z_delta:.1f})")
            print(f"    Peak |Vz|:      {v_max:.1f} mm/s")
            if f_max > 1e-3:
                print(f"    Peak |Fz|:      {f_max:.2f} N")

    total_duration = t[-1] - t[0]
    print(f"\n  Total duration: {total_duration:.3f} s")
    print(f"  Total samples:  {len(t)}")
    print("=" * 60)


# =============================================================================
# 主入口
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Jump trajectory visualization - simulate and plot all JumpTrajectory phases',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s
  %(prog)s --config src/dog_bringup/parameters/jump.yaml --par-id 0
  %(prog)s --par-id 1 --save jump_id1.png
        """
    )
    parser.add_argument('--config', type=str,
                        default='src/dog_bringup/parameters/jump.yaml',
                        help='YAML 配置文件路径')
    parser.add_argument('--par-id', type=int, default=0,
                        help='参数 ID (默认: 0)')
    parser.add_argument('--save', type=str, default=None,
                        help='保存图片到指定路径（不显示窗口）')
    parser.add_argument('--no-show', action='store_true',
                        help='不显示交互窗口')
    args = parser.parse_args()

    # 查找配置文件
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)  # scripts/ 的上一级

    config_path = args.config
    if not os.path.isabs(config_path):
        config_path = os.path.join(project_root, config_path)

    if not os.path.exists(config_path):
        print(f"Error: Config file not found: '{config_path}'")
        sys.exit(1)

    print(f"Reading config: {config_path}")
    print(f"Parameter ID:  {args.par_id}")

    with open(config_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    if args.par_id not in yaml_data:
        print(f"Error: Parameter ID {args.par_id} not found (available: {list(yaml_data.keys())})")
        sys.exit(1)

    params = yaml_data[args.par_id]
    print(f"Parameters:\n{yaml.dump(params, default_flow_style=False)}")

    # Simulate
    print("Simulating jump trajectory...")
    data = simulate_jump(params, dt=0.001)

    # Print stats
    print_stats(data, params)

    # 绘图
    # When --save is given, always generate plots (use non-interactive backend if needed)
    if args.save or not args.no_show:
        if args.no_show or args.save:
            import matplotlib
            matplotlib.use('Agg')  # non-interactive backend for headless/save
        if args.save:
            plot_trajectory(data, params, save_path=args.save)
            # Also save phase summary
            base, ext = os.path.splitext(args.save)
            summary_path = f"{base}_summary{ext}"
            plot_phase_summary(data, params, save_path=summary_path)
        else:
            plot_trajectory(data, params, save_path=None)
            plot_phase_summary(data, params)


if __name__ == '__main__':
    main()
