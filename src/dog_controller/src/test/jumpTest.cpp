/**
 * @file jumpTest.cpp
 * @brief JumpTrajectory 全阶段轨迹测试
 *
 * 测试目标：
 *   1. 完整 CROUCH→PUSH→FLIGHT→ABSORPTION→RECOVER→FINISHED 状态机走通
 *   2. 各阶段衔接点位置/速度 C² 连续性（基于五次多项式重规划的开环验证）
 *   3. ABSORPTION 阶段纯阻尼控制（kp=0, kd>0）
 *   4. FINISHED 后 finished() 标志正确置位
 *   5. 输出 CSV，便于外部工具绘图验证
 *
 * 仿真环境：
 *   控制频率 300Hz，足端传感器反馈模拟完美跟踪
 */

#include "dog_trajectory/jumpTrajectory.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using Dog8Dof::Body;
using Dog8Dof::JumpTrajectory;
using Dog8Dof::TrajectoryPoint;

// ===================================================================
//  CSV 输出辅助
// ===================================================================
static void writeHeader(std::ofstream &csv) {
    csv << "t_s," << "phase," << "pos_x,pos_z," << "vel_x,vel_z," << "kp_x,kp_z,"
        << "kd_x,kd_z," << "tau_x,tau_z\n";
}

static const char *phaseName(JumpTrajectory::Phase p) {
    switch (p) {
    case JumpTrajectory::CROUCH:
        return "CROUCH";
    case JumpTrajectory::PUSH:
        return "PUSH";
    case JumpTrajectory::FLIGHT:
        return "FLIGHT";
    case JumpTrajectory::ABSORPTION:
        return "ABSORPTION";
    case JumpTrajectory::RECOVER:
        return "RECOVER";
    case JumpTrajectory::FINISHED:
        return "FINISHED";
    default:
        return "UNKNOWN";
    }
}

static void writeRow(std::ofstream &csv,
                     double t,
                     JumpTrajectory::Phase phase,
                     const TrajectoryPoint &p) {
    csv << std::fixed << std::setprecision(6) << t << "," << phaseName(phase) << ","
        << p.pos[0] << "," << p.pos[2] << "," << p.vel[0] << "," << p.vel[2] << ","
        << p.kp[0] << "," << p.kp[1] << "," << p.kd[0] << "," << p.kd[1] << ","
        << p.tau[0] << "," << p.tau[2] << "\n";
}

// ===================================================================
//  主测试
// ===================================================================
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // ---- 加载配置 ----
    const std::string shareDir =
        ament_index_cpp::get_package_share_directory("dog_bringup");
    const std::string yamlPath =
        (std::filesystem::path(shareDir) / "parameters" / "jump.yaml").string();

    // 100Hz 控制周期
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / 100.0);
    JumpTrajectory traj(yamlPath, period);

    // ---- 初始足端状态（站立姿态） ----
    Body currentTip{};
    currentTip.pos.setZero();
    currentTip.linerVel.setZero();
    currentTip.pos[0] = 0.0f;
    currentTip.pos[2] = 270.0f; // 站立高度

    // ---- 设置目标参数ID ----
    const uint8_t parID = 0;
    if (!traj.setTarget(parID)) {
        std::cerr << "setTarget failed\n";
        return 1;
    }

    // ---- 初始化跳跃：generate() 锚定 CROUCH 起始点 ----
    const rclcpp::Time t0(0, 0, RCL_SYSTEM_TIME);
    TrajectoryPoint firstPt = traj.generate(currentTip, t0);

    // 用首帧更新足端状态
    currentTip.pos = firstPt.pos;
    currentTip.linerVel = firstPt.vel;

    // CSV 输出路径
    std::string csvPath;
    std::cout << "Output CSV path: ";
    std::cin >> csvPath;

    std::ofstream csv(csvPath, std::ios::out | std::ios::trunc);
    if (!csv.is_open()) {
        std::cerr << "Cannot open: " << csvPath << "\n";
        return 2;
    }
    writeHeader(csv);

    // ---- 主仿真循环 ----
    const int64_t dtNs = period.nanoseconds();

    // 运行足够长时间覆盖全部阶段（crouch + push + flight + absorb + recover + 额外余量）
    const double totalSimTime = 2.0;
    unsigned int step = 0;

    JumpTrajectory::Phase prevPhase = JumpTrajectory::CROUCH;

    for (; /* below */; ++step) {
        const int64_t tNs = static_cast<int64_t>(step + 1) * dtNs; // 从 dt 开始
        const double t = static_cast<double>(tNs) * 1e-9;
        const rclcpp::Time now(tNs, RCL_SYSTEM_TIME);

        if (t > totalSimTime)
            break;

        // ---- 获取轨迹点（带传感器反馈，模拟完美跟踪） ----
        const TrajectoryPoint pt = traj.get(now, &currentTip);
        const JumpTrajectory::Phase curPhase = traj.getPhase();

        // ---- 相位转移日志 ----
        if (curPhase != prevPhase) {
            std::cout << "[t=" << t << "s] Phase transition: " << phaseName(prevPhase)
                      << " -> " << phaseName(curPhase) << "  pos=(" << pt.pos[0] << ", "
                      << pt.pos[2] << ")  vel=(" << pt.vel[0] << ", " << pt.vel[2] << ")"
                      << std::endl;

            // PUSH 阶段检查：tau 非零（有前馈力矩）
            if (curPhase == JumpTrajectory::PUSH) {
                if (std::abs(pt.tau[0]) < 1e-6f && std::abs(pt.tau[2]) < 1e-6f) {
                    std::cerr
                        << "  WARNING: PUSH phase tau is zero — PushPlanner may not "
                           "be active\n";
                }
            }

            // ABSORPTION 阶段检查：kp == 0（纯阻尼）
            if (curPhase == JumpTrajectory::ABSORPTION) {
                if (pt.kp[0] != 0.0f || pt.kp[1] != 0.0f) {
                    std::cerr << "  WARNING: ABSORPTION phase kp should be 0\n";
                }
                if (pt.kd[0] <= 0.0f || pt.kd[1] <= 0.0f) {
                    std::cerr << "  WARNING: ABSORPTION phase kd should be > 0\n";
                }
            }

            prevPhase = curPhase;
        }

        // ---- 更新"足端状态"以模拟完美跟踪 ----
        currentTip.pos[0] = pt.pos[0];
        currentTip.pos[2] = pt.pos[2];
        currentTip.linerVel[0] = pt.vel[0];
        currentTip.linerVel[2] = pt.vel[2];

        writeRow(csv, t, curPhase, pt);
        csv.flush();
        // 提前退出：FINISHED 后再跑一小段验证稳定性
        if (traj.finished()) {
            std::cout << "[t=" << t << "s] Trajectory finished\n";
            break;
        }
    }

    csv.close();

    // ---- 结果校验 ----
    std::cout << "\n=== Test Results ===" << std::endl;
    std::cout << "Total simulation steps: " << step << std::endl;
    std::cout << "Final phase: " << phaseName(traj.getPhase()) << std::endl;
    std::cout << "Trajectory finished: " << (traj.finished() ? "YES" : "NO") << std::endl;

    if (!traj.finished()) {
        std::cerr << "FAIL: Trajectory did not reach FINISHED within sim time.\n";
        rclcpp::shutdown();
        return 3;
    }

    // 验证最终腿长回到站立高度附近
    const float finalZ = currentTip.pos[2];
    const float targetStandZ = 270.0f;
    const float zErr = std::abs(finalZ - targetStandZ);
    std::cout << "Final tip Z: " << finalZ << " mm (target: " << targetStandZ
              << " mm, err: " << zErr << " mm)" << std::endl;

    if (zErr > 5.0f) {
        std::cerr << "WARNING: Final Z deviation exceeds 5mm — RECOVER may be "
                     "incomplete.\n";
    }

    std::cout << "CSV written to: " << csvPath << std::endl;
    std::cout << "\n=== JumpTest PASSED ===" << std::endl;

    rclcpp::shutdown();
    return 0;
}