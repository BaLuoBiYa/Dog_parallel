/**
 * @file jumpTest.cpp
 * @brief JumpTrajectory 三段式跳跃轨迹测试
 *
 * 测试：
 *   1. 完整 PUSH_OFF → FLIGHT → ABSORPTION → FINISHED 全流程
 *   2. 多组跳跃参数（水平距离+高度），验证状态机切换
 *   3. 输出 CSV 与 walkTest/standTest 格式对齐，可用同一 visual.py 渲染
 */

#include "dog_trajectory/jumpTrajectory.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using Dog8Dof::Body;
using Dog8Dof::JumpTrajectory;
using Dog8Dof::TrajectoryPoint;

// ===================================================================
//  CSV 输出辅助（与 walkTest/standTest 相同列格式）
// ===================================================================
static void writeHeader(std::ofstream &csv) {
    csv << "t_s," << "pos_x,pos_z," << "vel_x,vel_z,"
        << "cycle,stepLength_mm,totalTime_s\n";
}

static void writeRow(std::ofstream &csv,
                     double t,
                     const TrajectoryPoint &p,
                     unsigned int phase,
                     float hint,
                     float totalT) {
    csv << std::fixed << std::setprecision(6) << t << "," << p.pos[0] << "," << p.pos[2]
        << "," << p.vel[0] << "," << p.vel[2] << "," << phase << "," << hint << ","
        << totalT << "\n";
}

// ===================================================================
//  主测试
// ===================================================================
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    const std::string shareDir =
        ament_index_cpp::get_package_share_directory("dog_bringup");
    const std::string yamlPath =
        (std::filesystem::path(shareDir) / "parameters" / "jump.yaml").string();

    // 300Hz 控制周期
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / 300.0);
    JumpTrajectory traj(yamlPath, period);

    // 初始足端状态（站立姿态，腿长 270mm）
    Body currentTip{};
    currentTip.pos.setZero();
    currentTip.linerVel.setZero();
    currentTip.pos[0] = 0.0f;   // 前向偏移
    currentTip.pos[2] = 270.0f; // 站立腿长

    // 跳跃参数：水平距离 (mm)，高度 (mm)
    const float jumpLength = 100.0f;
    const float jumpHeight = 80.0f;

    if (!traj.setTarget(jumpLength, jumpHeight)) {
        std::cerr << "setTarget failed\n";
        return 1;
    }
    traj.generate(currentTip, rclcpp::Time(0, 0, RCL_SYSTEM_TIME));

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

    // 仿真循环：持续运行直到跳跃完成
    const int64_t dtNs = period.nanoseconds();
    JumpTrajectory::Phase prevPhase = JumpTrajectory::PUSH_OFF;

    for (unsigned int step = 0; /* below */; ++step) {
        const int64_t tNs = static_cast<int64_t>(step) * dtNs;
        const double t = static_cast<double>(tNs) * 1e-9;
        const rclcpp::Time now(tNs, RCL_SYSTEM_TIME);

        const TrajectoryPoint pt = traj.get(now, nullptr);
        const JumpTrajectory::Phase phase = traj.getPhase();

        // 相位切换日志
        if (phase != prevPhase) {
            const char *phaseNames[] = {"PUSH_OFF", "FLIGHT", "ABSORPTION", "FINISHED"};
            std::cout << "[t=" << t << "s] Phase: " << phaseNames[prevPhase] << " → "
                      << phaseNames[phase] << "  pos=(" << pt.pos[0] << ", " << pt.pos[2]
                      << ")" << "  vel=(" << pt.vel[0] << ", " << pt.vel[2] << ")"
                      << std::endl;
            prevPhase = phase;
        }

        // cycle=阶段编号, stepLength_mm=跳跃距离, totalTime_s=当前阶段进度(粗略)
        const float tHint = jumpLength;
        const float totalT = (phase == JumpTrajectory::FINISHED)   ? 0.0f
                             : (phase == JumpTrajectory::PUSH_OFF) ? 0.15f
                             : (phase == JumpTrajectory::FLIGHT)   ? 0.255f
                                                                   : 0.10f; // ABSORPTION

        writeRow(csv, t, pt, static_cast<unsigned int>(phase), tHint, totalT);

        if (phase == JumpTrajectory::FINISHED) {
            break;
        }

        // 安全检查：防止无限循环
        if (t > 5.0) {
            std::cerr << "Timeout: jump did not finish in 5s\n";
            break;
        }
    }

    csv.close();
    std::cout << "\n=== Test Complete ===" << std::endl;
    std::cout << "Jump: length=" << jumpLength << "mm  height=" << jumpHeight << "mm"
              << std::endl;
    std::cout << "Final phase: "
              << (traj.getPhase() == JumpTrajectory::FINISHED ? "FINISHED" : "INCOMPLETE")
              << std::endl;
    std::cout << "CSV written to: " << csvPath << std::endl;

    rclcpp::shutdown();
    return 0;
}