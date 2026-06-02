/**
 * @file walkTest.cpp
 * @brief WalkTrajectory 在线重规划多周期测试
 *
 * 测试目标：
 *   1. 多个完整 swing↔stance↔swing 周期连续运行，验证跨相位位置/速度 C² 连续性
 *   2. 在线修改步长/速度，验证 Raibert 落脚点+低通滤波的在线重规划能力
 *   3. 零步长（原地踏步）切换，验证静止→运动→后退→静止的平滑过渡
 *   4. 输出 CSV，便于外部工具绘图验证
 *
 * 仿真环境：
 *   控制频率 300Hz，足端无传感器反馈（空指针），模拟纯开环轨迹质量
 */

#include "dog_trajectory/walkTrajectory.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using Dog8Dof::Body;
using Dog8Dof::TrajectoryPoint;
using Dog8Dof::WalkTrajectory;

// ===================================================================
//  CSV 输出辅助
// ===================================================================
static void writeHeader(std::ofstream &csv) {
    csv << "t_s," << "pos_x,pos_z," << "vel_x,vel_z,"
        << "cycle,stepLength_mm,totalTime_s\n";
}

static void writeRow(std::ofstream &csv,
                     double t,
                     const TrajectoryPoint &p,
                     unsigned int cycle,
                     float stepLen,
                     float totalT) {
    csv << std::fixed << std::setprecision(6) << t << "," << p.pos[0] << "," << p.pos[2]
        << "," << p.vel[0] << "," << p.vel[2] << "," << cycle << "," << stepLen << ","
        << totalT << "\n";
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
        (std::filesystem::path(shareDir) / "parameters" / "walk.yaml").string();

    // 300Hz 控制周期
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / 300.0);
    WalkTrajectory traj(yamlPath, period);

    // ---- 初始足端状态（站立姿态） ----
    Body currentTip{};
    currentTip.pos.setZero();
    currentTip.linerVel.setZero();
    currentTip.pos[0] = 0.0f;
    currentTip.pos[2] = 270.0f; // 站立高度

    // ---- 初始步态参数 ----
    const float baseHeight = 270.0f;
    float stepLength = 200.0f; // 当前步长 (mm)
    float stepHeight = 30.0f;  // 步高 (mm)
    float totalTime = 0.5f;    // 单周期总时间 (s)
    float phaseOffset = 0.5f;

    // 测试计划：在不同时间点在线修改步态参数
    struct ParamChange {
        double triggerTime_s;
        float newStepLength;
        float newTotalTime;
    };
    // const std::vector<ParamChange> changes = {
    //     {0.00, 200.0f, 0.5f},  // 前进 50mm/周期
    //     {1.40, 100.0f, 0.5f},  // 加速（大步长）
    //     {1.50, 300.0f, 0.4f},  // 小步幅 + 加快步频
    //     {3.75, 0.0f, 0.5f},    // 原地踏步（零步长）
    //     {4.30, -300.0f, 0.5f}, // 后退
    //     {5.15, 0.0f, 0.5f},    // 恢复原地踏步
    // };

    const std::vector<ParamChange> changes = {
        {0.00, 0.0f, 0.5f},
    }; // 同一周期内修改步长和周期时间，验证 Raibert 公式的在线重规划能力

    // 初始化第一个参数段
    if (!traj.setTarget(stepLength, stepHeight, baseHeight, totalTime, phaseOffset)) {
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

    // ---- 主仿真循环 ----
    const double simDuration =
        changes.back().triggerTime_s + 2.0; // 运行到最后一次参数变更后再多跑一段时间
    const int64_t dtNs = period.nanoseconds();

    size_t changeIdx = 0;
    unsigned int completedCycles = 0;
    // 上次周期结束时刻（基于 totalTime 的外部追踪）
    double lastCycleEndTime = 0.0;

    for (unsigned int step = 0; /* below */; ++step) {
        const int64_t tNs = static_cast<int64_t>(step) * dtNs;
        const double t = static_cast<double>(tNs) * 1e-9;
        const rclcpp::Time now(tNs, RCL_SYSTEM_TIME);

        if (t > simDuration)
            break;

        // ---- 在线参数变更 ----
        if (changeIdx < changes.size() && t >= changes[changeIdx].triggerTime_s) {
            const auto &ch = changes[changeIdx];
            stepLength = ch.newStepLength;
            totalTime = ch.newTotalTime;

            std::cout << "[t=" << t << "s] Change: stepLength=" << stepLength
                      << "mm  totalTime=" << totalTime << "s" << std::endl;

            // setTarget 更新内部 vDes_/swingTime_/stanceTime_
            // 当下一个摆动相 update() 时，新参数通过 Raibert 公式自然生效
            if (!traj.setTarget(
                    stepLength, stepHeight, baseHeight, totalTime, phaseOffset)) {
                std::cerr << "setTarget failed at t=" << t << "\n";
                return 3;
            }

            ++changeIdx;
        }

        // ---- 获取轨迹点（无传感器反馈，仅开环轨迹） ----
        const TrajectoryPoint pt = traj.get(now, nullptr);

        // 更新"足端状态"以模拟完美跟踪
        currentTip.pos[0] = pt.pos[0];
        currentTip.pos[2] = pt.pos[2];
        currentTip.linerVel[0] = pt.vel[0];
        currentTip.linerVel[2] = pt.vel[2];

        // ---- 周期计数：基于外部时间的简单方案 ----
        // 每当经过一个完整周期（totalTime），计数一次
        if (t - lastCycleEndTime >= totalTime) {
            ++completedCycles;
            lastCycleEndTime += totalTime;
            std::cout << "[t=" << t << "s] Cycle " << completedCycles
                      << " completed, stepLength=" << stepLength
                      << "mm, pos_x=" << pt.pos[0] << std::endl;
        }

        writeRow(csv, t, pt, completedCycles, stepLength, totalTime);
    }

    csv.close();
    std::cout << "\n=== Test Complete ===" << std::endl;
    std::cout << "Total cycles: " << completedCycles << std::endl;
    std::cout << "CSV written to: " << csvPath << std::endl;

    rclcpp::shutdown();
    return 0;
}