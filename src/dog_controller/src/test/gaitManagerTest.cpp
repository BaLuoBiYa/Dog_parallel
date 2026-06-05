/**
 * @file gaitManagerTest.cpp
 * @brief GaitManager 端到端测试 — 模拟电机反馈闭环在线重规划
 *
 * 测试流程：
 *   1. 通过逆运动学从初始足端位姿(50,270)求出初始电机角度
 *   2. GaitManager::init() 建立腿模型 → setWalkGait() 初始化步行
 *   3. 每周期调用 update()，用上一周期的电机目标作为反馈（模拟完美跟踪）
 *   4. 通过辅助 Leg2Dof 将电机目标反算为足端坐标，输出 CSV
 *   5. 中途切换步态参数，验证 GaitManager 的在线重规划与新接口
 *
 * CSV 列格式与 walkTest/standTest/jumpTest 对齐
 */

#include "dog_controller/gaitManager.hpp"
#include "dog_trajectory/jumpTrajectory.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using Dog8Dof::Body;
using Dog8Dof::Const;
using Dog8Dof::GaitManager;
using Dog8Dof::Leg2Dof;
using Dog8Dof::Motor;

// ===================================================================
//  CSV 输出辅助
// ===================================================================
static void writeHeader(std::ofstream &csv) {
    csv << "t_s," << "pos_x,pos_z," << "vel_x,vel_z,"
        << "cycle,stepLength_mm,totalTime_s\n";
}

static void writeRow(std::ofstream &csv,
                     double t,
                     const Body &tip,
                     unsigned int cycle,
                     float stepLen,
                     float totalT) {
    csv << std::fixed << std::setprecision(6) << t << "," << tip.pos[0] << ","
        << tip.pos[2] << "," << tip.linerVel[0] << "," << tip.linerVel[2] << "," << cycle
        << "," << stepLen << "," << totalT << "\n";
}

// ===================================================================
//  主测试
// ===================================================================
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // ---- 加载配置路径 ----
    const std::string shareDir =
        ament_index_cpp::get_package_share_directory("dog_bringup");
    const std::string walkYaml =
        (std::filesystem::path(shareDir) / "parameters" / "walk.yaml").string();
    const std::string standYaml =
        (std::filesystem::path(shareDir) / "parameters" / "stand.yaml").string();
    const std::string jumpYaml =
        (std::filesystem::path(shareDir) / "parameters" / "jump.yaml").string();

    // 300Hz 控制周期
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / 300.0);

    // ---- 辅助腿模型（用于 IK/FK 转换） ----
    Leg2Dof verifyLeg(Const::UPPER_LENGTH, Const::LOWER_LENGTH, 0.0f);

    // 通过逆运动学求初始电机角度：(50mm, 270mm) → 前后电机角度
    Eigen::Vector3f initTipPos(50.0f, 0.0f, 270.0f);
    Eigen::Vector3f initTipVel(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f initTipTau(0.0f, 0.0f, 0.0f);
    if (!verifyLeg.setTip(initTipPos, initTipVel, initTipTau)) {
        std::cerr << "Failed to compute initial motor angles\n";
        return 1;
    }
    Motor frontInit = verifyLeg.getMotorTarget(Leg2Dof::FRONT);
    Motor rearInit = verifyLeg.getMotorTarget(Leg2Dof::REAR);

    // ---- 创建 GaitManager ----
    GaitManager gm(period,
                   walkYaml,
                   standYaml,
                   jumpYaml,
                   Dog8Dof::JumpTrajectory::LF,
                   Const::UPPER_LENGTH,
                   Const::LOWER_LENGTH);

    // 初始化：传入初始电机角度，建立腿模型
    if (!gm.init(Eigen::Vector2f(frontInit.pos, rearInit.pos),
                 Eigen::Vector2f(0.0f, 0.0f),
                 Eigen::Vector2f(0.0f, 0.0f))) {
        std::cerr << "GaitManager init failed\n";
        return 2;
    }

    // ---- 设置步行步态 ----
    const float baseHeight = 270.0f;
    float stepLength = 50.0f;
    float stepHeight = 30.0f;
    float totalTime = 0.5f;

    if (!gm.setWalkGait(stepLength, stepHeight, baseHeight, totalTime, 0.0f)) {
        std::cerr << "setWalkGait failed\n";
        return 3;
    }

    // ---- 参数变更计划 ----
    struct ParamChange {
        double trigger_s;
        float newStepLength;
        float newTotalTime;
    };
    const std::vector<ParamChange> changes = {
        {0.00, 50.0f, 0.5f},  // t=0: 50mm/周期
        {1.50, 100.0f, 0.5f}, // t=1.5s: 加速大步长
        {3.00, 30.0f, 0.3f},  // t=3.0s: 小步快频
        {4.50, 0.0f, 0.5f},   // t=4.5s: 原地踏步
    };

    // CSV 输出
    std::string csvPath;
    std::cout << "Output CSV path: ";
    std::cin >> csvPath;

    std::ofstream csv(csvPath, std::ios::out | std::ios::trunc);
    if (!csv.is_open()) {
        std::cerr << "Cannot open: " << csvPath << "\n";
        return 4;
    }
    writeHeader(csv);

    // ---- 主仿真循环 ----
    const double simDuration = changes.back().trigger_s + 2.0;
    const int64_t dtNs = period.nanoseconds();
    size_t changeIdx = 0;
    unsigned int completedCycles = 0;
    double lastCycleEndTime = 0.0;

    // 电机反馈初始值（从 IK 求得）
    float frontPos = frontInit.pos, rearPos = rearInit.pos;
    float frontVel = 0.0f, rearVel = 0.0f;
    float frontTau = 0.0f, rearTau = 0.0f;

    for (unsigned int step = 0; /* below */; ++step) {
        const int64_t tNs = static_cast<int64_t>(step) * dtNs;
        const double t = static_cast<double>(tNs) * 1e-9;

        if (t > simDuration)
            break;

        // ---- 在线参数变更 ----
        if (changeIdx < changes.size() && t >= changes[changeIdx].trigger_s) {
            const auto &ch = changes[changeIdx];
            stepLength = ch.newStepLength;
            totalTime = ch.newTotalTime;

            std::cout << "[t=" << t << "s] Change: stepLength=" << stepLength
                      << "mm  totalTime=" << totalTime << "s" << std::endl;

            // 通过 GaitManager 切换步态（内部自动 setTarget → generate）
            gm.setWalkGait(stepLength, stepHeight, baseHeight, totalTime, 0.0f);

            ++changeIdx;
        }

        // ---- 每周期调用 GaitManager::update() ----
        // 传入上一周期的电机目标（位置、速度、力矩）作为当前反馈
        gm.update(Eigen::Vector2f(frontPos, rearPos),
                  Eigen::Vector2f(frontVel, rearVel),
                  Eigen::Vector2f(frontTau, rearTau));

        // 获取本周期电机目标，保存为下一周期的反馈
        Motor front = gm.getFrontTarget();
        Motor rear = gm.getRearTarget();
        frontPos = front.pos;
        rearPos = rear.pos;
        frontVel = front.vel;
        rearVel = rear.vel;
        frontTau = front.tau;
        rearTau = rear.tau;

        // ---- 从电机目标反算足端坐标（用于 CSV 输出） ----
        verifyLeg.updateMotor(Eigen::Vector2f(front.pos, rear.pos),
                              Eigen::Vector2f(front.vel, rear.vel),
                              Eigen::Vector2f(front.tau, rear.tau));
        Body tip = verifyLeg.getTipCurrent();

        // ---- 周期计数 ----
        if (t - lastCycleEndTime >= totalTime) {
            ++completedCycles;
            lastCycleEndTime += totalTime;
            std::cout << "[t=" << t << "s] Cycle " << completedCycles
                      << " completed, tip=(" << tip.pos[0] << ", " << tip.pos[2] << ")"
                      << std::endl;
        }

        writeRow(csv, t, tip, completedCycles, stepLength, totalTime);

        // 按控制周期延时，确保模拟时间与实际时间同步
        rclcpp::sleep_for(
            std::chrono::nanoseconds(static_cast<int64_t>(1.0e9 / 300.0)));
    }

    csv.close();
    std::cout << "\n=== Test Complete ===" << std::endl;
    std::cout << "Total cycles: " << completedCycles << std::endl;
    std::cout << "CSV written to: " << csvPath << std::endl;

    rclcpp::shutdown();
    return 0;
}
