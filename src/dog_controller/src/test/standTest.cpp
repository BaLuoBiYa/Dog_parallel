/**
 * @file standTest.cpp
 * @brief StandTrajectory 在线重规划多目标测试
 *
 * 测试：
 *   1. 单次 transition → hold，验证位置/速度 C² 连续
 *   2. 在线切换目标，验证重规划无缝衔接
 *   3. 输出 CSV 便于可视化
 */

#include "dog_trajectory/standTrajectory.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using Dog8Dof::Body;
using Dog8Dof::StandTrajectory;
using Dog8Dof::TrajectoryPoint;

// ===================================================================
//  CSV 输出辅助（与 walkTest 相同列格式，以便同一可视化脚本处理）
// ===================================================================
static void writeHeader(std::ofstream &csv) {
    csv << "t_s," << "pos_x,pos_z," << "vel_x,vel_z,"
        << "cycle,stepLength_mm,totalTime_s\n";
}

static void writeRow(std::ofstream &csv,
                     double t,
                     const TrajectoryPoint &p,
                     unsigned int seg,
                     float hint,
                     float totalT) {
    csv << std::fixed << std::setprecision(6) << t << "," << p.pos[0] << "," << p.pos[2]
        << "," << p.vel[0] << "," << p.vel[2] << "," << seg << "," << hint << ","
        << totalT << "\n";
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    const std::string shareDir =
        ament_index_cpp::get_package_share_directory("dog_bringup");
    const std::string yamlPath =
        (std::filesystem::path(shareDir) / "parameters" / "stand.yaml").string();

    const rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / 300.0);
    StandTrajectory traj(yamlPath, period);

    // 初始足端
    Body currentTip{};
    currentTip.pos.setZero();
    currentTip.linerVel.setZero();
    currentTip.pos[0] = 50.0f;
    currentTip.pos[2] = 270.0f;

    // 多目标测试：在线切换站立位置
    struct TargetChange {
        double trigger_s;
        float tipX;
        float tipZ;
    };
    const std::vector<TargetChange> targets = {
        {0.00, 50.0f, 240.0f}, // t=0:    站到 (50, 240)
        {3.50, 20.0f, 270.0f}, // t=5.5s: 切换到 (20, 270)
        {5.00, 0.0f, 250.0f},  // t=11s:  切换到 (0, 250)
    };

    // 初始目标
    traj.setTarget(targets[0].tipX, targets[0].tipZ);
    traj.generate(currentTip, rclcpp::Time(0, 0, RCL_SYSTEM_TIME));

    std::string csvPath;
    std::cout << "Output CSV path: ";
    std::cin >> csvPath;

    std::ofstream csv(csvPath, std::ios::out | std::ios::trunc);
    if (!csv.is_open()) {
        std::cerr << "Cannot open: " << csvPath << "\n";
        return 1;
    }
    writeHeader(csv);

    const double simDuration = targets.back().trigger_s + 6.0; // 最后一段留 6s 完成过渡
    const int64_t dtNs = period.nanoseconds();
    size_t targetIdx = 0;

    for (unsigned int step = 0; /* below */; ++step) {
        const int64_t tNs = static_cast<int64_t>(step) * dtNs;
        const double t = static_cast<double>(tNs) * 1e-9;
        const rclcpp::Time now(tNs, RCL_SYSTEM_TIME);

        if (t > simDuration)
            break;

        // 在线目标切换
        if (targetIdx + 1 < targets.size() && t >= targets[targetIdx + 1].trigger_s) {
            ++targetIdx;
            const auto &tc = targets[targetIdx];
            std::cout << "[t=" << t << "s] New target: (" << tc.tipX << ", " << tc.tipZ
                      << ")" << std::endl;
            traj.setTarget(tc.tipX, tc.tipZ);
            traj.generate(currentTip, now);
        }

        const TrajectoryPoint pt = traj.get(now, nullptr);

        // 模拟完美跟踪
        currentTip.pos[0] = pt.pos[0];
        currentTip.pos[2] = pt.pos[2];
        currentTip.linerVel[0] = pt.vel[0];
        currentTip.linerVel[2] = pt.vel[2];

        // cycle=目标段索引, stepLength_mm=目标Z（用于标记参变）,
        // totalTime_s=过渡时间(HOLD=0)
        const float tHint = targets[targetIdx].tipZ;
        const float totalT = traj.isHolding() ? 0.0f : 5.0f;
        writeRow(csv, t, pt, static_cast<unsigned int>(targetIdx), tHint, totalT);
    }

    csv.close();
    std::cout << "\n=== Test Complete ===" << std::endl;
    std::cout << "Final phase: " << (traj.isHolding() ? "HOLD" : "TRANSITION")
              << std::endl;
    std::cout << "CSV written to: " << csvPath << std::endl;

    rclcpp::shutdown();
    return 0;
}