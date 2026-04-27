#include "dog_controller/gaitManager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

using namespace std::chrono;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    const std::string share_dir =
        ament_index_cpp::get_package_share_directory("dog_bringup");
    const std::string walkProfile =
        (std::filesystem::path(share_dir) / "parameters" / "walk.yaml").string();
    const std::string standProfile =
        (std::filesystem::path(share_dir) / "parameters" / "stand.yaml").string();
    const std::string jumpProfile =
        (std::filesystem::path(share_dir) / "parameters" / "jumpLegacy.yaml").string();

    auto clock = std::make_shared<rclcpp::Clock>();
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / (1000.0 * 10));

    auto gm = std::make_unique<Dog8Dof::GaitManager>(
        clock, period, walkProfile, standProfile, jumpProfile);

    // 初始化为合理的电机状态（零位）
    Eigen::Vector2f pos;
    pos.setZero();
    Eigen::Vector2f vel;
    vel.setZero();
    Eigen::Vector2f tau;
    tau.setZero();
    if (!gm->init(pos, vel, tau)) {
        std::cerr << "GaitManager init failed" << std::endl;
        return 1;
    }

    // 测试参数
    const int iterations = 200;
    std::vector<long long> durations;
    durations.reserve(iterations);
    int success = 0;
    int failed = 0;

    std::mt19937_64 rng(123456);
    std::uniform_real_distribution<float> stepLenDist(40.0f, 80.0f);
    std::uniform_real_distribution<float> stepHeightDist(30.0f, 70.0f);
    std::uniform_real_distribution<float> baseHeightDist(80.0f, 300.0f);
    std::uniform_real_distribution<float> phaseDist(0.0f, 1.0f);

    for (int i = 0; i < iterations; ++i) {
        const float stepLength = stepLenDist(rng);
        const float stepHeight = stepHeightDist(rng);
        const float baseHeight = baseHeightDist(rng);
        const float phaseOffset = phaseDist(rng);

        try {
            const auto t0 = high_resolution_clock::now();
            bool ok = gm->setWalkGait(stepLength, stepHeight, baseHeight, phaseOffset);
            const auto t1 = high_resolution_clock::now();
            const auto us = duration_cast<microseconds>(t1 - t0).count();

            if (ok) {
                durations.push_back(us);
                ++success;
            } else {
                ++failed;
            }
        } catch (const std::exception &e) {
            ++failed;
        }

        if ((i + 1) % 50 == 0) {
            std::cout << "Progress: " << (i + 1) << "/" << iterations << std::endl;
        }
    }

    if (!durations.empty()) {
        const auto minv = *std::min_element(durations.begin(), durations.end());
        const auto maxv = *std::max_element(durations.begin(), durations.end());
        const double mean =
            std::accumulate(durations.begin(), durations.end(), 0.0) / durations.size();
        std::sort(durations.begin(), durations.end());
        const auto median = durations[durations.size() / 2];

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "GaitManager replan (setWalkGait) stats over " << durations.size()
                  << " runs:\n";
        std::cout << "  success=" << success << " failed=" << failed << "\n";
        std::cout << "  min=" << (minv / 1000.0) << " ms  max=" << (maxv / 1000.0)
                  << " ms  mean=" << (mean / 1000.0)
                  << " ms  median=" << (median / 1000.0) << " ms\n";
    } else {
        std::cout << "No successful replans. success=" << success << " failed=" << failed
                  << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}