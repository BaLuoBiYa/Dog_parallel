#pragma once

// #include "dog_controller/controller/balance_controller.hpp"
#include "dog_tools/filters/lowpass_filter.hpp"
#include "dog_trajectory/base.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace Dog8Dof {

    class StandTrajectory : public BaseTrajectory {
      public:
        explicit StandTrajectory(const std::string &configPath, const Body &initTipPos, const rclcpp::Time &initTime);

        TrajectoryPoint generate(const rclcpp::Time currentTime, const Body &currentTip) override;

        bool start(const rclcpp::Time currentTime, const Body &currentTip) override;
        bool reloadParameters() override;

        struct parameters_t {
            float standKp;
            float standKd;
            float transitionKp;
            float transitionKd;
            float filterAlpha;
            float reachThresholdSQ;
            rclcpp::Duration duration = rclcpp::Duration::from_seconds(0.0);
            Eigen::Vector3<float> standTipPos;
        };

      private:
        LowPassFilter xFilter_;
        LowPassFilter zFilter_;

        Eigen::Vector3<float> startTipPos_;

        parameters_t parameters_;
    };

} // namespace Dog8Dof
