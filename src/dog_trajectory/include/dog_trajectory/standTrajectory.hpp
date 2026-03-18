#pragma once

#include "dog_trajectory/base.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace Dog8Dof {

    class StandTrajectory : public BaseTrajectory {
      public:
        explicit StandTrajectory(const std::string &configPath, rclcpp::Duration period);

        /**
         * @brief 设置站立步态的目标
         * @param tandTipPos 站立足尖位置
         */
        bool setTarget(Body &standTip);
        bool generate(const Body &currentTip) override;
        bool start(const rclcpp::Time &startTime) override;
        TrajectoryPoint get(const rclcpp::Time &currentTime) override;

        bool reloadParameters() override;

        struct parameters_t {
            float standKp;
            float standKd;

            float transitionKp;
            float transitionKd;
            float transitionVelMax;
            float transitionTime;
            float stepHeightFactor;

            float reachThresholdSQ;
        };

      private:
        // LowPassFilter xFilter_;
        // LowPassFilter zFilter_;

        Eigen::Vector2<float> standTipPos_;
        parameters_t parameters_;
    };

} // namespace Dog8Dof
