#pragma once
#include "dog_tools/filters/lowpass_filter.hpp"
#include "dog_tools/filters/smoother.hpp"
#include "dog_trajectory/base.hpp"

namespace Dog8Dof {
    class WalkTrajectory : public BaseTrajectory {
      public:
        enum Phase { STANCE, SWING };
        struct parameters_t {
            float walkKp;
            float walkKd;
            float filterAlpha;
            float horizontalPushGain;
            float verticalPushGain;
            float accelerationGainX;
            float accelerationGainZ;

            bool useSmootherTrajectory_ = false;
            float smootherW = 30.0;
            float smootherD = 0.8;

            float swingContactThreshold;

            rclcpp::Duration duration = rclcpp::Duration::from_seconds(0.0);
        };

        explicit WalkTrajectory(const std::string &configPath, const Body &initTipPos, const rclcpp::Time &initTime);

        TrajectoryPoint generate(const rclcpp::Time currentTime, const Body &currentTip) override;

        bool start(const rclcpp::Time currentTime, const Body &currentTip) override;
        bool reloadParameters() override;

        /**
         * @brief 设置轨迹规划目标
         * @param phase 支撑还是摆动
         * @param stepLength 步长
         * @param stepHeight 步高
         */
        bool setTarget(Phase phase, float stepLength, float stepHeight);

      private:
        parameters_t parameters_;
        Smoother xSmoother_;
        Smoother zSmoother_;

        LowPassFilter xFilter_, zFilter_;
        LowPassFilter vxFilter_, vzFilter_;

        bool landingMode_;
        Eigen::Vector3<float> landingStartPos_;
        float landingProgress_;
        Eigen::Vector3<float> lastPos_;

        float stepLength_;
        float stepHeight_;

        Phase phase_;
    };

} // namespace Dog8Dof