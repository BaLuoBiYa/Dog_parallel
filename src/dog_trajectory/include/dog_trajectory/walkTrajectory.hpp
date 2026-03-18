#pragma once
#include "dog_tools/cubicBezier.hpp"
#include "dog_trajectory/base.hpp"

namespace Dog8Dof {
    class WalkTrajectory : public BaseTrajectory {
      public:
        explicit WalkTrajectory(const std::string &configPath, rclcpp::Duration period);
        enum sequence { swingFirst, stanceFirst };

        /**
         * @brief 设置步行步态的目标
         * @param stepLength 步长
         * @param stepHeight 步高
         * @param order 从支撑还是摆动相开始
         */
        bool setTarget(float stepLength, float stepHeight, sequence order);
        bool generate(const Body &currentTip) override;
        bool start(const rclcpp::Time &startTime) override;
        TrajectoryPoint get(const rclcpp::Time &currentTime) override;

        bool reloadParameters() override;

        struct parameters_t {
            float walkKp;
            float walkKd;

            float reachThresholdSQ;
            float transitionVelMax;

            float horizontalPushGain;
            float verticalPushGain;

            float totalTime;
            float swingDutyCycle;
        };

      private:
        parameters_t parameters_;
        float stepLength_, stepHeight_;
        sequence order_;

        /**
         * @brief 生成移动到摆动相位初始位置的轨迹
         * @param transitionTime 整个步态用时
         * @param initTip 初始足尖位置
         */
        Trajectory startFromSwing(const float transitionTime, const Body &initTip);

        /**
         * @brief 生成移动到支撑相位初始位置的轨迹
         * @param transitionTime 整个步态用时
         * @param initTip 初始足尖位置
         */
        Trajectory startFromStance(const float transitionTime, const Body &initTip);

        /**
         * @brief 生成支撑相的轨迹
         * @param transitionTime 整个步态用时
         * @param steps 该相位占用的分辨率步数
         */
        Trajectory generateStance(const float transitionTime, const size_t steps);

        /**
         * @brief 生成摆动相的轨迹
         * @param ransitionTime 整个步态用时
         * @param steps 该相位占用的分辨率步数
         */
        Trajectory generateSwing(const float transitionTime, const size_t steps);
    };

} // namespace Dog8Dof