#pragma once
#include "dog_trajectory/base.hpp"

namespace Dog8Dof {
    class WalkTrajectory : public BaseTrajectory {
      public:
        explicit WalkTrajectory(const std::string &configPath, rclcpp::Duration period);

        /**
         * @brief 设置步行步态的目标
         * @param stepLength 步长
         * @param stepHeight 步高
         * @param baseHeight 基座高度
         * @param phaseOffset 步态相位偏移，范围 [0, 1)，默认为 0
         */
        bool setTarget(float stepLength,
                       float stepHeight,
                       float baseHeight,
                       float phaseOffset = 0.0f,
                       bool useNearestStart = false);
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

            float blendTimeNs;
            float startblendTimeNs;

            Eigen::Vector2f deltaSpeed;
        };

      private:
        parameters_t parameters_;
        float stepLength_, stepHeight_;
        float baseHeight_;
        float phaseOffset_ = 0.0f;
        bool useNearestStart_ = false;

        /**
         * @brief 生成一个完整闭合步态周期
         * @param cycleTime 一个步态周期总时长
         * @param totalSteps 一个步态周期总离散点数
         */
        Trajectory generateClosedCycle(const float cycleTime, const size_t totalSteps);

        /**
         * @brief 在闭合轨迹中找到与当前足端最近的点索引
         * @param cycle 闭合步态轨迹
         * @param currentTip 当前足端状态
         */
        size_t findNearestPointIndex(const Trajectory &cycle,
                                     const Body &currentTip) const;

        /**
         * @brief 将闭合轨迹重排，使 startIndex 对应点成为起点
         */
        Trajectory rotateCycleToStart(const Trajectory &cycle, size_t startIndex) const;

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

        /**
         * @brief 生成过渡的轨迹
         * @param lhs 第一段轨迹
         * @param rhs 第二段轨迹
         */
        Trajectory generateBlend(const Trajectory &lhs, const Trajectory &rhs);

        /**
         * @brief 生成原地踏步的轨迹
         */
        Trajectory generateZeroLength(const Body &currentTip);

        rclcpp::Logger logger_ = rclcpp::get_logger("walkTrajectory");
    };

} // namespace Dog8Dof