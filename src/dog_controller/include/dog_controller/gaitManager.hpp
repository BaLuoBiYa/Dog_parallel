#pragma once
#include "dog_controller/leg.hpp"
#include "dog_trajectory/jumpTrajectoryLegacy.hpp"
#include "dog_trajectory/standTrajectory.hpp"
#include "dog_trajectory/walkTrajectory.hpp"
namespace Dog8Dof {
    class GaitManager {
      public:
        GaitManager(rclcpp::Clock::SharedPtr clock,
                    rclcpp::Duration period,

                    std::string walkConfig,
                    std::string standConfig,
                    std::string jumpConfig,

                    float upperLength = Const::UPPER_LENGTH,
                    float lowerLength = Const::LOWER_LENGTH,
                    float tipLength = 0.0f);

        /**
         * @brief 设置目标步态参数
         * @return 是否成功更新模型状态
         */
        bool setJumpGait(const JumpTrajectoryLegacy::jumpIndex &jumpIndex);
        bool setWalkGait(float stepLength,
                         float stepHeight,
                         float baseHeight,
                         float phaseOffset = 0.0f,
                         bool useNearestStart = false);
        bool setStandGait(float tipX, float tipZ);

        /**
         * @brief 启动步态
         */
        bool start(const rclcpp::Time &startTime);

        /**
         * @brief 检测步态周期是否结束
         */
        bool isFinshed() {
            return currentTrajectory_ ? currentTrajectory_->finished() : true;
        }

        /**
         * @brief 更新当前姿态
         * @param pos 前后电机位置。
         * @param vel 前后电机速度。
         * @param tau 前后电机力矩。
         * @return 是否成功更新模型状态
         */
        bool update(const Eigen::Vector2f &pos,
                    const Eigen::Vector2f &vel,
                    const Eigen::Vector2f &tau);

        Motor getFrontTarget() {
            return leg_.getMotorTarget(Leg2Dof::FRONT);
        };
        Motor getRearTarget() {
            return leg_.getMotorTarget(Leg2Dof::REAR);
        }

      private:
        rclcpp::Clock::SharedPtr clock_;
        Leg2Dof leg_;
        std::unique_ptr<BaseTrajectory> currentTrajectory_;
        TrajectoryPoint targetPoint_;
        Body currentTip_;

        WalkTrajectory walkTrajectory_;
        StandTrajectory standTrajecory_;
        JumpTrajectoryLegacy jumpTrajectory_;
    };
} // namespace Dog8Dof
