#include "dog_controller/gaitManager.hpp"
namespace Dog8Dof {
    GaitManager::GaitManager(rclcpp::Clock::SharedPtr clock,
                             rclcpp::Duration period,

                             std::string walkConfig,
                             std::string standConfig,
                             std::string jumpConfig,

                             float upperLength,
                             float lowerLength,
                             float tipLength)
        : clock_(clock), leg_(upperLength, lowerLength, tipLength),
          walkTrajectory_(walkConfig, period), standTrajecory_(standConfig, period),
          jumpTrajectory_(jumpConfig, period) {
    }

    bool GaitManager::setJumpGait(const JumpTrajectoryLegacy::jumpIndex &jumpIndex) {
        if (!jumpTrajectory_.setTarget(jumpIndex)) {
            throw std::invalid_argument(
                "JumpTrajectoryLegacy::setTarget failed for config: " +
                jumpTrajectory_.jumpMapping.at(jumpIndex));
        }
        currentTrajectory_ = std::make_unique<JumpTrajectoryLegacy>(jumpTrajectory_);
        if (!currentTrajectory_->generate(currentTip_)) {
            currentTrajectory_.reset();
            throw std::runtime_error("JumpTrajectoryLegacy::generate failed");
        }
        if (!currentTrajectory_->start(clock_->now())) {
            currentTrajectory_.reset();
            throw std::runtime_error("JumpTrajectoryLegacy::start failed");
        }
        return true;
    }

    bool GaitManager::setWalkGait(float stepLength, float stepHeight, float baseHeight) {
        if (!walkTrajectory_.setTarget(stepLength, stepHeight, baseHeight)) {
            throw std::invalid_argument(
                "WalkTrajectory::setTarget failed (invalid walk parameters)");
        }
        currentTrajectory_ = std::make_unique<WalkTrajectory>(walkTrajectory_);
        if (!currentTrajectory_->generate(currentTip_)) {
            currentTrajectory_.reset();
            throw std::runtime_error("WalkTrajectory::generate failed");
        }
        if (!currentTrajectory_->start(clock_->now())) {
            currentTrajectory_.reset();
            throw std::runtime_error("WalkTrajectory::start failed");
        }
        return true;
    }

    bool GaitManager::setStandGait(float tipX, float tipZ) {
        if (!standTrajecory_.setTarget(tipX, tipZ)) {
            throw std::invalid_argument(
                "StandTrajectory::setTarget failed (invalid tip coordinates)");
        }
        currentTrajectory_ = std::make_unique<StandTrajectory>(standTrajecory_);
        if (!currentTrajectory_->generate(currentTip_)) {
            currentTrajectory_.reset();
            throw std::runtime_error("StandTrajectory::generate failed");
        }
        if (!currentTrajectory_->start(clock_->now())) {
            currentTrajectory_.reset();
            throw std::runtime_error("StandTrajectory::start failed");
        }
        return true;
    }

    bool GaitManager::update(const Eigen::Vector2f &pos,
                             const Eigen::Vector2f &vel,
                             const Eigen::Vector2f &tau) {
        auto logger = rclcpp::get_logger("GaitManager");

        // 更新电机状态到腿模型
        if (!leg_.updateMotor(pos, vel, tau)) {
            RCLCPP_ERROR(logger,
                         "Leg2Dof::updateMotor failed: impossible motor position");
            return false;
        }

        // 同步 currentTip_
        currentTip_ = leg_.getTipCurrent();

        if (!currentTrajectory_) {
            RCLCPP_ERROR(logger, "No current trajectory set");
            return false;
        }

        targetPoint_ = currentTrajectory_->get(clock_->now());
        // 获取目标点

        if (!leg_.setTip(targetPoint_.pos, targetPoint_.vel, targetPoint_.tau)) {
            RCLCPP_ERROR(logger, "Leg2Dof::setTip failed: target tip unreachable");
            return false;
        }

        if (!leg_.updateKp(Leg2Dof::FRONT, targetPoint_.kp[0])) {
            RCLCPP_ERROR(logger, "Leg2Dof::updateKp failed: kp out of range");
            return false;
        }
        if (!leg_.updateKp(Leg2Dof::REAR, targetPoint_.kp[1])) {
            RCLCPP_ERROR(logger, "Leg2Dof::updateKp failed: kp out of range");
            return false;
        }

        if (!leg_.updateKd(Leg2Dof::FRONT, targetPoint_.kd[0])) {
            RCLCPP_ERROR(logger, "Leg2Dof::updateKd failed: kd out of range");
            return false;
        }
        if (!leg_.updateKd(Leg2Dof::REAR, targetPoint_.kd[1])) {
            RCLCPP_ERROR(logger, "Leg2Dof::updateKd failed: kd out of range");
            return false;
        }

        return true;
    }

} // namespace Dog8Dof