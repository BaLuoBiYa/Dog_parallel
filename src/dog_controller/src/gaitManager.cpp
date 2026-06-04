/**
 * @file gaitManager.cpp
 * @brief GaitManager 实现 — 在线重规划 + 逆运动学 + 周期自循环
 */

#include "dog_controller/gaitManager.hpp"

namespace Dog8Dof {

    // =========================================================================
    //  构造
    // =========================================================================
    GaitManager::GaitManager(rclcpp::Clock::SharedPtr clock,
                             rclcpp::Duration period,
                             std::string walkConfig,
                             std::string standConfig,
                             std::string jumpConfig,
                             JumpTrajectory::LegID legID,
                             float upperLength,
                             float lowerLength,
                             float tipLength)
        : clock_(clock), leg_(upperLength, lowerLength, tipLength) {
        walkTrajectory_ = std::make_shared<WalkTrajectory>(walkConfig, period);
        standTrajectory_ = std::make_shared<StandTrajectory>(standConfig, period);
        jumpTrajectory_ = std::make_shared<JumpTrajectory>(jumpConfig, period, legID);
    }

    // =========================================================================
    //  init() — 首次建立腿模型状态
    // =========================================================================
    bool GaitManager::init(const Eigen::Vector2f &pos,
                           const Eigen::Vector2f &vel,
                           const Eigen::Vector2f &tau) {
        if (!leg_.updateMotor(pos, vel, tau)) {
            RCLCPP_ERROR(logger_,
                         "Leg2Dof::updateMotor failed: impossible motor position");
            return false;
        }
        currentTip_ = leg_.getTipCurrent();
        return true;
    }

    // =========================================================================
    //  setWalkGait() — 切换到步行步态
    // =========================================================================
    bool GaitManager::setWalkGait(float stepLength,
                                  float stepHeight,
                                  float baseHeight,
                                  float totalTime,
                                  float phaseOffset) {
        if (!walkTrajectory_->setTarget(
                stepLength, stepHeight, baseHeight, totalTime, phaseOffset)) {
            throw std::invalid_argument(
                "WalkTrajectory::setTarget failed (invalid walk parameters)");
        }

        // 从跳跃步态切换走时，强制重置跳跃轨迹状态
        if (currentTrajectory_ == jumpTrajectory_) {
            jumpTrajectory_->reset();
        }

        const bool isNewMode = (currentTrajectory_ != walkTrajectory_);

        // 记录上一个步态类型
        if (currentTrajectory_ == standTrajectory_) {
            previousGaitType_ = GaitType::STAND;
        } else if (currentTrajectory_ == jumpTrajectory_) {
            previousGaitType_ = GaitType::JUMP;
        }

        currentTrajectory_ = walkTrajectory_;

        // generate() 仅在首次切换到 WALK 模式时调用，避免高频重置轨迹导致抖动
        if (isNewMode) {
            walkTrajectory_->generate(currentTip_, clock_->now());
        }

        RCLCPP_INFO(logger_,
                    "Walk gait set: stepLen=%.0f stepH=%.0f cycleT=%.3f",
                    static_cast<double>(stepLength),
                    static_cast<double>(stepHeight),
                    static_cast<double>(totalTime));
        return true;
    }

    // =========================================================================
    //  setJumpGait() — 切换到跳跃步态
    // =========================================================================
    bool GaitManager::setJumpGait(uint8_t parID) {
        if (!jumpTrajectory_->setTarget(parID)) {
            RCLCPP_ERROR(logger_,
                         "JumpTrajectory::setTarget failed (parID=%d) — is a jump "
                         "already in progress?",
                         parID);
            return false;
        }

        // 记录上一个步态类型
        if (currentTrajectory_ == standTrajectory_) {
            previousGaitType_ = GaitType::STAND;
        } else if (currentTrajectory_ == walkTrajectory_) {
            previousGaitType_ = GaitType::WALK;
        }

        currentTrajectory_ = jumpTrajectory_;
        jumpTrajectory_->generate(currentTip_, clock_->now());
        RCLCPP_INFO(logger_, "Jump gait set: parID=%d", parID);
        return true;
    }

    bool GaitManager::isJumping() const {
        return currentTrajectory_ == jumpTrajectory_ &&
               jumpTrajectory_->getPhase() != JumpTrajectory::FINISHED;
    }

    // =========================================================================
    //  setStandGait() — 切换到站立步态
    // =========================================================================
    bool GaitManager::setStandGait(float tipX, float tipZ, bool useInitTransition) {
        if (!standTrajectory_->setTarget(tipX, tipZ)) {
            throw std::invalid_argument(
                "StandTrajectory::setTarget failed (invalid tip coordinates)");
        }

        // 从跳跃步态切换走时，强制重置跳跃轨迹状态
        if (currentTrajectory_ == jumpTrajectory_) {
            jumpTrajectory_->reset();
        }

        const bool isNewMode = (currentTrajectory_ != standTrajectory_);

        // 记录上一个步态类型
        if (currentTrajectory_ == walkTrajectory_) {
            previousGaitType_ = GaitType::WALK;
        } else if (currentTrajectory_ == jumpTrajectory_) {
            previousGaitType_ = GaitType::JUMP;
        }

        currentTrajectory_ = standTrajectory_;

        // generate() 仅在首次切换到 STAND 模式时调用，避免高频重置轨迹导致抖动
        if (isNewMode) {
            // 根据上一个步态和初始化标志选择合适的缓冲时间
            standTrajectory_->reloadParameters();
            auto params = standTrajectory_->getParameters();

            float transitionTime = params.transitionTime; // 默认

            if (useInitTransition) {
                // 初始化站起
                transitionTime = params.transitionTimeInit;
                RCLCPP_INFO(logger_, "Using init transition time: %.2f s", transitionTime);
            } else if (previousGaitType_ == GaitType::WALK) {
                // Walk → Stand
                transitionTime = params.transitionTimeWalkToStand;
                RCLCPP_INFO(logger_, "Using walk-to-stand transition time: %.2f s", transitionTime);
            } else if (previousGaitType_ == GaitType::JUMP) {
                // Jump → Stand (通过Walk中转)
                transitionTime = params.transitionTimeJumpToWalk;
                RCLCPP_INFO(logger_, "Using jump-to-walk transition time: %.2f s", transitionTime);
            }

            // 使用自定义过渡时间配置并生成轨迹
            standTrajectory_->configure(params, tipX, tipZ, transitionTime);
            standTrajectory_->generate(currentTip_, clock_->now());

            RCLCPP_INFO(logger_,
                        "Stand gait set: target=(%.0f, %.0f), transition=%.2f s",
                        static_cast<double>(tipX),
                        static_cast<double>(tipZ),
                        static_cast<double>(transitionTime));
        }

        return true;
    }

    // =========================================================================
    //  update() — 每控制周期的核心入口
    // =========================================================================
    bool GaitManager::update(const Eigen::Vector2f &pos,
                             const Eigen::Vector2f &vel,
                             const Eigen::Vector2f &tau) {
        // 1. 更新腿模型：电机状态 → 足端位姿
        if (!leg_.updateMotor(pos, vel, tau)) {
            RCLCPP_ERROR(logger_,
                         "Leg2Dof::updateMotor failed: impossible motor position");
            return false;
        }
        currentTip_ = leg_.getTipCurrent();

        if (!currentTrajectory_) {
            RCLCPP_ERROR(logger_, "No current trajectory set");
            return false;
        }

        // 2. 在线重规划：跳跃模式传入当前足端真值做反馈，其余模式开环
        targetPoint_ = (currentTrajectory_ == jumpTrajectory_)
                           ? currentTrajectory_->get(clock_->now(), &currentTip_)
                           : currentTrajectory_->get(clock_->now(), nullptr);

        // 3. 逆运动学：期望足端 → 期望关节角度
        if (!leg_.setTip(targetPoint_.pos, targetPoint_.vel, targetPoint_.tau)) {
            RCLCPP_ERROR(logger_, "Leg2Dof::setTip failed: target tip unreachable");
            return false;
        }

        // 4. PD 增益下发
        if (!leg_.updateKp(Leg2Dof::FRONT, targetPoint_.kp[0]) ||
            !leg_.updateKp(Leg2Dof::REAR, targetPoint_.kp[1]) ||
            !leg_.updateKd(Leg2Dof::FRONT, targetPoint_.kd[0]) ||
            !leg_.updateKd(Leg2Dof::REAR, targetPoint_.kd[1])) {
            RCLCPP_ERROR(logger_, "Leg2Dof::updateKp/Kd failed");
            return false;
        }

        // Jump 完成后回落 Stand
        if (currentTrajectory_ == jumpTrajectory_ &&
            jumpTrajectory_->getPhase() == JumpTrajectory::FINISHED) {
            // 跳跃完成后自动切回站立（以轨迹目标位置为准，避免传感器滞后导致位置错误）
            standTrajectory_->setTarget(targetPoint_.pos[0], targetPoint_.pos[2]);
            standTrajectory_->generate(currentTip_, clock_->now());
            currentTrajectory_ = standTrajectory_;
            RCLCPP_INFO(logger_, "Jump finished, fallback to stand");
        }

        return true;
    }

    // =========================================================================
    //  isFinished() — Stand 到达 HOLD 则完成，Walk 永不完成，Jump 完成即 true
    // =========================================================================
    bool GaitManager::isFinished() const {
        if (!currentTrajectory_)
            return true;
        if (currentTrajectory_ == walkTrajectory_)
            return false;
        if (currentTrajectory_ == jumpTrajectory_)
            return jumpTrajectory_->getPhase() == JumpTrajectory::FINISHED;
        return standTrajectory_->isHolding();
    }

    // =========================================================================
    //  电机输出
    // =========================================================================
    Motor GaitManager::getFrontTarget() const {
        return leg_.getMotorTarget(Leg2Dof::FRONT);
    }

    Motor GaitManager::getRearTarget() const {
        return leg_.getMotorTarget(Leg2Dof::REAR);
    }

    Body GaitManager::getTipCurrent() const {
        return currentTip_;
    }

    Body GaitManager::getTipTarget() const {
        return Body(targetPoint_.pos, targetPoint_.vel, targetPoint_.tau);
    }

} // namespace Dog8Dof