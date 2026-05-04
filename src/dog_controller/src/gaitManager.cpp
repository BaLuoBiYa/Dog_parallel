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
                             float upperLength,
                             float lowerLength,
                             float tipLength)
        : clock_(clock), leg_(upperLength, lowerLength, tipLength) {
        walkTrajectory_ = std::make_shared<WalkTrajectory>(walkConfig, period);
        standTrajectory_ = std::make_shared<StandTrajectory>(standConfig, period);
        jumpTrajectory_ = std::make_shared<JumpTrajectory>(jumpConfig, period);
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
        currentTrajectory_ = walkTrajectory_;

        // 根据相位偏移和占空比选择起步相位
        //   phaseOffset < swingDuty  → SWING 起步（跳过已过的部分摆动）
        //   phaseOffset >= swingDuty → STANCE 起步（跳过已过的部分支撑）
        const float swingDuty = walkTrajectory_->swingDuty();
        const float phase = std::fmod(phaseOffset, 1.0f);
        const WalkTrajectory::GaitPhase initGait =
            (phase < swingDuty) ? WalkTrajectory::SWING : WalkTrajectory::STANCE;

        walkTrajectory_->generate(currentTip_, clock_->now(), initGait);
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
    bool GaitManager::setJumpGait(float jumpLength, float jumpHeight) {
        if (!jumpTrajectory_->setTarget(jumpLength, jumpHeight)) {
            throw std::invalid_argument(
                "JumpTrajectory::setTarget failed (invalid jump parameters)");
        }
        currentTrajectory_ = jumpTrajectory_;
        jumpTrajectory_->generate(currentTip_, clock_->now());
        RCLCPP_INFO(logger_,
                    "Jump gait set: length=%.0f height=%.0f",
                    static_cast<double>(jumpLength),
                    static_cast<double>(jumpHeight));
        return true;
    }

    // =========================================================================
    //  setStandGait() — 切换到站立步态
    // =========================================================================
    bool GaitManager::setStandGait(float tipX, float tipZ) {
        if (!standTrajectory_->setTarget(tipX, tipZ)) {
            throw std::invalid_argument(
                "StandTrajectory::setTarget failed (invalid tip coordinates)");
        }
        currentTrajectory_ = standTrajectory_;

        // 以当前足端为起点初始化过渡（TRANSITION）
        standTrajectory_->generate(currentTip_, clock_->now());
        RCLCPP_INFO(logger_,
                    "Stand gait set: target=(%.0f, %.0f)",
                    static_cast<double>(tipX),
                    static_cast<double>(tipZ));
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

        // 2. 在线重规划：传入传感器反馈实现闭环
        targetPoint_ = currentTrajectory_->get(clock_->now(), &currentTip_);

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

        // 5. Walk 自循环 / Jump 完成后回落 Stand
        if (currentTrajectory_ == walkTrajectory_ && walkTrajectory_->finished()) {
            walkTrajectory_->generate(currentTip_, clock_->now(), WalkTrajectory::SWING);
        } else if (currentTrajectory_ == jumpTrajectory_ &&
                   jumpTrajectory_->getPhase() == JumpTrajectory::FINISHED) {
            // 跳跃完成后自动切回站立（以当前足端位置作为站立目标）
            standTrajectory_->setTarget(currentTip_.pos[0], currentTip_.pos[2]);
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

} // namespace Dog8Dof