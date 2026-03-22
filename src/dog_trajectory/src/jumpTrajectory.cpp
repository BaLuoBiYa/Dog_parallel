#include "dog_trajectory/jumpTrajectory.hpp"
#include "dog_tools/quinticPolynomial.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace Dog8Dof {
    namespace {
        constexpr float kEps = 1e-6f;
        constexpr float kMinPhaseTime = 0.05f;
        constexpr float kPi = 3.14159265358979323846f;
    } // namespace

    JumpTrajectory::JumpTrajectory(const std::string &configPath, rclcpp::Duration period)
        : BaseTrajectory(configPath, period) {
        if (!reloadParameters()) {
            throw std::invalid_argument("Could not load Config YAML!");
        }
    }

    bool JumpTrajectory::reloadParameters() {
        configFile_ = YAML::LoadFile(configPath_);
        try {
            parameters_ = configFile_.as<parameters_t>();
        } catch (const YAML::Exception &e) {
            std::cout << "YAML Error: " << e.what() << std::endl;
            return false;
        }

        if (parameters_.crouchHeight < Const::BODY_HEIGHT_MIN) {
            std::cout << "Crouch depth should be greater than min body height!"
                      << std::endl;
            return false;
        }
        if (parameters_.crouchTime <= 0.0f) {
            std::cout << "Crouch time must be positive!" << std::endl;
            return false;
        }
        if (parameters_.transitionTime <= 0.0f) {
            std::cout << "Transition time must be positive!" << std::endl;
            return false;
        }
        if (parameters_.pushTimeMin <= 0.0f ||
            parameters_.pushTimeMax < parameters_.pushTimeMin) {
            std::cout << "Push time bounds invalid!" << std::endl;
            return false;
        }
        if (parameters_.pushTimeScale <= 0.0f) {
            std::cout << "Push time scale must be positive!" << std::endl;
            return false;
        }

        return true;
    }

    bool JumpTrajectory::setTarget(float jumpLength, float jumpHeight) {
        if (jumpHeight <= 0.0f) {
            std::cout << "Jump height must be positive" << std::endl;
            return false;
        }
        if (std::fabs(jumpLength) <= kEps) {
            std::cout << "Jump length magnitude must be > 0" << std::endl;
            return false;
        }

        jumpHight_ = jumpHeight;
        jumpLength_ = jumpLength;
        return true;
    }

    bool JumpTrajectory::generate(const Body &currentTip) {
        if (std::fabs(jumpLength_) <= kEps || jumpHight_ <= kEps) {
            return false;
        }

        finished_ = false;
        trajectory_.progress = 0;
        target_ = TrajectoryPoint();
        trajectory_ = Trajectory(0);

        const float dir = (jumpLength_ >= 0.0f) ? 1.0f : -1.0f;
        const float L = std::fabs(jumpLength_);

        const float theta = std::atan2(4.0f * jumpHight_, L);
        const float V0 = std::sqrt(
            (Const::G * (L * L + 16.0f * jumpHight_ * jumpHight_)) / (8.0f * jumpHight_));
        const float T0 = (2.0f * V0 * std::sin(theta)) / Const::G;

        const float launchAngle = (dir > 0.0f) ? theta : (kPi - theta);

        const Trajectory crouch = generateCrouching(currentTip);
        const Trajectory push = generatePushing(V0, launchAngle);
        const Trajectory flying = generateFlying(T0);
        const Trajectory landing = generateLanding();

        trajectory_ = crouch + push + flying + landing;
        return !trajectory_.points.empty();
    }

    Trajectory JumpTrajectory::generateCrouching(const Body initTip) {
        const Eigen::Vector2f zero(0.0f, 0.0f);
        const Eigen::Vector2f startPos(initTip.pos[0], initTip.pos[2]);
        const Eigen::Vector2f startVel(initTip.linerVel[0], initTip.linerVel[2]);
        const Eigen::Vector2f finishPos(0.0f, parameters_.crouchHeight);

        const size_t steps =
            stepsFromDurationNs(static_cast<size_t>(parameters_.crouchTime * 1e9f));

        return QuinticPolynomial::generate(startPos,
                                           finishPos,
                                           startVel,
                                           zero,
                                           zero,
                                           zero,
                                           parameters_.crouchTime,
                                           steps);
    }

    Trajectory JumpTrajectory::generatePushing(const float launchVel,
                                               const float launchAngle) {
        const Eigen::Vector2f startPos(0.0f, parameters_.crouchHeight);
        const Eigen::Vector2f startVel(0.0f, 0.0f);

        const float vx = launchVel * std::cos(launchAngle);
        const float vz = -launchVel * std::sin(launchAngle);
        const Eigen::Vector2f finishVel(vx, vz);

        const float deltaZ = parameters_.takeoffHeight - parameters_.crouchHeight;
        const float pushTimeFromKinematics = 2.0f * deltaZ / vz;
        const float pushTimeFallback =
            parameters_.transitionTime * parameters_.pushTimeScale;

        float pushTime = pushTimeFromKinematics;
        if (!std::isfinite(pushTime) || pushTime <= 0.0f) {
            pushTime = pushTimeFallback;
        }
        pushTime = std::clamp(pushTime, parameters_.pushTimeMin, parameters_.pushTimeMax);

        const float takeoffX = 0.5f * vx * pushTime;
        const float takeoffZ = parameters_.takeoffHeight;

        pushTime_ = pushTime;
        takeoffX_ = takeoffX;
        takeoffZ_ = takeoffZ;
        launchVx_ = vx;
        launchVz_ = vz;

        const Eigen::Vector2f finishPos(takeoffX_, takeoffZ_);

        const size_t steps = stepsFromDurationNs(static_cast<size_t>(pushTime * 1e9f));
        const Eigen::Vector2f zero(0.0f, 0.0f);

        return QuinticPolynomial::generate(
            startPos, finishPos, startVel, finishVel, zero, zero, pushTime, steps);
    }

    Trajectory JumpTrajectory::generateFlying(const float flyTime) {
        if (flyTime <= kEps) {
            return Trajectory(0);
        }

        const float T1 = flyTime * 0.5f;
        const float T2 = flyTime - T1;

        const size_t totalSteps =
            stepsFromDurationNs(static_cast<size_t>(flyTime * 1e9f));
        size_t steps1 =
            static_cast<size_t>(std::lround(static_cast<float>(totalSteps) * 0.5f));
        steps1 = std::max<size_t>(1, steps1);
        size_t steps2 = std::max<size_t>(1, totalSteps - steps1);

        const Eigen::Vector2f zero(0.0f, 0.0f);

        const Eigen::Vector2f p0(takeoffX_, takeoffZ_);
        const Eigen::Vector2f p2(-takeoffX_, takeoffZ_);

        const float midX = 0.5f * (p0[0] + p2[0]);
        const float midZ =
            takeoffZ_ + std::fabs(jumpLength_) * parameters_.flyHeightRatio;
        const Eigen::Vector2f p1(midX, midZ);

        const Eigen::Vector2f v0(launchVx_, launchVz_);
        const Eigen::Vector2f v2(launchVx_, -launchVz_);

        const Eigen::Vector2f vm =
            QuinticPolynomial::solveMidVel2D(p0, p1, p2, v0, v2, T1, T2);

        const Eigen::Vector2f am =
            QuinticPolynomial::solveMidAcc2D(p0, v0, zero, p1, vm, p2, v2, zero, T1, T2);

        Trajectory seg1 =
            QuinticPolynomial::generate(p0, p1, v0, vm, zero, am, T1, steps1);
        Trajectory seg2 =
            QuinticPolynomial::generate(p1, p2, vm, v2, am, zero, T2, steps2);

        return seg1 + seg2;
    }

    Trajectory JumpTrajectory::generateLanding() {
        const Eigen::Vector2f zero(0.0f, 0.0f);

        const Eigen::Vector2f pTouch(-takeoffX_, takeoffZ_);
        const Eigen::Vector2f vTouch(launchVx_, -launchVz_);

        const float totalTime = std::max(kMinPhaseTime, parameters_.transitionTime);
        const float T1 = totalTime * 0.6f;
        const float T2 = std::max(kMinPhaseTime, totalTime - T1);

        const size_t totalSteps =
            stepsFromDurationNs(static_cast<size_t>(totalTime * 1e9f));
        size_t steps1 =
            static_cast<size_t>(std::lround(static_cast<float>(totalSteps) * 0.6f));
        steps1 = std::max<size_t>(1, steps1);
        size_t steps2 = std::max<size_t>(1, totalSteps - steps1);

        const float zCompress = Const::BODY_HEIGHT_MIN;
        const Eigen::Vector2f pCompress(pTouch[0], zCompress);
        const Eigen::Vector2f vCompress(0.0f, 0.0f);

        const Eigen::Vector2f pRecover(0.0f, parameters_.crouchHeight);
        const Eigen::Vector2f vRecover(0.0f, 0.0f);

        Trajectory absorb = QuinticPolynomial::generate(
            pTouch, pCompress, vTouch, vCompress, zero, zero, T1, steps1);
        Trajectory recover = QuinticPolynomial::generate(
            pCompress, pRecover, vCompress, vRecover, zero, zero, T2, steps2);

        return absorb + recover;
    }

    bool JumpTrajectory::start(const rclcpp::Time &startTime) {
        startTimeNanoSec_ = static_cast<size_t>(startTime.nanoseconds());
        trajectory_.progress = 0;

        if (trajectory_.points.empty()) {
            return false;
        }

        for (size_t i = 0; i < trajectory_.time.size(); ++i) {
            trajectory_.time[i] = startTimeNanoSec_ + i * durationNanoSec_;
        }

        target_ = trajectory_.points.front();
        target_.kp[0] = parameters_.transitionKp;
        target_.kp[1] = parameters_.transitionKp;
        target_.kd[0] = parameters_.transitionKd;
        target_.kd[1] = parameters_.transitionKd;
        return true;
    }

    TrajectoryPoint JumpTrajectory::get(const rclcpp::Time &currentTime) {
        if (finished_) {
            return target_;
        }

        if (trajectory_.points.empty()) {
            finished_ = true;
            return target_;
        }

        const size_t currentNanoSec = static_cast<size_t>(currentTime.nanoseconds());

        auto assignGains = [this]() {
            target_.kp[0] = parameters_.transitionKp;
            target_.kp[1] = parameters_.transitionKp;
            target_.kd[0] = parameters_.transitionKd;
            target_.kd[1] = parameters_.transitionKd;
        };

        if (currentNanoSec <= trajectory_.time.front()) {
            target_ = trajectory_.points.front();
            trajectory_.progress = 0;
            assignGains();
            return target_;
        }

        if (currentNanoSec >= trajectory_.time.back()) {
            target_ = trajectory_.points.back();
            trajectory_.progress = trajectory_.points.size();
            finished_ = true;
            assignGains();
            return target_;
        }

        interpolate(currentNanoSec);
        assignGains();
        return target_;
    }

} // namespace Dog8Dof

namespace YAML {
    template <> struct convert<Dog8Dof::JumpTrajectory::parameters_t> {
        static Node encode(const Dog8Dof::JumpTrajectory::parameters_t &rhs) {
            Node node;
            node["transitionKp"] = rhs.transitionKp;
            node["transitionKd"] = rhs.transitionKd;
            node["transitionTime"] = rhs.transitionTime;
            node["crouchHeight"] = rhs.crouchHeight;
            node["crouchTime"] = rhs.crouchTime;
            node["takeoffHeight"] = rhs.takeoffHeight;
            node["pushTimeMin"] = rhs.pushTimeMin;
            node["pushTimeMax"] = rhs.pushTimeMax;
            node["pushTimeScale"] = rhs.pushTimeScale;
            node["flyHeightRatio"] = rhs.flyHeightRatio;
            return node;
        }

        static bool decode(const Node &node, Dog8Dof::JumpTrajectory::parameters_t &rhs) {
            if (!node["transitionKp"] || !node["transitionKd"] ||
                !node["transitionTime"] || !node["crouchHeight"] || !node["crouchTime"] ||
                !node["takeoffHeight"] || !node["pushTimeMin"] || !node["pushTimeMax"] ||
                !node["pushTimeScale"] || !node["flyHeightRatio"]) {
                return false;
            }

            rhs.transitionKp = node["transitionKp"].as<float>();
            rhs.transitionKd = node["transitionKd"].as<float>();
            rhs.transitionTime = node["transitionTime"].as<float>();
            rhs.crouchHeight = node["crouchHeight"].as<float>();
            rhs.crouchTime = node["crouchTime"].as<float>();
            rhs.takeoffHeight = node["takeoffHeight"].as<float>();
            rhs.pushTimeMin = node["pushTimeMin"].as<float>();
            rhs.pushTimeMax = node["pushTimeMax"].as<float>();
            rhs.pushTimeScale = node["pushTimeScale"].as<float>();
            rhs.flyHeightRatio = node["flyHeightRatio"].as<float>();
            return true;
        }
    };
} // namespace YAML