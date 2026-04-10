#include "dog_trajectory/jumpTrajectory.hpp"
#include "dog_tools/quinticPolynomial.hpp"

#include <algorithm>
#include <cmath>

namespace Dog8Dof {
    namespace {
        constexpr float kEps = 1e-6f;
        constexpr float kMinPhaseTime = 0.05f;

        struct SlipState {
            float x = 0.0f;
            float z = 0.0f;
            float vx = 0.0f;
            float vz = 0.0f;
        };

        float deg2rad(float deg) {
            return deg * static_cast<float>(M_PI / 180.0);
        }

        TrajectoryPoint makeTipPoint(float footX, float footZ, const SlipState &com) {
            TrajectoryPoint p;
            p.pos[0] = footX - com.x;
            p.pos[1] = 0.0f;
            p.pos[2] = footZ - com.z;

            p.vel[0] = -com.vx;
            p.vel[1] = 0.0f;
            p.vel[2] = -com.vz;
            p.tau.setZero();
            return p;
        }

        void stepSlip(SlipState &state,
                      float dt,
                      float footX,
                      float footZ,
                      float mass,
                      float restLength,
                      float stiffness,
                      float damping,
                      float targetVx,
                      float targetVz,
                      float vxTrackGain,
                      float vzTrackGain) {
            const float rx = state.x - footX;
            const float rz = state.z - footZ;
            const float legLenSq = rx * rx + rz * rz;
            const float legLen = std::sqrt(std::max(legLenSq, kEps));
            const float legDot = (rx * state.vx + rz * state.vz) / legLen;

            float springForce = stiffness * (restLength - legLen) - damping * legDot;
            if (springForce < 0.0f) {
                springForce = 0.0f;
            }

            const float invLegLen = 1.0f / legLen;
            const float accSpringX = (springForce / mass) * (rx * invLegLen);
            const float accSpringZ = (springForce / mass) * (rz * invLegLen);

            const float ax = accSpringX + vxTrackGain * (targetVx - state.vx);
            const float az = accSpringZ + Const::G + vzTrackGain * (targetVz - state.vz);

            state.vx += ax * dt;
            state.vz += az * dt;
            state.x += state.vx * dt;
            state.z += state.vz * dt;
        }

        float speedNorm(const SlipState &state) {
            return std::sqrt(state.vx * state.vx + state.vz * state.vz);
        }
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
            RCLCPP_ERROR(logger_, "YAML Error: %s", e.what());
            return false;
        }

        if (parameters_.crouchHeight < Const::BODY_HEIGHT_MIN) {
            RCLCPP_ERROR(logger_, "Crouch depth should be greater than min body height!");
            return false;
        }
        if (parameters_.crouchTime <= 0.0f) {
            RCLCPP_ERROR(logger_, "Crouch time must be positive!");
            return false;
        }
        if (parameters_.transitionTime <= 0.0f) {
            RCLCPP_ERROR(logger_, "Transition time must be positive!");
            return false;
        }
        if (parameters_.pushTimeMin <= 0.0f ||
            parameters_.pushTimeMax < parameters_.pushTimeMin) {
            RCLCPP_ERROR(logger_, "Push time bounds invalid!");
            return false;
        }
        if (parameters_.pushTimeScale <= 0.0f) {
            RCLCPP_ERROR(logger_, "Push time scale must be positive!");
            return false;
        }
        if (parameters_.takeoffHeight >= parameters_.crouchHeight) {
            RCLCPP_ERROR(logger_, "Takeoff height must be less than crouch height");
            return false;
        }
        if (parameters_.slipMass <= 0.0f || parameters_.slipRestLength <= 0.0f ||
            parameters_.slipStiffness <= 0.0f || parameters_.slipDamping < 0.0f ||
            parameters_.slipStanceMaxTime <= 0.0f || parameters_.slipStopSpeed <= 0.0f) {
            RCLCPP_ERROR(logger_, "SLIP parameters invalid");
            return false;
        }
        if (parameters_.slipTouchdownAngleDeg <= 0.0f ||
            parameters_.slipTouchdownAngleDeg >= 80.0f) {
            RCLCPP_ERROR(logger_, "SLIP touchdown angle should be in (0, 80) deg");
            return false;
        }

        return true;
    }

    bool JumpTrajectory::setTarget(float launchSpeed, float flyTime) {
        if (std::fabs(launchSpeed) <= kEps) {
            RCLCPP_ERROR(logger_, "Launch speed magnitude must be positive");
            return false;
        }
        if (flyTime <= 0.0f) {
            RCLCPP_ERROR(logger_, "Fly time must be positive");
            return false;
        }
        targetLaunchSpeed_ = launchSpeed;
        targetFlyTime_ = flyTime;
        return true;
    }

    bool JumpTrajectory::generate(const Body &currentTip) {
        if (std::fabs(targetLaunchSpeed_) <= kEps || targetFlyTime_ <= kEps) {
            return false;
        }

        finished_ = false;
        trajectory_.progress = 0;
        target_ = TrajectoryPoint();
        trajectory_ = Trajectory(0);

        const float T = targetFlyTime_;
        const float speedAbs = std::fabs(targetLaunchSpeed_);
        const float dir = (targetLaunchSpeed_ >= 0.0f) ? 1.0f : -1.0f;

        // 同高度起落时，竖直分量由飞行时间唯一确定
        const float vz0 = -0.5f * Const::G * T; // +Z向下，向上起跳为负
        const float vzNeed = std::fabs(vz0);
        if (speedAbs < vzNeed) {
            RCLCPP_ERROR(logger_, "Launch speed too small for requested fly time");
            return false;
        }

        const float vx0 =
            dir * std::sqrt(std::max(0.0f, speedAbs * speedAbs - vz0 * vz0));

        takeoffVx_ = vx0;
        takeoffVz_ = vz0;
        jumpLength_ = vx0 * T;

        const Trajectory crouch = generateCrouching(currentTip);
        const Trajectory takeoff = generateSlipTakeoff(vx0, vz0);
        const Trajectory flying = generateSlipFlight(T);
        const Trajectory landing = generateSlipLanding();

        trajectory_ = crouch + takeoff + flying + landing;
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

    Trajectory JumpTrajectory::generateSlipTakeoff(float targetVx, float targetVz) {
        const float dt = static_cast<float>(durationNanoSec_) * 1e-9f;
        const float simTime =
            std::clamp(parameters_.slipStanceMaxTime,
                       parameters_.pushTimeMin,
                       std::max(parameters_.pushTimeMax, parameters_.slipStanceMaxTime));

        const size_t maxSteps = stepsFromDurationNs(static_cast<size_t>(simTime * 1e9f));
        const size_t minSteps =
            stepsFromDurationNs(static_cast<size_t>(parameters_.pushTimeMin * 1e9f));

        Trajectory phase(0);
        phase.points.reserve(maxSteps);
        phase.time.reserve(maxSteps);

        takeoffFootX_ = 0.0f;
        takeoffFootZ_ = 0.0f;

        SlipState com;
        com.x = 0.0f;
        com.z = -parameters_.crouchHeight;
        com.vx = 0.0f;
        com.vz = 0.0f;

        const float vxGain = 8.0f;
        const float vzGain = 5.0f;

        for (size_t i = 0; i < maxSteps; ++i) {
            phase.points.push_back(makeTipPoint(takeoffFootX_, takeoffFootZ_, com));
            phase.time.push_back(0);

            stepSlip(com,
                     dt,
                     takeoffFootX_,
                     takeoffFootZ_,
                     parameters_.slipMass,
                     parameters_.slipRestLength,
                     parameters_.slipStiffness,
                     parameters_.slipDamping,
                     targetVx,
                     targetVz,
                     vxGain,
                     vzGain);

            const float rx = com.x - takeoffFootX_;
            const float rz = com.z - takeoffFootZ_;
            const float legLen = std::sqrt(std::max(rx * rx + rz * rz, kEps));
            const float legDot = (rx * com.vx + rz * com.vz) / legLen;

            if (i + 1 >= std::max<size_t>(2, minSteps) &&
                legLen >= parameters_.slipRestLength && legDot > 0.0f && com.vz < -kEps) {
                break;
            }
        }

        if (phase.points.empty()) {
            return Trajectory(0);
        }

        const TrajectoryPoint &last = phase.points.back();
        takeoffX_ = last.pos[0];
        takeoffZ_ = last.pos[2];
        takeoffVx_ = last.vel[0];
        takeoffVz_ = last.vel[2];
        pushTime_ = static_cast<float>(phase.points.size()) * dt;
        return phase;
    }

    Trajectory JumpTrajectory::generateSlipFlight(float flyTime) {
        if (flyTime <= kEps) {
            return Trajectory(0);
        }

        const size_t steps = stepsFromDurationNs(static_cast<size_t>(flyTime * 1e9f));
        Trajectory phase(0);
        phase.points.reserve(steps);
        phase.time.reserve(steps);

        SlipState com0;
        com0.x = takeoffFootX_ - takeoffX_;
        com0.z = takeoffFootZ_ - takeoffZ_;
        com0.vx = -takeoffVx_;
        com0.vz = -takeoffVz_;

        const SlipState comEnd = {
            com0.x + com0.vx * flyTime,
            com0.z + com0.vz * flyTime + 0.5f * Const::G * flyTime * flyTime,
            com0.vx,
            com0.vz + Const::G * flyTime};

        const float dir = (targetLaunchSpeed_ >= 0.0f) ? 1.0f : -1.0f;
        const float theta = deg2rad(parameters_.slipTouchdownAngleDeg);
        landingFootX_ = comEnd.x - dir * parameters_.slipRestLength * std::sin(theta);
        landingFootZ_ = comEnd.z + parameters_.slipRestLength * std::cos(theta);

        for (size_t i = 0; i < steps; ++i) {
            const float s = (steps == 1)
                                ? 0.0f
                                : static_cast<float>(i) / static_cast<float>(steps - 1);
            const float t = s * flyTime;

            SlipState com;
            com.x = com0.x + com0.vx * t;
            com.z = com0.z + com0.vz * t + 0.5f * Const::G * t * t;
            com.vx = com0.vx;
            com.vz = com0.vz + Const::G * t;

            const float footX = takeoffFootX_ + (landingFootX_ - takeoffFootX_) * s;
            const float footZ = takeoffFootZ_ + (landingFootZ_ - takeoffFootZ_) * s;

            phase.points.push_back(makeTipPoint(footX, footZ, com));
            phase.time.push_back(0);
        }

        touchdownX_ = landingFootX_ - comEnd.x;
        touchdownZ_ = landingFootZ_ - comEnd.z;
        touchdownVx_ = -comEnd.vx;
        touchdownVz_ = -comEnd.vz;
        jumpLength_ = comEnd.x - com0.x;

        return phase;
    }

    Trajectory JumpTrajectory::generateSlipLanding() {
        const float dt = static_cast<float>(durationNanoSec_) * 1e-9f;
        const size_t maxSteps = stepsFromDurationNs(
            static_cast<size_t>(parameters_.slipStanceMaxTime * 1e9f));

        Trajectory absorb(0);
        absorb.points.reserve(maxSteps);
        absorb.time.reserve(maxSteps);

        SlipState com;
        com.x = landingFootX_ - touchdownX_;
        com.z = landingFootZ_ - touchdownZ_;
        com.vx = -touchdownVx_;
        com.vz = -touchdownVz_;

        const size_t minSteps =
            stepsFromDurationNs(static_cast<size_t>(kMinPhaseTime * 1e9f));
        const float vxGain = 6.0f;
        const float vzGain = 4.0f;

        for (size_t i = 0; i < maxSteps; ++i) {
            absorb.points.push_back(makeTipPoint(landingFootX_, landingFootZ_, com));
            absorb.time.push_back(0);

            stepSlip(com,
                     dt,
                     landingFootX_,
                     landingFootZ_,
                     parameters_.slipMass,
                     parameters_.slipRestLength,
                     parameters_.slipStiffness,
                     parameters_.slipDamping,
                     0.0f,
                     0.0f,
                     vxGain,
                     vzGain);

            const float rx = com.x - landingFootX_;
            const float rz = com.z - landingFootZ_;
            const float legLen = std::sqrt(std::max(rx * rx + rz * rz, kEps));
            const float legDot = (rx * com.vx + rz * com.vz) / legLen;

            // 原有速度停止条件
            const bool slowEnough = (speedNorm(com) <= parameters_.slipStopSpeed);

            // 新增：腿已回到/超过静长且在继续伸长，说明已失去有效压缩接触
            const bool lostCompression =
                (legLen >= parameters_.slipRestLength && legDot > 0.0f);

            // 新增：数值保护，防止继续深度下探
            const bool overDrop =
                (com.z > landingFootZ_ + parameters_.slipRestLength * 1.2f);

            if (i + 1 >= std::max<size_t>(2, minSteps) &&
                (slowEnough || lostCompression || overDrop)) {
                break;
            }
        }

        if (absorb.points.empty()) {
            return Trajectory(0);
        }

        const TrajectoryPoint &tail = absorb.points.back();
        const Eigen::Vector2f startPos(tail.pos[0], tail.pos[2]);
        const Eigen::Vector2f startVel(tail.vel[0], tail.vel[2]);
        const Eigen::Vector2f finishPos(0.0f, parameters_.crouchHeight);
        const Eigen::Vector2f zero(0.0f, 0.0f);

        const float recoverTime = std::max(kMinPhaseTime, parameters_.transitionTime);
        const size_t recoverSteps =
            stepsFromDurationNs(static_cast<size_t>(recoverTime * 1e9f));
        Trajectory recover = QuinticPolynomial::generate(
            startPos, finishPos, startVel, zero, zero, zero, recoverTime, recoverSteps);

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
            node["slipMass"] = rhs.slipMass;
            node["slipRestLength"] = rhs.slipRestLength;
            node["slipStiffness"] = rhs.slipStiffness;
            node["slipDamping"] = rhs.slipDamping;
            node["slipTouchdownAngleDeg"] = rhs.slipTouchdownAngleDeg;
            node["slipStanceMaxTime"] = rhs.slipStanceMaxTime;
            node["slipStopSpeed"] = rhs.slipStopSpeed;
            return node;
        }

        static bool decode(const Node &node, Dog8Dof::JumpTrajectory::parameters_t &rhs) {
            if (!node["transitionKp"] || !node["transitionKd"] ||
                !node["transitionTime"] || !node["crouchHeight"] || !node["crouchTime"] ||
                !node["takeoffHeight"] || !node["pushTimeMin"] || !node["pushTimeMax"] ||
                !node["pushTimeScale"] || !node["flyHeightRatio"] || !node["slipMass"] ||
                !node["slipRestLength"] || !node["slipStiffness"] ||
                !node["slipDamping"] || !node["slipTouchdownAngleDeg"] ||
                !node["slipStanceMaxTime"] || !node["slipStopSpeed"]) {
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
            rhs.slipMass = node["slipMass"].as<float>();
            rhs.slipRestLength = node["slipRestLength"].as<float>();
            rhs.slipStiffness = node["slipStiffness"].as<float>();
            rhs.slipDamping = node["slipDamping"].as<float>();
            rhs.slipTouchdownAngleDeg = node["slipTouchdownAngleDeg"].as<float>();
            rhs.slipStanceMaxTime = node["slipStanceMaxTime"].as<float>();
            rhs.slipStopSpeed = node["slipStopSpeed"].as<float>();
            return true;
        }
    };
} // namespace YAML