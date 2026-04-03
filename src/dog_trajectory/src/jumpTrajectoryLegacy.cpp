#include "dog_trajectory/jumpTrajectoryLegacy.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iostream>

namespace Dog8Dof {
    constexpr float kEps = 1e-6f;

    JumpTrajectoryLegacy::JumpTrajectoryLegacy(const std::string &configPath,
                                               rclcpp::Duration period)
        : BaseTrajectory(configPath, period) {
        if (!reloadParameters()) {
            throw std::invalid_argument("Could not load jump config YAML");
        }
    }

    float JumpTrajectoryLegacy::clamp01(float x) {
        return std::clamp(x, 0.0f, 1.0f);
    }

    float JumpTrajectoryLegacy::smoothInterp(float s) {
        return 0.5f * (1.0f - std::cos(M_PI * s));
    }

    float JumpTrajectoryLegacy::smoothInterpDeriv(float s, float duration) {
        return 0.5f * std::sin(M_PI * s) * (M_PI / duration);
    }

    float JumpTrajectoryLegacy::smoothInterpDeriv2(float s, float duration) {
        return 0.5f * std::cos(M_PI * s) * (M_PI / duration) * (M_PI / duration);
    }

    float JumpTrajectoryLegacy::explosiveInterp(float s, float exponent) {
        return std::pow(s, exponent);
    }

    float
    JumpTrajectoryLegacy::explosiveInterpDeriv(float s, float exponent, float duration) {
        if (s <= 0.0f) {
            return 0.0f;
        }
        return exponent * std::pow(s, exponent - 1.0f) / duration;
    }

    float
    JumpTrajectoryLegacy::explosiveInterpDeriv2(float s, float exponent, float duration) {
        if (s <= 0.0f || exponent < 2.0f) {
            return 0.0f;
        }
        return exponent * (exponent - 1.0f) * std::pow(s, exponent - 2.0f) /
               (duration * duration);
    }

    void JumpTrajectoryLegacy::fillPoint(
        TrajectoryPoint &pt, float x, float z, float vx, float vz, float kp, float kd) {
        pt.pos[0] = x;
        pt.pos[1] = 0.0f;
        pt.pos[2] = z;

        pt.vel[0] = vx;
        pt.vel[1] = 0.0f;
        pt.vel[2] = vz;

        pt.tau.setZero();
        pt.kp[0] = kp;
        pt.kp[1] = kp;
        pt.kd[0] = kd;
        pt.kd[1] = kd;
    }

    bool JumpTrajectoryLegacy::reloadParameters() {
        configFile_ = YAML::LoadFile(configPath_);
        try {
            for (YAML::const_iterator it = configFile_.begin(); it != configFile_.end();
                 ++it) {
                std::string subTreeName = it->first.as<std::string>();
                YAML::Node subTree = it->second;
                parameters_t parameter = subTree.as<parameters_t>();
                configMapping_[subTreeName] = parameter;
            }
            // 加载多树的配置文件
        } catch (const YAML::Exception &e) {
            std::cout << "YAML Error: " << e.what() << std::endl;
            return false;
        }

        for (std::map<std::string, parameters_t>::iterator it = configMapping_.begin();
             it != configMapping_.end();
             ++it) {
            std::string currentConfigName = it->first;
            parameters_t currentParameter = it->second;

            if (currentParameter.baseHeight < Const::LEG_LENGTH_MIN ||
                currentParameter.baseHeight > Const::LEG_LENGTH_MAX) {
                std::cout << "In config " << currentConfigName
                          << " baseHeight out of range" << std::endl;
                return false;
            }

            if (currentParameter.crouchDepth < 0.0f ||
                currentParameter.pushStroke < 0.0f) {
                std::cout << "In config " << currentConfigName
                          << " crouchDepth/pushStroke must be non-negative" << std::endl;
                return false;
            }

            if (currentParameter.crouchDuration <= 0.0f ||
                currentParameter.pushDuration <= 0.0f ||
                currentParameter.flightDuration <= 0.0f ||
                currentParameter.landDuration <= 0.0f) {
                std::cout << "In config " << currentConfigName
                          << " phase duration must be positive" << std::endl;
                return false;
            }

            if (currentParameter.pushVelocityGain <= 0.0f) {
                std::cout << "In config " << currentConfigName
                          << " pushVelocityGain must be positive" << std::endl;
                return false;
            }

            if (currentParameter.pushXMode != "smooth" &&
                currentParameter.pushXMode != "explosive") {
                std::cout << "In config " << currentConfigName
                          << "pushXMode must be smooth or explosive" << std::endl;
                return false;
            }

            std::cout << "Successfully loaded jump config of " << currentConfigName
                      << std::endl;
        }

        return true;
    }

    bool JumpTrajectoryLegacy::setTarget(std::string configName) {
        try {
            parameters_ = configMapping_.at(configName);
        } catch (const std::out_of_range &e) {
            std::cout << e.what() << std::endl;
            return false;
        }
        return true;
    }

    void JumpTrajectoryLegacy::samplePhase(Phase phase,
                                           float timeInPhase,
                                           float &targetX,
                                           float &targetZ,
                                           float &targetVx,
                                           float &targetVz) const {
        const float crouchZ = parameters_.baseHeight - parameters_.crouchDepth;
        const float takeoffZ = crouchZ + parameters_.pushStroke;
        const float takeoffX = -parameters_.pushBackwardDist;

        targetX = 0.0f;
        targetZ = parameters_.baseHeight;
        targetVx = 0.0f;
        targetVz = 0.0f;

        switch (phase) {
        case Phase::Crouching: {
            const float T = parameters_.crouchDuration;
            const float s = clamp01(timeInPhase / T);
            const float p = smoothInterp(s);
            const float pd = smoothInterpDeriv(s, T);

            targetZ = initialFootZ_ + (crouchZ - initialFootZ_) * p;
            targetX = parameters_.crouchForwardDist * p;

            targetVz = (crouchZ - initialFootZ_) * pd;
            targetVx = parameters_.crouchForwardDist * pd;
            break;
        }
        case Phase::PushingOff: {
            const float T = parameters_.pushDuration;
            const float s = clamp01(timeInPhase / T);

            const float pz = explosiveInterp(s, parameters_.pushExponentZ);
            const float pzd = explosiveInterpDeriv(s, parameters_.pushExponentZ, T);
            targetZ = crouchZ + (takeoffZ - crouchZ) * pz;
            targetVz = (takeoffZ - crouchZ) * pzd;

            float px = 0.0f;
            float pxd = 0.0f;
            if (parameters_.pushXMode == "explosive") {
                px = explosiveInterp(s, parameters_.pushExponentX);
                pxd = explosiveInterpDeriv(s, parameters_.pushExponentX, T);
            } else {
                px = smoothInterp(s);
                pxd = smoothInterpDeriv(s, T);
            }

            const float xSpan =
                parameters_.crouchForwardDist + parameters_.pushBackwardDist;
            targetX = parameters_.crouchForwardDist - xSpan * px;
            targetVx = -xSpan * pxd;

            targetVx *= parameters_.pushVelocityGain;
            targetVz *= parameters_.pushVelocityGain;
            break;
        }
        case Phase::InFlight: {
            const float T = parameters_.flightDuration;
            const float s = clamp01(timeInPhase / T);
            const float p = smoothInterp(s);
            const float pd = smoothInterpDeriv(s, T);

            const float endZ = parameters_.baseHeight - parameters_.flightApexZOffset;
            const float endX = parameters_.landingPrepX;

            targetZ = takeoffZ + (endZ - takeoffZ) * p;
            targetX = takeoffX + (endX - takeoffX) * p;

            targetVz = (endZ - takeoffZ) * pd;
            targetVx = (endX - takeoffX) * pd;
            break;
        }
        case Phase::Landing: {
            const float T = parameters_.landDuration;
            const float s = clamp01(timeInPhase / T);
            const float p = smoothInterp(s);
            const float pd = smoothInterpDeriv(s, T);

            const float extendedZ =
                parameters_.baseHeight - parameters_.flightApexZOffset;
            const float extendedX = parameters_.landingPrepX;

            targetZ = extendedZ + (parameters_.baseHeight - extendedZ) * p;
            targetX = extendedX + (0.0f - extendedX) * p;

            targetVz = (parameters_.baseHeight - extendedZ) * pd;
            targetVx = (0.0f - extendedX) * pd;
            break;
        }
        }

        targetZ = std::clamp(targetZ, Const::LEG_LENGTH_MIN, Const::LEG_LENGTH_MAX);
    }

    void JumpTrajectoryLegacy::appendPhase(Phase phase,
                                           float duration,
                                           Trajectory &traj) const {
        const size_t steps = stepsFromDurationNs(static_cast<size_t>(duration * 1e9f));
        if (steps == 0) {
            return;
        }

        float kp = parameters_.transitionKp;
        float kd = parameters_.transitionKd;
        switch (phase) {
        case Phase::Crouching:
            kp = parameters_.phaseKpCrouch;
            kd = parameters_.phaseKdCrouch;
            break;
        case Phase::PushingOff:
            kp = parameters_.phaseKpPush;
            kd = parameters_.phaseKdPush;
            break;
        case Phase::InFlight:
            kp = parameters_.phaseKpFlight;
            kd = parameters_.phaseKdFlight;
            break;
        case Phase::Landing:
            kp = parameters_.phaseKpLand;
            kd = parameters_.phaseKdLand;
            break;
        }

        for (size_t i = 0; i < steps; ++i) {
            const float s = (steps == 1)
                                ? 0.0f
                                : static_cast<float>(i) / static_cast<float>(steps - 1);
            const float t = s * duration;

            float x = 0.0f;
            float z = 0.0f;
            float vx = 0.0f;
            float vz = 0.0f;
            samplePhase(phase, t, x, z, vx, vz);

            TrajectoryPoint pt;
            fillPoint(pt, x, z, vx, vz, kp, kd);
            traj.points.push_back(pt);
            traj.time.push_back(0);
        }
    }

    bool JumpTrajectoryLegacy::generate(const Body &currentTip) {
        // if (std::fabs(targetLaunchSpeed_) <= kEps && targetFlyTime_ <= kEps) {
        //     return false;
        // }

        finished_ = false;
        trajectory_.progress = 0;
        target_ = TrajectoryPoint();
        trajectory_ = Trajectory(0);

        initialFootZ_ =
            std::clamp(currentTip.pos[2], Const::LEG_LENGTH_MIN, Const::LEG_LENGTH_MAX);

        // parameters_.flightDuration =
        //     (targetFlyTime_ > kEps) ? targetFlyTime_ : parameters_.flightDuration;

        // const float speedScale =
        //     std::clamp(std::fabs(targetLaunchSpeed_) / 5000.0f, 0.5f, 2.0f);
        // const float oldPushGain = parameters_.pushVelocityGain;
        // const_cast<parameters_t &>(parameters_).pushVelocityGain =
        //     oldPushGain * speedScale;

        Trajectory traj(0);
        appendPhase(Phase::Crouching, parameters_.crouchDuration, traj);
        appendPhase(Phase::PushingOff, parameters_.pushDuration, traj);
        appendPhase(Phase::InFlight, parameters_.flightDuration, traj);
        appendPhase(Phase::Landing, parameters_.landDuration, traj);

        // const_cast<parameters_t &>(parameters_).pushVelocityGain = oldPushGain;

        trajectory_ = std::move(traj);
        return !trajectory_.points.empty();
    }

    bool JumpTrajectoryLegacy::start(const rclcpp::Time &startTime) {
        startTimeNanoSec_ = static_cast<size_t>(startTime.nanoseconds());
        trajectory_.progress = 0;

        if (trajectory_.points.empty()) {
            return false;
        }

        for (size_t i = 0; i < trajectory_.time.size(); ++i) {
            trajectory_.time[i] = startTimeNanoSec_ + i * durationNanoSec_;
        }

        target_ = trajectory_.points.front();
        return true;
    }

    TrajectoryPoint JumpTrajectoryLegacy::get(const rclcpp::Time &currentTime) {
        if (finished_) {
            return target_;
        }

        if (trajectory_.points.empty()) {
            finished_ = true;
            return target_;
        }

        const size_t currentNanoSec = static_cast<size_t>(currentTime.nanoseconds());

        if (currentNanoSec <= trajectory_.time.front()) {
            target_ = trajectory_.points.front();
            trajectory_.progress = 0;
            return target_;
        }

        if (currentNanoSec >= trajectory_.time.back()) {
            target_ = trajectory_.points.back();
            trajectory_.progress = trajectory_.points.size();
            finished_ = true;
            return target_;
        }

        interpolate(currentNanoSec);

        const size_t idx = std::min(trajectory_.progress, trajectory_.points.size() - 1);
        target_.kp = trajectory_.points[idx].kp;
        target_.kd = trajectory_.points[idx].kd;
        return target_;
    }

} // namespace Dog8Dof

namespace YAML {
    template <> struct convert<Dog8Dof::JumpTrajectoryLegacy::parameters_t> {
        static Node encode(const Dog8Dof::JumpTrajectoryLegacy::parameters_t &rhs) {
            Node node;
            node["transitionKp"] = rhs.transitionKp;
            node["transitionKd"] = rhs.transitionKd;
            node["transitionTime"] = rhs.transitionTime;

            node["baseHeight"] = rhs.baseHeight;
            node["crouchDepth"] = rhs.crouchDepth;
            node["pushStroke"] = rhs.pushStroke;

            node["crouchDuration"] = rhs.crouchDuration;
            node["pushDuration"] = rhs.pushDuration;
            node["flightDuration"] = rhs.flightDuration;
            node["landDuration"] = rhs.landDuration;

            node["flightApexZOffset"] = rhs.flightApexZOffset;
            node["landingPrepX"] = rhs.landingPrepX;

            node["pushExponentZ"] = rhs.pushExponentZ;
            node["pushExponentX"] = rhs.pushExponentX;
            node["pushXMode"] = rhs.pushXMode;

            node["crouchForwardDist"] = rhs.crouchForwardDist;
            node["pushBackwardDist"] = rhs.pushBackwardDist;
            node["pushVelocityGain"] = rhs.pushVelocityGain;

            node["phaseKpCrouch"] = rhs.phaseKpCrouch;
            node["phaseKdCrouch"] = rhs.phaseKdCrouch;
            node["phaseKpPush"] = rhs.phaseKpPush;
            node["phaseKdPush"] = rhs.phaseKdPush;
            node["phaseKpFlight"] = rhs.phaseKpFlight;
            node["phaseKdFlight"] = rhs.phaseKdFlight;
            node["phaseKpLand"] = rhs.phaseKpLand;
            node["phaseKdLand"] = rhs.phaseKdLand;
            return node;
        }

        static bool decode(const Node &node,
                           Dog8Dof::JumpTrajectoryLegacy::parameters_t &rhs) {
            auto getFloat = [&](std::initializer_list<const char *> keys,
                                float fallback) {
                for (const char *k : keys) {
                    if (node[k]) {
                        return node[k].as<float>(fallback);
                    }
                }
                return fallback;
            };

            auto getString = [&](std::initializer_list<const char *> keys,
                                 const std::string &fallback) {
                for (const char *k : keys) {
                    if (node[k]) {
                        return node[k].as<std::string>(fallback);
                    }
                }
                return fallback;
            };

            rhs.transitionKp = getFloat({"transitionKp"}, 1.0f);
            rhs.transitionKd = getFloat({"transitionKd"}, 0.05f);
            rhs.transitionTime = getFloat({"transitionTime"}, 0.4f);

            rhs.baseHeight = getFloat({"baseHeight", "base_height"}, 180.0f);
            rhs.crouchDepth = getFloat({"crouchDepth", "crouch_depth"}, 50.0f);
            rhs.pushStroke = getFloat({"pushStroke", "push_stroke"}, 60.0f);

            rhs.crouchDuration = getFloat({"crouchDuration", "crouch_duration"}, 0.30f);
            rhs.pushDuration = getFloat({"pushDuration", "push_duration"}, 0.10f);
            rhs.flightDuration = getFloat({"flightDuration", "flight_duration"}, 0.30f);
            rhs.landDuration = getFloat({"landDuration", "land_duration"}, 0.25f);

            rhs.flightApexZOffset =
                getFloat({"flightApexZOffset", "flight_apex_z_offset"}, 80.0f);
            rhs.landingPrepX = getFloat({"landingPrepX", "landing_prep_x"}, 30.0f);

            rhs.pushExponentZ = getFloat({"pushExponentZ", "push_exponent_z"}, 3.0f);
            rhs.pushExponentX = getFloat({"pushExponentX", "push_exponent_x"}, 2.0f);
            rhs.pushXMode = getString({"pushXMode", "push_x_mode"}, "smooth");

            rhs.crouchForwardDist =
                getFloat({"crouchForwardDist", "crouch_forward_dist"}, 0.0f);
            rhs.pushBackwardDist =
                getFloat({"pushBackwardDist", "push_backward_dist"}, 0.0f);
            rhs.pushVelocityGain =
                getFloat({"pushVelocityGain", "push_velocity_gain"}, 1.0f);

            rhs.phaseKpCrouch = getFloat({"phaseKpCrouch", "kp_crouch"}, 0.6f);
            rhs.phaseKdCrouch = getFloat({"phaseKdCrouch", "kd_crouch"}, 0.01f);
            rhs.phaseKpPush = getFloat({"phaseKpPush", "kp_push"}, 1.2f);
            rhs.phaseKdPush = getFloat({"phaseKdPush", "kd_push"}, 0.01f);
            rhs.phaseKpFlight = getFloat({"phaseKpFlight", "kp_flight"}, 0.3f);
            rhs.phaseKdFlight = getFloat({"phaseKdFlight", "kd_flight"}, 0.01f);
            rhs.phaseKpLand = getFloat({"phaseKpLand", "kp_land"}, 0.3f);
            rhs.phaseKdLand = getFloat({"phaseKdLand", "kd_land"}, 0.05f);

            return true;
        }
    };
} // namespace YAML