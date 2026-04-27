#include "dog_trajectory/standTrajectory.hpp"
#include "dog_tools/quinticPolynomial.hpp"

namespace Dog8Dof {
    StandTrajectory::StandTrajectory(const std::string &configPath,
                                     rclcpp::Duration period)
        : BaseTrajectory(configPath, period) {
        if (!reloadParameters()) {
            throw std::invalid_argument("Could not load Config YAML!");
        }
        // 加载参数
    }

    bool StandTrajectory::reloadParameters() {
        configFile_ = YAML::LoadFile(configPath_);
        // 重新加载参数文件
        try {
            parameters_ = configFile_.as<parameters_t>();
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(
                rclcpp::get_logger("stand_trajectory"), "YAML Error: %s", e.what());
            return false;
        }
        // xFilter_.set_Alpha(parameters_.filterAlpha);
        // zFilter_.set_Alpha(parameters_.filterAlpha);

        return true;
    }

    bool StandTrajectory::setTarget(float tipX, float tipZ) {
        standTipPos_[0] = tipX;
        standTipPos_[1] = tipZ;
        return true;
    }

    bool StandTrajectory::generate(const Body &currentTip) {
        finished_ = false;
        trajectory_.progress = 0;
        target_ = TrajectoryPoint();
        const size_t totalSteps =
            stepsFromDurationNs(static_cast<size_t>(parameters_.transitionTime * 1e9));

        trajectory_ = Trajectory(totalSteps);

        const Eigen::Vector2f startPos(currentTip.pos[0], currentTip.pos[2]);
        const Eigen::Vector2f startVel(currentTip.linerVel[0], currentTip.linerVel[2]);

        const Eigen::Vector2f finishPos(standTipPos_[0], standTipPos_[1]);
        const Eigen::Vector2f finishVel(0, 0);

        const Eigen::Vector2f tangentVec = finishPos - startPos;
        const float stepHeight = abs(tangentVec[0] * parameters_.stepHeightFactor);
        // 根据步长计算步高

        if (tangentVec.squaredNorm() <= parameters_.reachThresholdSQ) {
            target_.kp[0] = parameters_.standKp;
            target_.kp[1] = parameters_.standKp;
            target_.kd[0] = parameters_.standKd;
            target_.kd[1] = parameters_.standKd;
            target_.pos = currentTip.pos;
            target_.vel.setZero();
            target_.tau[0] = 0;
            target_.tau[1] = 0;
            target_.tau[2] = (Const::BODY_MASS * Const::G) / 4;
            finished_ = true;
            return true;
        } // 判断无需进行路径规划直接保持位置

        const Eigen::Vector2f directionVec = getHeightVec(tangentVec);

        Eigen::Vector2f midPos = (finishPos + startPos) / 2 + directionVec * stepHeight;
        if (midPos[1] <= Const::BODY_HEIGHT_MIN) {
            midPos[1] = Const::BODY_HEIGHT_MIN;
        } // 防止足尖穿过机体

        const float durationSec = controlDurationNanoSec_ * 1e-9;
        const float T1 = durationSec * 0.5 * totalSteps;
        const float T2 = T1;
        const Eigen::Vector2f zeroAcc(0.0f, 0.0f);

        Eigen::Vector2f midVel = QuinticPolynomial::solveMidVel2D(
            startPos, midPos, finishPos, startVel, finishVel, T1, T2);
        Eigen::Vector2f midAcc = QuinticPolynomial::solveMidAcc2D(startPos,
                                                                  startVel,
                                                                  zeroAcc,
                                                                  midPos,
                                                                  midVel,
                                                                  finishPos,
                                                                  finishVel,
                                                                  zeroAcc,
                                                                  T1,
                                                                  T2);
        // 计算中点速度 BY GPT-5.3-Codex

        Trajectory first = QuinticPolynomial::generate(
            startPos, midPos, startVel, midVel, zeroAcc, midAcc, T1, totalSteps / 2);
        Trajectory second = QuinticPolynomial::generate(
            midPos, finishPos, midVel, finishVel, midAcc, zeroAcc, T2, totalSteps / 2);

        trajectory_ = first + second;

        return !trajectory_.points.empty();
    }

    bool StandTrajectory::start(const rclcpp::Time &startTime) {
        startTimeNanoSec_ = static_cast<size_t>(startTime.nanoseconds());
        trajectory_.progress = 0;

        // generate 已经判定“无需过渡”时，直接保持 target_
        if (finished_) {
            return true;
        }

        if (trajectory_.points.empty()) {
            return false;
        }

        if (trajectory_.time.size() != trajectory_.points.size()) {
            trajectory_.time.resize(trajectory_.points.size());
        }

        for (size_t i = 0; i < trajectory_.time.size(); ++i) {
            trajectory_.time[i] = startTimeNanoSec_ + i * controlDurationNanoSec_;
        }

        target_ = trajectory_.points.front();
        target_.kp[0] = parameters_.transitionKp;
        target_.kp[1] = parameters_.transitionKp;
        target_.kd[0] = parameters_.transitionKd;
        target_.kd[1] = parameters_.transitionKd;
        return true;
    }

    TrajectoryPoint StandTrajectory::get(const rclcpp::Time &currentTime) {
        if (finished_) {
            return target_;
        }
        const size_t currentNanoSec = static_cast<size_t>(currentTime.nanoseconds());

        auto assignTransitionGains = [this]() {
            target_.kp[0] = parameters_.transitionKp;
            target_.kp[1] = parameters_.transitionKp;
            target_.kd[0] = parameters_.transitionKd;
            target_.kd[1] = parameters_.transitionKd;
        };

        auto assignStandGains = [this]() {
            target_.vel.setZero();
            target_.tau[0] = 0;
            target_.tau[1] = 0;
            target_.tau[2] = (Const::BODY_MASS * Const::G) / 4;
            target_.kp[0] = parameters_.standKp;
            target_.kp[1] = parameters_.standKp;
            target_.kd[0] = parameters_.standKd;
            target_.kd[1] = parameters_.standKd;
        };

        if (currentNanoSec <= trajectory_.time.front()) {
            target_ = trajectory_.points.front();
            assignTransitionGains();
            trajectory_.progress = 0;
            return target_;
        }

        if (currentNanoSec >= trajectory_.time.back()) {
            target_ = trajectory_.points.back();
            assignStandGains();
            trajectory_.progress = trajectory_.points.size();
            finished_ = true;
            return target_;
        }

        interpolate(currentNanoSec);

        if (trajectory_.progress == trajectory_.points.size()) {
            assignStandGains();
            finished_ = true;
        } else {
            assignTransitionGains();
        }
        return target_;
    }

} // namespace Dog8Dof

namespace YAML {
    template <> struct convert<Dog8Dof::StandTrajectory::parameters_t> {
        static Node encode(const Dog8Dof::StandTrajectory::parameters_t &rhs) {
            Node node;
            node["standKp"] = rhs.standKp;
            node["standKd"] = rhs.standKd;

            node["transitionKp"] = rhs.transitionKp;
            node["transitionKd"] = rhs.transitionKd;
            node["transitionVelMax"] = rhs.transitionVelMax;

            node["reachThreshold"] = sqrt(rhs.reachThresholdSQ);
            node["stepHeightFactor"] = rhs.stepHeightFactor;
            node["transitionTime"] = rhs.transitionTime;
            return node;
        }

        static bool decode(const Node &node,
                           Dog8Dof::StandTrajectory::parameters_t &rhs) {
            rhs.standKp = node["standKp"].as<float>(0.001);
            rhs.standKd = node["standKd"].as<float>(0.001);

            rhs.transitionKp = node["transitionKp"].as<float>(0.001);
            rhs.transitionKd = node["transitionKd"].as<float>(0.001);
            rhs.transitionVelMax = node["transitionVelMax"].as<float>(100.0);

            rhs.reachThresholdSQ = node["reachThreshold"].as<float>(100.0) *
                                   node["reachThreshold"].as<float>(100.0);
            rhs.stepHeightFactor = node["stepHeightFactor"].as<float>(0.1);
            rhs.transitionTime = node["transitionTime"].as<float>(0.5);
            return true;
        }
    };
} // namespace YAML