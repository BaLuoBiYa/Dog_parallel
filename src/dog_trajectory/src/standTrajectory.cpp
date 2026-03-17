#include "dog_trajectory/standTrajectory.hpp"

namespace Dog8Dof {
    StandTrajectory::StandTrajectory(const std::string &configPath, rclcpp::Duration period)
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
            std::cout << "YAML Error" << e.what() << std::endl;
            return false;
        }
        xFilter_.set_Alpha(parameters_.filterAlpha);
        zFilter_.set_Alpha(parameters_.filterAlpha);

        return true;
    }

    bool StandTrajectory::generate(const Body &currentTip) {
        finished_ = false;
        progress_ = 0;
        target_ = TrajectoryPoint();
        trajectory_ = Trajectory(Const::RESOLUTION);

        const Eigen::Vector2<float> startPos(currentTip.pos[0], currentTip.pos[2]);
        const Eigen::Vector2<float> startVel(currentTip.linerVel[0], currentTip.linerVel[2]);

        const Eigen::Vector2<float> finishPos(standTipPos_[0], standTipPos_[1]);
        const Eigen::Vector2<float> finishVel(0, 0);

        const Eigen::Vector2<float> tangentVec = finishPos - startPos;
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
            target_.tau[3] = -(Const::BODY_MASS * Const::G) / 4;
            finished_ = true;
            return true;
        }

        const Eigen::Vector2<float> normVec0(-tangentVec[1], tangentVec[0]);
        const Eigen::Vector2<float> normVec1(tangentVec[1], -tangentVec[0]);
        // 计算起始向量的两个法向量

        Eigen::Vector2<float> directionVec;
        if (normVec0[1] <= 0) {
            directionVec = normVec0.normalized();
        } else {
            directionVec = normVec1.normalized();
        }
        // 朝向Z负方向（朝向天空的向量作为运动方向）

        Eigen::Vector2<float> midPos = (finishPos + startPos) / 2 + directionVec * stepHeight;
        if (midPos[1] <= Const::BODY_HEIGHT_MIN) {
            midPos[1] = Const::BODY_HEIGHT_MIN;
        } // 防止足尖穿过机体

        const float T1 = static_cast<float>(durationNanoSec_) / 1000 * 0.5;
        const float T2 = T1;

        const Eigen::Vector2f d1 = midPos - startPos;
        const Eigen::Vector2f d2 = finishPos - midPos;

        Eigen::Vector2f midVel =
            (6.0f * (d1 / (T1 * T1) + d2 / (T2 * T2)) - 2.0f * (startVel / T1 + finishVel / T2)) /
            (4.0f * (1.0f / T1 + 1.0f / T2));
        // 计算中点速度 BY GPT-5.3-Codex

        const float n = midVel.norm();
        if (n > parameters_.transitionVelMax && n > 1e-6f) {
            midVel *= (parameters_.transitionVelMax / n);
        }

        Trajectory first =
            CubicBezier::generate(startPos, midPos, startVel, midVel, T1, Const::RESOLUTION / 2);
        Trajectory second =
            CubicBezier::generate(midPos, finishPos, midVel, finishVel, T2, Const::RESOLUTION / 2);

        trajectory_ = first + second;

        return true;
    }

    bool StandTrajectory::start(const rclcpp::Time &startTime) {
        startTimeNanoSec_ = static_cast<size_t>(startTime.nanoseconds());
        progress_ = 0;

        // generate 已经判定“无需过渡”时，直接保持 target_
        if (finished_) {
            return true;
        }

        if (trajectory_.points.empty()) {
            return false;
        }

        // if (trajectory_.time.size() != trajectory_.points.size()) {
        //     trajectory_.time.resize(trajectory_.points.size());
        // }

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
    TrajectoryPoint StandTrajectory::get(const rclcpp::Time &currentTime) {
        if (finished_) {
            return target_;
        }

        const size_t pointCount = trajectory_.points.size();
        const size_t currentNanoSec = static_cast<size_t>(currentTime.nanoseconds());

        auto assignTransitionGains = [this]() {
            target_.kp[0] = parameters_.transitionKp;
            target_.kp[1] = parameters_.transitionKp;
            target_.kd[0] = parameters_.transitionKd;
            target_.kd[1] = parameters_.transitionKd;
        };

        auto assignStandGains = [this]() {
            target_.kp[0] = parameters_.standKp;
            target_.kp[1] = parameters_.standKp;
            target_.kd[0] = parameters_.standKd;
            target_.kd[1] = parameters_.standKd;
        };

        if (currentNanoSec <= trajectory_.time.front()) {
            target_ = trajectory_.points.front();
            assignStandGains();
            progress_ = 0;
            return target_;
        }

        if (currentNanoSec >= trajectory_.time.back()) {
            target_ = trajectory_.points.back();
            assignStandGains();
            progress_ = pointCount;
            finished_ = true;
            return target_;
        }
        const auto searchBegin = trajectory_.time.begin() + progress_;
        const auto upperIt = std::lower_bound(searchBegin, trajectory_.time.end(), currentNanoSec);

        const size_t upperIndex = static_cast<size_t>(upperIt - trajectory_.time.begin());
        const size_t lowerIndex = upperIndex - 1;

        const size_t t0 = trajectory_.time[lowerIndex];
        const size_t t1 = trajectory_.time[upperIndex];

        float alpha = 0.0f;
        if (t1 > t0) {
            alpha = static_cast<float>(currentNanoSec - t0) / static_cast<float>(t1 - t0);
        }
        alpha = std::clamp(alpha, 0.0f, 1.0f);

        const TrajectoryPoint &p0 = trajectory_.points[lowerIndex];
        const TrajectoryPoint &p1 = trajectory_.points[upperIndex];

        target_.pos = p0.pos + (p1.pos - p0.pos) * alpha;
        target_.vel = p0.vel + (p1.vel - p0.vel) * alpha;
        target_.tau = p0.tau + (p1.tau - p0.tau) * alpha;

        assignTransitionGains();
        progress_ = upperIndex;
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

            node["filterAlpha"] = rhs.filterAlpha;
            node["reachThreshold"] = sqrt(rhs.reachThresholdSQ);
            node["stepHeightFactor"] = rhs.stepHeightFactor;
            return node;
        }

        static bool decode(const Node &node, Dog8Dof::StandTrajectory::parameters_t &rhs) {
            rhs.standKp = node["standKp"].as<float>(0.0);
            rhs.standKd = node["standKd"].as<float>(0.0);

            rhs.transitionKp = node["transitionKp"].as<float>(0.0);
            rhs.transitionKd = node["transitionKd"].as<float>(0.0);
            rhs.transitionVelMax = node["transitionVelMax"].as<float>(0.0);

            rhs.filterAlpha = node["filterAlpha"].as<float>(0.0);
            rhs.reachThresholdSQ =
                node["reachThreshold"].as<float>(0.0) * node["reachThreshold"].as<float>(0.0);
            rhs.stepHeightFactor = node["stepHeightFactor"].as<float>(0.0);
            return true;
        }
    };
} // namespace YAML