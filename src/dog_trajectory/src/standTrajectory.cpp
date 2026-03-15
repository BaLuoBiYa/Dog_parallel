#include "dog_trajectory/standTrajectory.hpp"

namespace Dog8Dof {
    StandTrajectory::StandTrajectory(const std::string &configPath,
                                     const Body &initTipPos,
                                     const rclcpp::Time &initTime)
        : BaseTrajectory(configPath) {
        if (!reloadParameters()) {
            throw std::invalid_argument("Could not load Config YAML!");
        }
        // 加载参数
        if (!start(initTime, initTipPos)) {
            throw std::invalid_argument("Could init state!");
        }
        // 初始化姿态
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

    bool StandTrajectory::start(const rclcpp::Time currentTime, const Body &currentTip) {
        (void)currentTip;
        startTime_ = currentTime;
        finished_ = false;
        return true;
    }

    TrajectoryPoint StandTrajectory::generate(const rclcpp::Time currentTime, const Body &currentTip) {
        const float distanceSQ = (currentTip.pos - parameters_.standTipPos).squaredNorm();
        if (distanceSQ <= parameters_.reachThresholdSQ) {
            target_.kp[0] = parameters_.standKp;
            target_.kp[1] = parameters_.standKp;
            target_.kd[0] = parameters_.standKd;
            target_.kd[1] = parameters_.standKd;
            target_.pos = parameters_.standTipPos;
            target_.vel.setZero();
            target_.tau.setZero();
            finished_ = true;
            return target_;
            // 判定为已经处于站立姿态，保持不变
        } else {
            rclcpp::Duration elapsedTime = currentTime - startTime_;
            float phase = elapsedTime.seconds() / duration_.seconds();
            if (phase > 1) {
                target_.kp[0] = parameters_.standKp;
                target_.kp[1] = parameters_.standKp;
                target_.kd[0] = parameters_.standKd;
                target_.kd[1] = parameters_.standKd;
                target_.pos = currentTip.pos;
                target_.vel.setZero();
                target_.tau.setZero();
                finished_ = true;
                return target_;
                // 超时仍未到达，以站立参数保持在当前位置
            } else {

                // Cosine ease-in-out interpolation
                float smooth_phase = 0.5f * (1.0f - cos(phase * M_PI));

                float final_target_x = parameters_.standTipPos[0];
                float final_target_z = parameters_.standTipPos[2];

                float target_x = startTipPos_[0] + (final_target_x - startTipPos_[0]) * smooth_phase;
                float target_z = startTipPos_[2] + (final_target_z - startTipPos_[2]) * smooth_phase;

                target_.pos[0] = xFilter_.get_Lowpass(target_x);
                target_.pos[2] = zFilter_.get_Lowpass(target_z);
                target_.kp[0] = parameters_.transitionKp;
                target_.kp[1] = parameters_.transitionKp;
                target_.kd[0] = parameters_.transitionKd;
                target_.kd[1] = parameters_.transitionKd;
                return target_;
            }
        }
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
            node["filterAlpha"] = rhs.filterAlpha;
            node["reachThreshold"] = sqrt(rhs.reachThresholdSQ);
            node["duration"] = rhs.duration.seconds();
            node["standTip"]["x"] = rhs.standTipPos[0];
            node["standTip"]["z"] = rhs.standTipPos[2];
            return node;
        }

        static bool decode(const Node &node, Dog8Dof::StandTrajectory::parameters_t &rhs) {
            rhs.standKp = node["standKp"].as<float>(0.0);
            rhs.standKd = node["standKd"].as<float>(0.0);
            rhs.filterAlpha = node["filterAlpha"].as<float>(0.0);
            rhs.transitionKp = node["transitionKp"].as<float>(0.0);
            rhs.transitionKd = node["transitionKd"].as<float>(0.0);
            rhs.reachThresholdSQ = node["reachThreshold"].as<float>(0.0) * node["reachThreshold"].as<float>(0.0);
            rhs.duration = rclcpp::Duration::from_seconds(node["duration"].as<float>(0.0));
            rhs.standTipPos[0] = node["standTip"]["x"].as<float>(0.0);
            rhs.standTipPos[1] = 0.0f;
            rhs.standTipPos[2] = node["standTip"]["z"].as<float>(0.0);
            return true;
        }
    };
} // namespace YAML