#include "dog_trajectory/walkTrajectory.hpp"

namespace Dog8Dof {
    WalkTrajectory::WalkTrajectory(const std::string &configPath, const Body &initTipPos, const rclcpp::Time &initTime)
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

    bool WalkTrajectory::reloadParameters() {
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
        vxFilter_.set_Alpha(parameters_.filterAlpha);
        vzFilter_.set_Alpha(parameters_.filterAlpha);

        xSmoother_.set_params(parameters_.smootherW, parameters_.smootherD);
        zSmoother_.set_params(parameters_.smootherW, parameters_.smootherD);
        return true;
    }

    bool WalkTrajectory::start(const rclcpp::Time currentTime, const Body &currentTip) {

        // 重置平滑着陆状态变量
        landingMode_ = false;
        landingProgress_ = 0.0;
        landingStartPos_.Zero();

        // Use the actual current foot positions from the state message to reset filters.
        // This ensures a smooth transition from any state (like stand) to walking.
        xFilter_.reset();
        zFilter_.reset();
        lastPos_[0] = xFilter_.get_Lowpass(currentTip.pos[0]);
        lastPos_[1] = 0.0;
        lastPos_[2] = zFilter_.get_Lowpass(currentTip.pos[2]);

        xSmoother_.reset(currentTip.pos[0], currentTip.linerVel[0]);
        zSmoother_.reset(currentTip.pos[2], currentTip.linerVel[2]);

        startTime_ = currentTime;
        finished_ = false;
        return true;
    }

    bool WalkTrajectory::setTarget(Phase phase, float stepLength, float stepHeight) {
        phase_ = phase;
        stepLength_ = stepLength;
        stepHeight_ = stepHeight;
        return true;
    }

} // namespace Dog8Dof

namespace YAML {
    template <> struct convert<Dog8Dof::WalkTrajectory::parameters_t> {
        static Node encode(const Dog8Dof::WalkTrajectory::parameters_t &rhs) {
            Node node;
            node["walkKp"] = rhs.walkKp;
            node["walkKd"] = rhs.walkKd;
            node["filterAlpha"] = rhs.filterAlpha;

            node["pushGain"]["horizontal"] = rhs.horizontalPushGain;
            node["pushGain"]["vertical"] = rhs.verticalPushGain;

            node["accelerationGain"]["x"] = rhs.accelerationGainX;
            node["accelerationGain"]["z"] = rhs.accelerationGainZ;

            node["useSmootherTrajectory"] = rhs.useSmootherTrajectory_;
            node["smoother"]["w"] = rhs.smootherW;
            node["smoother"]["d"] = rhs.smootherD;

            node["swingContactThreshold"] = rhs.swingContactThreshold;
            node["duration"] = rhs.duration.seconds();
            return node;
        }

        static bool decode(const Node &node, Dog8Dof::WalkTrajectory::parameters_t &rhs) {
            rhs.walkKp = node["walkKp"].as<float>(0.0);
            rhs.walkKd = node["walkKd"].as<float>(0.0);
            rhs.filterAlpha = node["filterAlpha"].as<float>(0.0);

            rhs.horizontalPushGain = node["pushGain"]["horizontal"].as<float>(0.0);
            rhs.verticalPushGain = node["pushGain"]["vertical"].as<float>(0.0);

            rhs.accelerationGainX = node["accelerationGain"]["x"].as<float>(0.0);
            rhs.accelerationGainZ = node["accelerationGain"]["z"].as<float>(0.0);

            rhs.useSmootherTrajectory_ = node["useSmootherTrajectory"].as<bool>(false);
            rhs.smootherW = node["smoother"]["w"].as<float>(0.0);
            rhs.smootherD = node["smoother"]["d"].as<float>(0.0);

            rhs.swingContactThreshold = node["swingContactThreshold"].as<float>(0.0);
            rhs.duration = rclcpp::Duration::from_seconds(node["duration"].as<float>(0.0));
            return true;
        }
    };
} // namespace YAML