#include "dog_remote/commandManager.hpp"

namespace Dog8Dof {
    CommandManager::CommandManager(std::string nodeName) : rclcpp::Node(nodeName) {
        std::string configPath = this->declare_parameter<std::string>("configPath");

        std::string portName, twistTopic, commandTopic, modeTopic;
        try {
            YAML::Node node = YAML::LoadFile(configPath);

            portName = node["port"].as<std::string>();

            YAML::Node topic = node["topic"];
            twistTopic = topic["twist"].as<std::string>();
            modeTopic = topic["mode"].as<std::string>();

            controllerMapping_ = node["channel"].as<controllerMapping_t>();
            remoteParams_ = node["controlSetting"].as<remoteParams_t>();
        } catch (std::exception &e) {
            RCLCPP_ERROR_STREAM(logger_, "Can't read config file");
            throw std::runtime_error(e.what());
        }

        try {
            subs_ = std::make_unique<SbusReader>(portName, 100000);
        } catch (std::exception &e) {
            RCLCPP_ERROR_STREAM(logger_, "Can't connect to remote");
            throw std::runtime_error(e.what());
        }

        visionCmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            twistTopic,
            rclcpp::QoS(1),
            std::bind(&CommandManager::twistCallback, this, std::placeholders::_1));

        commandPublisher_ = this->create_publisher<dog_msg::msg::TrajectoryCommand>(
            commandTopic, rclcpp::QoS(1));

        visionControl_ = this->create_service<std_srvs::srv::Trigger>(
            modeTopic,
            std::bind(&CommandManager::checkState,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));

        if (sbusHandler_.joinable()) {
            sbusHandler_.join();
        }

        sbusHandler_ = std::thread([this] { this->remoteToCmd(); });
    }

    CommandManager::~CommandManager() {
        if (sbusHandler_.joinable()) {
            sbusHandler_.join();
        }
    }

    bool CommandManager::cmdToGait(const Command cmd) {

        switch (cmd.gait) {
        case cmd.Gait::WALK: {
            cmd_.gait = dog_msg::msg::TrajectoryCommand::WALK;
            // 直线行走时计算步长
            // 转化为mm/s
            const float XLinearVelosity_mm = cmd.linerVel * 1000;
            // 速度/频率 = 步长
            const float stepLength = XLinearVelosity_mm / cmd.stepDuration;

            // 转弯时计算步幅偏置
            const float expectedAngle = cmd.angularVel / cmd.stepDuration;
            // 第三边
            const float anotherAngle =
                M_PI - atan2(bodyWidth, bodyLength) - expectedAngle;

            // 步长偏移
            float bias = 0.0f;
            // 防止计算得到的数值小于0
            if (anotherAngle < 0) {
                bias = 0;
            } else {
                // 通过正弦定理得到步长偏置
                bias = sin(expectedAngle) *
                       (sqrt(bodyLength * bodyLength + bodyWidth * bodyWidth) / 2) /
                       sin(anotherAngle);
            }

            cmd_.par1[0] = stepLength + bias;
            cmd_.par1[1] = stepLength + bias;
            cmd_.par1[2] = stepLength - bias;
            cmd_.par1[3] = stepLength - bias;
            // 写入步长

            std::array<float, 4> dz = getDetlaZ(cmd);
            for (uint8_t i = 0; i < 4; i++) {
                if (cmd_.par1[i] == 0.0f) {
                    cmd_.par2[i] = 20.0f;
                    cmd_.par3[i] = visoinBaisHeight_ + dz[i];
                    cmd_.par4[i] = cmd.stepDuration;
                    continue;
                }

                cmd_.par2[i] = stepHeightFactor_ * cmd_.par1[i]; // 计算步高
                cmd_.par3[i] = visoinBaisHeight_ + dz[i];        // 计算基座高度
                cmd_.par4[i] = cmd.stepDuration;                 // 步态周期

                if (std::abs(cmd_.par1[i]) > Const::STEP_LENGTH_MAX) {
                    RCLCPP_WARN(this->get_logger(), "Step length out of range");
                    return false;
                }
                if (cmd_.par2[i] > cmd_.par3[i] || cmd_.par2[i] < 0.0f) {
                    RCLCPP_WARN(this->get_logger(), "Step height out of range");
                    return false;
                }
                if (cmd_.par3[i] > Const::BODY_HEIGHT_MAX ||
                    cmd_.par3[i] < Const::BODY_HEIGHT_MIN) {
                    RCLCPP_WARN(this->get_logger(), "Base height out of range");
                    return false;
                }
                if (cmd_.par4[i] < 0.0f) {
                    RCLCPP_WARN(this->get_logger(), "Step duration out of range");
                    return false;
                }
            }
        } break;

        case cmd.Gait::STAND: {
            cmd_.gait = dog_msg::msg::TrajectoryCommand::STAND;
            std::array<float, 4> dz = getDetlaZ(cmd);

            for (uint8_t i = 0; i < 4; ++i) {
                cmd_.par1[i] = 0.0f;                   // tipX
                cmd_.par2[i] = cmd.baseHeight + dz[i]; // tipZ
                cmd_.par3[i] = 0.0f;                   // Unused
                cmd_.par4[i] = cmd.stepDuration;
            }
        } break;

        default:
            cmd_.gait = dog_msg::msg::TrajectoryCommand::STAND;
            for (uint8_t i = 0; i < 4; ++i) {
                cmd_.par1[i] = 0.0f;
                cmd_.par2[i] = visoinBaisHeight_;
                cmd_.par3[i] = 0.0f;
                cmd_.par4[i] = visionStepDuration_;
            }
            RCLCPP_WARN(this->get_logger(), "Unknown gait, back to stand");
            break;
        }

        commandPublisher_->publish(cmd_);
        RCLCPP_DEBUG(this->get_logger(), "Gait updated");
        return true;
    }

    void CommandManager::checkState(const std_srvs::srv::Trigger::Request::SharedPtr req,
                                    std_srvs::srv::Trigger::Response::SharedPtr res) {
        (void)req;
        if (visionMode_) {
            res->message = "Switch to remote mode";
            visionMode_ = false;
            res->success = true;
        } else {
            res->message = "Switch to vision mode";
            visionMode_ = true;
            res->success = true;
        }
    }

    void CommandManager::twistCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
        if (visionMode_) {
            Command cmd;
            cmd.linerVel = msg->linear.x;
            cmd.angularVel = msg->angular.z;
            cmd.pitch = msg->angular.y;
            cmd.roll = msg->angular.x;
            cmd.stepDuration = visionStepDuration_;
            cmd.baseHeight = visoinBaisHeight_;
            cmd.gait = Command::WALK;
            cmdToGait(cmd);
        } else {
            return;
        }
    }

    void CommandManager::remoteToCmd() {
        std::array<uint16_t, 16> remoteData;
        while (rclcpp::ok()) {
            if (visionMode_) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            if (subs_->hasFreshFrame()) {
                remoteData = subs_->getAllChannelValues();
                Command cmd;
                RemoteRank gaitCode = getRank(remoteData[controllerMapping_.gait]);
                RemoteRank slopeEnableCode =
                    getRank(remoteData[controllerMapping_.slopeEnable]);

                if (gaitCode == RemoteRank::LV1) {
                    cmd.gait = Command::STAND;

                } else if (gaitCode == RemoteRank::LV2) {
                    cmd.gait = Command::WALK;
                    cmd.linerVel = normalizeExp(remoteData[controllerMapping_.moveX]) *
                                   remoteParams_.maxLinerSpeed;
                    cmd.angularVel = normalizeExp(remoteData[controllerMapping_.moveY]) *
                                     remoteParams_.maxAngularSpeed;
                } else if (gaitCode == RemoteRank::LV3) {
                    cmd.gait = Command::JUMP;
                } else {
                    RCLCPP_WARN(this->get_logger(), "Unknown Gait, ignore");
                    continue;
                }

                if (slopeEnableCode == RemoteRank::LV1) {
                    cmd.pitch = normalizeExp(remoteData[controllerMapping_.pitch]) *
                                remoteParams_.maxPitch;
                    cmd.roll = normalizeExp(remoteData[controllerMapping_.roll]) *
                               remoteParams_.maxRoll;
                } else {
                    cmd.pitch = 0.0f;
                    cmd.roll = 0.0f;
                }

                cmd.stepDuration =
                    normalizeExp(remoteData[controllerMapping_.stepDuration]) *
                    remoteParams_.maxStepDuration;
                cmd.baseHeight = normalizeExp(remoteData[controllerMapping_.height]) *
                                 remoteParams_.maxHeight;

                cmdToGait(cmd);
                continue;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
        }
    }

    float CommandManager::normalizeExp(uint16_t raw) {
        float v = (static_cast<float>(raw) - remoteParams_.midChannelValue) /
                  remoteParams_.midChannelValue;
        if (v > 1.0f)
            v = 1.0f;
        if (v < -1.0f)
            v = -1.0f;
        float mag = std::pow(std::fabs(v), remoteParams_.expFactor); // 非线性指数
        return std::copysign(mag, v);                                // 恢复符号
    }

    std::array<float, 4> CommandManager::getDetlaZ(const Command &cmd) {
        const float L2 = bodyLength * 0.5;
        const float W2 = bodyWidth * 0.5;

        const float croll = std::cos(cmd.roll);
        const float spitch = std::sin(cmd.pitch);
        const float sroll = std::sin(cmd.roll);

        auto zAt = [&](float x, float y) { return -croll * spitch * x + sroll * y; };

        std::array<float, 4> dz;
        dz[0] = zAt(+L2, +W2); // LF
        dz[1] = zAt(+L2, -W2); // LR
        dz[2] = zAt(-L2, +W2); // RF
        dz[3] = zAt(-L2, -W2); // RR

        return dz;
    }

    CommandManager::RemoteRank CommandManager::getRank(uint16_t data) {
        if (data > remoteParams_.lv3ChannelValue) {
            return RemoteRank::LV3;
        } else if (data > remoteParams_.lv2ChannelValue) {
            return RemoteRank::LV2;
        } else if (data > remoteParams_.lv1ChannelValue) {
            return RemoteRank::LV1;
        } else {
            return RemoteRank::LV1; // 默认最低等级
        }
    }

} // namespace Dog8Dof

namespace YAML {
    template <> struct convert<Dog8Dof::CommandManager::remoteParams_t> {
        static Node encode(const Dog8Dof::CommandManager::remoteParams_t &rhs) {
            Node node;
            node["maxLinerSpeed"] = rhs.maxLinerSpeed;
            node["maxAngularSpeed"] = rhs.maxAngularSpeed;
            node["maxHeight"] = rhs.maxHeight;
            node["maxStepDuration"] = rhs.maxStepDuration;
            node["maxRoll"] = rhs.maxRoll;
            node["maxPitch"] = rhs.maxPitch;
            node["expFactor"] = rhs.expFactor;
            return node;
        }

        static bool decode(const Node &node,
                           Dog8Dof::CommandManager::remoteParams_t &rhs) {
            rhs.maxLinerSpeed = node["maxLinerSpeed"].as<float>();
            rhs.maxAngularSpeed = node["maxAngularSpeed"].as<float>();
            rhs.maxHeight = node["maxHeight"].as<float>();
            rhs.maxStepDuration = node["maxStepDuration"].as<float>();
            rhs.maxRoll = node["maxRoll"].as<float>();
            rhs.maxPitch = node["maxPitch"].as<float>();
            rhs.expFactor = node["expFactor"].as<float>();
            return true;
        }
    };

    template <> struct convert<Dog8Dof::CommandManager::controllerMapping_t> {
        static Node encode(const Dog8Dof::CommandManager::controllerMapping_t &rhs) {
            Node node;

            node["move"]["x"] = rhs.moveX;
            node["move"]["y"] = rhs.moveY;

            node["roll"] = rhs.roll;
            node["pitch"] = rhs.pitch;

            node["height"] = rhs.height;
            node["stepDuration"] = rhs.stepDuration;
            node["gait"] = rhs.gait;
            node["slopeEnable"] = rhs.slopeEnable;
            return node;
        }

        static bool decode(const Node &node,
                           Dog8Dof::CommandManager::controllerMapping_t &rhs) {

            rhs.moveX = node["move"]["x"].as<uint8_t>();
            rhs.moveY = node["move"]["y"].as<uint8_t>();

            rhs.roll = node["roll"].as<uint8_t>();
            rhs.pitch = node["pitch"].as<uint8_t>();

            rhs.height = node["height"].as<uint8_t>();
            rhs.stepDuration = node["stepDuration"].as<uint8_t>();
            rhs.gait = node["gait"].as<uint8_t>();
            rhs.slopeEnable = node["slopeEnable"].as<uint8_t>();
            return true;
        }
    };
} // namespace YAML

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto cm = std::make_shared<Dog8Dof::CommandManager>();
    rclcpp::spin(cm);
    rclcpp::shutdown();
    return 0;
}