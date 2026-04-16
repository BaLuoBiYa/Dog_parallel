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

            YAML::Node channel = node["channel"];
            controllerMapping_.moveX = channel["move"]["x"].as<uint8_t>();
            controllerMapping_.moveY = channel["move"]["y"].as<uint8_t>();

            controllerMapping_.roll = channel["roll"].as<uint8_t>();
            controllerMapping_.pitch = channel["pitch"].as<uint8_t>();

            controllerMapping_.height = channel["height"].as<uint8_t>();

            YAML::Node controllerSetting = node["controlSetting"];
            remoteParams_.maxLinerSpeed = controllerSetting["maxLinerSpeed"].as<float>();
            remoteParams_.maxAngularSpeed =
                controllerSetting["maxAngularSpeed"].as<float>();
            remoteParams_.maxHeight = controllerSetting["maxHeight"].as<float>();
            remoteParams_.maxStepDuration =
                controllerSetting["maxStepDuration"].as<float>();
            remoteParams_.maxRoll = controllerSetting["maxRoll"].as<float>();
            remoteParams_.maxPitch = controllerSetting["maxPitch"].as<float>();
            remoteParams_.expFactor_ = controllerSetting["expFactor"].as<float>();

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

        cmd_.gait = dog_msg::msg::TrajectoryCommand::WALK;

        // 直线行走时计算步长
        // 转化为mm/s
        const float XLinearVelosity_mm = cmd.linerVel * 1000;
        // 速度/频率 = 步长
        const float stepLength = XLinearVelosity_mm / stepFreq_;

        // 转弯时计算步幅偏置
        const float expectedAngle = cmd.angularVel / stepFreq_;
        // 第三边
        const float anotherAngle = M_PI - atan2(bodyWidth, bodyLength) - expectedAngle;

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

        const double L2 = bodyLength * 0.5;
        const double W2 = bodyWidth * 0.5;

        const double croll = std::cos(cmd.roll);
        const double spitch = std::sin(cmd.pitch);
        const double sroll = std::sin(cmd.roll);

        auto zAt = [&](double x, double y) { return -croll * spitch * x + sroll * y; };

        std::array<double, 4> dz;
        dz[0] = zAt(+L2, +W2); // LF
        dz[1] = zAt(+L2, -W2); // LR
        dz[2] = zAt(-L2, +W2); // RF
        dz[3] = zAt(-L2, -W2); // RR

        for (size_t i = 0; i < 4; i++) {
            cmd_.par2[i] = stepHeightFactor_ * cmd_.par1[i]; // 计算步高
            cmd_.par3[i] = baisHeight_ + dz[i];              // 计算基座高度
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
            cmdToGait(cmd);
        } else {
            return;
        }
    }

    void CommandManager::remoteToCmd() {
        std::array<uint16_t, 16> remoteData;
        while (true) {
            if (visionMode_) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            if (subs_->hasFreshFrame()) {
                remoteData = subs_->getAllChannelValues();
                Command cmd;

                cmd.linerVel = normalizeExp(remoteData[controllerMapping_.moveX]) *
                               remoteParams_.maxLinerSpeed;
                cmd.angularVel = normalizeExp(remoteData[controllerMapping_.moveY]) *
                                 remoteParams_.maxAngularSpeed;
                cmd.pitch = normalizeExp(remoteData[controllerMapping_.pitch]) *
                            remoteParams_.maxPitch;
                cmd.roll = normalizeExp(remoteData[controllerMapping_.roll]) *
                           remoteParams_.maxRoll;

                cmdToGait(cmd);
                continue;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
        float mag = std::pow(std::fabs(v), remoteParams_.expFactor_); // 非线性指数
        return std::copysign(mag, v);                                 // 恢复符号
    }

} // namespace Dog8Dof

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto cm = std::make_shared<Dog8Dof::CommandManager>();
    rclcpp::spin(cm);
    rclcpp::shutdown();
    return 0;
}