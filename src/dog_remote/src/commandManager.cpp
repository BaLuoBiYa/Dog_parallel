#include "dog_remote/commandManager.hpp"

namespace Dog8Dof {
    CommandManager::CommandManager(std::string configPath, std::string nodeName)
        : rclcpp::Node(nodeName) {

        std::string portName, visionTopic, commandTopic, visonControlService;
        try {
            YAML::Node node = YAML::LoadFile(configPath);
            expFactor_ = node["expFactor"].as<float>(0.5);
            portName = node["port"].as<std::string>();
            visionTopic = node["visonTopic"].as<std::string>();

            YAML::Node channel = node["channel"];
            controllerMapping_.moveX = channel["move"]["x"].as<uint8_t>();
            controllerMapping_.moveY = channel["move"]["y"].as<uint8_t>();

            controllerMapping_.slopeX = channel["slope"]["x"].as<uint8_t>();
            controllerMapping_.slopeZ = channel["slope"]["y"].as<uint8_t>();

            controllerMapping_.height = channel["height"].as<uint8_t>();
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
            visionTopic,
            rclcpp::QoS(1),
            std::bind(&CommandManager::twistToCmd, this, std::placeholders::_1));

        commandPublisher_ = this->create_publisher<dog_msg::msg::TrajectoryCommand>(
            commandTopic, rclcpp::QoS(1));

        visionControl_ = this->create_service<std_srvs::srv::Trigger>(
            visonControlService, &CommandManager::checkState);
    }

    void CommandManager::twistToCmd(const geometry_msgs::msg::Twist::SharedPtr msg) {

        cmd_.gait = dog_msg::msg::TrajectoryCommand::WALK;

        // 提取数据内容
        const float XLinearVelosity = msg->linear.x;
        const float ZAngularVelosity = msg->angular.z;

        // 直线行走时计算步长
        // 转化为mm/s
        const float XLinearVelosity_mm = XLinearVelosity * 1000;
        // 速度/频率 = 步长
        const float stepLength = XLinearVelosity_mm / stepFreq_;

        // 转弯时计算步幅偏置
        const float expectedAngle = ZAngularVelosity / stepFreq_;
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

        for (size_t i = 0; i < 4; i++) {
            cmd_.par2[i] = stepHeightFactor_ * cmd_.par1[i];
            cmd_.par3[i] = baisHeight_;
        }

        RCLCPP_INFO(this->get_logger(), "Gait updated");
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

} // namespace Dog8Dof