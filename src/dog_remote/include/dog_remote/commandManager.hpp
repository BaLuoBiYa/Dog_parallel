#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

#include <thread>

#include "dog_msg/msg/trajectory_command.hpp"
#include "dog_remote/sbusReader.hpp"
#include "dog_tools/const.hpp"

namespace Dog8Dof {
    class CommandManager : public rclcpp::Node {
      public:
        CommandManager(std::string nodeName = "dog_translater");
        ~CommandManager();

      private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr visionCmd_;
        rclcpp::Publisher<dog_msg::msg::TrajectoryCommand>::SharedPtr commandPublisher_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr visionControl_;
        dog_msg::msg::TrajectoryCommand cmd_;

        std::unique_ptr<SbusReader> subs_;

        struct {
            uint8_t moveX;
            uint8_t moveY;
            uint8_t roll;
            uint8_t pitch;
            uint8_t height;
        } controllerMapping_;

        struct {
            float maxLinerSpeed;
            float maxAngularSpeed;

            float maxHeight;
            float maxStepDuration;

            float maxRoll;
            float maxPitch;

            float expFactor_;

            const uint16_t maxChannelValue = 1920;
            const uint16_t midChannelValue = maxChannelValue / 2;

            const uint16_t lv1ChannelValue = 1920;
            const uint16_t lv2ChannelValue = 1920;
            const uint16_t lv3ChannelValue = 1920;
        } remoteParams_;

        struct Command {
            float linerVel;
            float angularVel;
            float roll;
            float pitch;
        };

        float stepFreq_;
        float stepHeightFactor_ = 0.3f;
        float baisHeight_ = (Const::BODY_HEIGHT_MAX + Const::BODY_HEIGHT_MIN) / 2.0f;

        const float bodyLength = Const::BODY_LENGTH_LONGITUDINAL;
        const float bodyWidth = Const::BODY_WIDTH_LATERAL;

        std::atomic<bool> visionMode_ = false;

        const rclcpp::Logger logger_ = this->get_logger();
        std::thread sbusHandler_;

        bool cmdToGait(const Command cmd);
        void twistCallback(geometry_msgs::msg::Twist::SharedPtr msg);
        bool isInVisionMode() {
            return visionMode_;
        }
        void checkState(const std_srvs::srv::Trigger::Request::SharedPtr req,
                        std_srvs::srv::Trigger::Response::SharedPtr res);
        void remoteToCmd();

        float normalizeExp(uint16_t raw);
    };

} // namespace Dog8Dof