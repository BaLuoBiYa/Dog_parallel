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
        CommandManager(std::string configPath, std::string nodeName = "dog_translater");

      private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr visionCmd_;
        rclcpp::Publisher<dog_msg::msg::TrajectoryCommand>::SharedPtr commandPublisher_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr visionControl_;
        dog_msg::msg::TrajectoryCommand cmd_;

        std::unique_ptr<SbusReader> subs_;

        float expFactor_;
        struct {
            uint8_t moveX;
            uint8_t moveY;
            uint8_t slopeX;
            uint8_t slopeZ;
            uint8_t height;
        } controllerMapping_;

        float stepFreq_;
        float stepHeightFactor_ = 0.3f;
        float baisHeight_ = (Const::BODY_HEIGHT_MAX + Const::BODY_HEIGHT_MIN) / 2.0f;

        const float bodyLength = Const::BODY_LENGTH_LONGITUDINAL;
        const float bodyWidth = Const::BODY_WIDTH_LATERAL;

        bool visionMode_ = false;

        const rclcpp::Logger logger_ = this->get_logger();
        std::thread sbusHandler_;

        dog_msg::msg::TrajectoryCommand velTocmd(float vx, float vy);
        void twistToCmd(geometry_msgs::msg::Twist::SharedPtr msg);
        bool isInVisionMode() {
            return visionMode_;
        }
        void checkState(const std_srvs::srv::Trigger::Request::SharedPtr req,
                        std_srvs::srv::Trigger::Response::SharedPtr res);
        void remoteToCmd();
    };

} // namespace Dog8Dof