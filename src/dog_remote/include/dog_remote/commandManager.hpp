#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "dog_msg/msg/trajectory_command.hpp"
#include "dog_tools/const.hpp"

namespace Dog8Dof {
    class CommandManager : public rclcpp::Node {
      public:
        CommandManager(std::string nodeName = "dog_translater");

      private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSub_;
        rclcpp::Publisher<dog_msg::msg::TrajectoryCommand>::SharedPtr commandPublisher_;
        dog_msg::msg::TrajectoryCommand cmd_;

        struct Command {
            float linerVel;
            float angularVel;
            float roll;
            float pitch;
            float stepDuration;
            float baseHeight;
            enum Gait { STAND = 0, WALK = 1, JUMP = 2 } gait;
        };

        float stepHeightFactor_ = 0.3f;
        float stepDuration_ = 0.5f;
        float baseHeight_ = 200.0f;

        const float bodyLength = Const::BODY_LENGTH_LONGITUDINAL;
        const float bodyWidth = Const::BODY_WIDTH_LATERAL;

        const rclcpp::Logger logger_ = this->get_logger();

        bool cmdToGait(const Command cmd);
        void twistCallback(geometry_msgs::msg::Twist::SharedPtr msg);

        std::array<float, 4> getDetlaZ(const Command &cmd);
    };

} // namespace Dog8Dof