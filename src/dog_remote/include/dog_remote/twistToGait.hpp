#pragma once
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dog_msg/msg/trajectory_command.hpp"
#include "dog_tools/const.hpp"

namespace Dog8Dof {
    class TwistToGait : public rclcpp::Node {
      public:
        TwistToGait();

      private:
        std::string topicName_;
        rclcpp::Publisher<dog_msg::msg::TrajectoryCommand>::SharedPtr pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

        // 机身长宽，固定数值
        const float bodyLength = Const::BODY_LENGTH_LONGITUDINAL;
        const float bodyWidth = Const::BODY_WIDTH_LATERAL;

        const float stepHeight_ = 100.0f;
        const float baisHeight_ = 270.0f;
        // 假设视觉控制阶段步频固定为该值
        const float visualPlusNum = 30.0f;
        // 8000来自于生成轨迹生成时的计算phase分母，0.001为时间布长 （待优化）
        const float stepFrequency = (visualPlusNum / 8000) / 0.001;

        float lastXLinearVelosity_ = 0.0f;
        float lastZAngularVelosity_ = 0.0f;

        void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    };
} // namespace Dog8Dof