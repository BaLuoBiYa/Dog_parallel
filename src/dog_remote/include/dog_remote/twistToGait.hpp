#pragma once
#include <atomic>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dog_msg/msg/gait_twist.hpp"

namespace Dog8Dof {
    /**
     * @brief Twist → GaitTwist 转换节点
     *
     * 订阅标准 geometry_msgs/Twist，注入默认步态参数，发布 dog_msg/GaitTwist。
     * 步态模式通过终端命令手动切换（线程安全）。
     */
    class TwistToGait : public rclcpp::Node {
      public:
        TwistToGait();

        /** 终端切换步态："stand" / "walk" */
        void switchGait(const std::string &mode);
        void printStatus() const;

      private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
        rclcpp::Publisher<dog_msg::msg::GaitTwist>::SharedPtr pub_;

        std::atomic<uint8_t> currentGait_{dog_msg::msg::GaitTwist::STAND};

        float stepHeight_ = 100.0f;
        float baseHeight_ = 250.0f;
        float stepDuration_ = 0.5f;

        void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    };
} // namespace Dog8Dof