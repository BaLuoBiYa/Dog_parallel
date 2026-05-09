#include "dog_remote/twistToGait.hpp"

#include <cmath>
#include <iostream>
#include <thread>

namespace Dog8Dof {
    TwistToGait::TwistToGait() : rclcpp::Node("dog_translator") {
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/static_twist",
            10,
            std::bind(&TwistToGait::twistCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<dog_msg::msg::GaitTwist>("/dog/command", 1);

        RCLCPP_INFO(this->get_logger(),
                    "Twist->GaitTwist translator ready, /static_twist -> /dog/command");
        printStatus();
    }

    void TwistToGait::switchGait(const std::string &mode) {
        if (mode == "stand" || mode == "s") {
            currentGait_ = dog_msg::msg::GaitTwist::STAND;
        } else if (mode == "walk" || mode == "w") {
            currentGait_ = dog_msg::msg::GaitTwist::WALK;
        } else {
            std::cout << "[GaitSwitch] 未知模式: " << mode
                      << "，可用: stand(s) / walk(w)\n";
            return;
        }
        printStatus();
    }

    void TwistToGait::printStatus() const {
        std::cout << "[GaitSwitch] 当前步态: "
                  << (currentGait_ == dog_msg::msg::GaitTwist::STAND ? "STAND" : "WALK")
                  << " | 步高=" << stepHeight_ << " 基座高=" << baseHeight_
                  << " 周期=" << stepDuration_ << "s\n"
                  << "[GaitSwitch] 输入 s/stand 站立, w/walk 行走, q 退出\n";
    }

    void TwistToGait::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        dog_msg::msg::GaitTwist out;

        out.gait = currentGait_;

        // 从 Twist 提取速度
        out.linear_x = static_cast<float>(msg->linear.x);
        out.linear_y = static_cast<float>(msg->linear.y);
        out.yaw_rate = static_cast<float>(msg->angular.z);

        // 注入默认步态参数
        out.step_height = stepHeight_;
        out.base_height = baseHeight_;
        out.step_duration = stepDuration_;

        pub_->publish(out);

        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Twist->GaitTwist: gait="
                               << static_cast<int>(out.gait) << " vx=" << out.linear_x
                               << " vy=" << out.linear_y << " wz=" << out.yaw_rate);
    }
} // namespace Dog8Dof

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Dog8Dof::TwistToGait>();

    // 独立线程监听终端输入切换步态
    std::thread inputThread([&node]() {
        std::string line;
        while (rclcpp::ok() && std::getline(std::cin, line)) {
            if (line == "q" || line == "quit") {
                rclcpp::shutdown();
                break;
            }
            node->switchGait(line);
        }
    });

    rclcpp::spin(node);

    if (inputThread.joinable()) {
        inputThread.join();
    }
    return 0;
}