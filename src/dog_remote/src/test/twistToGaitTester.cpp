#include "dog_remote/twistToGait.hpp"
namespace Dog8Dof {
    TwistToGait::TwistToGait() : rclcpp::Node("dog_translator") {
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/static_twist",
            10,
            std::bind(&TwistToGait::twistCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<dog_msg::msg::TrajectoryCommand>("/dog/command", 1);

        lastXLinearVelosity_ = 0.0f;
        lastZAngularVelosity_ = 0.0f;
    }

    void TwistToGait::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

        // 提取数据内容
        const float XLinearVelosity = msg->linear.x;
        const float ZAngularVelosity = msg->angular.z;

        // if (std::abs(lastXLinearVelosity_ - XLinearVelosity) > 0.01f ||
        //     std::abs(lastZAngularVelosity_ - ZAngularVelosity) > 0.01f) {
        //     RCLCPP_INFO_STREAM(
        //         this->get_logger(),
        //         "Received Twist - Linear X: " << XLinearVelosity
        //                                       << " Angular Z: " << ZAngularVelosity);
        //     lastXLinearVelosity_ = XLinearVelosity;
        //     lastZAngularVelosity_ = ZAngularVelosity;
        // } else {
        //     // 如果变化不大，则不进行计算，直接返回
        //     return;
        // }

        dog_msg::msg::TrajectoryCommand command;
        command.gait = dog_msg::msg::TrajectoryCommand::WALK;

        // 直线行走时计算步长
        // 转化为mm/s
        const float XLinearVelosity_mm = XLinearVelosity * 1000;
        // 速度/频率 = 步长
        const float stepLength = XLinearVelosity_mm / stepDuration_;

        // 转弯时计算步幅偏置
        const float expectedAngle = ZAngularVelosity / stepDuration_;
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

        command.par1[0] = stepLength + bias;
        command.par1[1] = stepLength + bias;
        command.par1[2] = stepLength - bias;
        command.par1[3] = stepLength - bias;
        // 写入步长

        for (size_t i = 0; i < 4; i++) {
            command.par2[i] = stepHeight_;
            command.par3[i] = baisHeight_;
            command.par4[i] = stepDuration_;
        }

        pub_->publish(command);
        RCLCPP_INFO_STREAM(this->get_logger(), "LF step: " << command.par1[0]);
    }
} // namespace Dog8Dof

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Dog8Dof::TwistToGait>();
    rclcpp::spin(node);
    return 0;
}