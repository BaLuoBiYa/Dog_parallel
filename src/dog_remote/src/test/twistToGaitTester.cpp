#include "dog_remote/twistToGait.hpp"

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Dog8Dof::TwistToGait>();
    rclcpp::spin(node);
    return 0;
}