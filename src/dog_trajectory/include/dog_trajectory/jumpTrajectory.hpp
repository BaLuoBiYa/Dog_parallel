// #include "dog_trajectory/base.hpp"

// namespace Dog8Dof {
//     class JumpTrajectory : public BaseTrajectory {
//       public:
//         explicit JumpTrajectory(const std::string &configPath, rclcpp::Duration
//         period);

//         /**
//          * @brief 设置跳跃步态的目标
//          * @param jumpLength 跳跃长度
//          * @param jumpHeight 跳跃高度
//          */
//         bool setTarget(float jumpLength, float jumpHeight);
//         TrajectoryPoint generate(const Body &currentTip) override;
//         // bool start(const rclcpp::Time &startTime) override;
//         // TrajectoryPoint get(const rclcpp::Time &currentTime) override;

//         bool reloadParameters() override;

//         struct parameters_t {
//             float crouchKp = 1.0f;
//             float crouchKd = 0.05f;
//             float crouchDuration = 0.3f;
//             float crouchDepth = 50.0f;

//             float pushDuration = 0.1f;

//             float landDuration = 0.25f;
//         };

//       private:
//         float jumpLength_ = 0.0f;
//         float jumpHeight_ = 0.0f;

//         parameters_t parameters_;

//         Trajectory generateCrouching(const Body initTip);
//         Trajectory generateTakeoff(float targetVx, float targetVz);
//         Trajectory generateFlight(float flyTime);
//         Trajectory generateLanding();

//         rclcpp::Logger logger_ = rclcpp::get_logger("jumpTrajectory");
//     };

// } // namespace Dog8Dof