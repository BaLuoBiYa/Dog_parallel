#include "dog_trajectory/base.hpp"

namespace Dog8Dof {
    class JumpTrajectory : public BaseTrajectory {
      public:
        explicit JumpTrajectory(const std::string &configPath, rclcpp::Duration period);

        /**
         * @brief 设置步行步态的目标
         * @param jumpLength 跳跃高度
         * @param jumpHeight 跳跃距离
         */
        bool setTarget(float jumpLength, float jumpHeight);
        bool generate(const Body &currentTip) override;
        bool start(const rclcpp::Time &startTime) override;
        TrajectoryPoint get(const rclcpp::Time &currentTime) override;

        bool reloadParameters() override;

        struct parameters_t {
            float transitionKp;
            float transitionKd;
            float transitionTime;

            float crouchHeight;
            float crouchTime;

            float takeoffHeight;
            float pushTimeMin;
            float pushTimeMax;
            float pushTimeScale;

            float flyHeightRatio;
        };

      private:
        float jumpLength_, jumpHight_;
        float takeoffX_ = 0.0f, takeoffZ_ = 0.0f;
        float launchVx_ = 0.0f, launchVz_ = 0.0f;
        float pushTime_ = 0.0f;
        parameters_t parameters_;

        Trajectory generateCrouching(const Body initTip);
        Trajectory generatePushing(const float launchVel, const float launchAngle);
        Trajectory generateFlying(const float flyTime);
        Trajectory generateLanding();
    };

} // namespace Dog8Dof