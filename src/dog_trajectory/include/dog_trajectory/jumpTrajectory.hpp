#include "dog_trajectory/base.hpp"

namespace Dog8Dof {
    class JumpTrajectory : public BaseTrajectory {
      public:
        explicit JumpTrajectory(const std::string &configPath, rclcpp::Duration period);

        /**
         * @brief 设置步行步态的目标
         * @param launchSpeed 起跳速度
         * @param flyTime 飞行时间
         */
        bool setTarget(float launchSpeed, float flyTime);
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

            float slipMass;
            float slipRestLength;
            float slipStiffness;
            float slipDamping;
            float slipTouchdownAngleDeg;
            float slipStanceMaxTime;
            float slipStopSpeed;
        };

      private:
        // float jumpLength_, jumpHight_;
        float takeoffX_ = 0.0f, takeoffZ_ = 0.0f;
        // float launchVx_ = 0.0f, launchVz_ = 0.0f;
        float pushTime_ = 0.0f;
        float targetLaunchSpeed_ = 0.0f;
        float targetFlyTime_ = 0.0f;
        float jumpLength_ = 0.0f; // 派生：机体位移
        float takeoffVx_ = 0.0f;  // 派生：离地水平速度
        float takeoffVz_ = 0.0f;  // 派生：离地竖直速度（向上为负）
        float takeoffFootX_ = 0.0f;
        float takeoffFootZ_ = 0.0f;
        float landingFootX_ = 0.0f;
        float landingFootZ_ = 0.0f;
        float touchdownX_ = 0.0f;
        float touchdownZ_ = 0.0f;
        float touchdownVx_ = 0.0f;
        float touchdownVz_ = 0.0f;
        parameters_t parameters_;

        Trajectory generateCrouching(const Body initTip);
        Trajectory generateSlipTakeoff(float targetVx, float targetVz);
        Trajectory generateSlipFlight(float flyTime);
        Trajectory generateSlipLanding();
    };

} // namespace Dog8Dof