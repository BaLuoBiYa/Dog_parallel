#pragma once

#include "dog_trajectory/base.hpp"

#include <string>

namespace Dog8Dof {
    class JumpTrajectoryLegacy : public BaseTrajectory {
      public:
        using jumpIndex = float;
        explicit JumpTrajectoryLegacy(const std::string &configPath,
                                      rclcpp::Duration period);

        bool setTarget(jumpIndex configName);
        bool generate(const Body &currentTip) override;
        bool start(const rclcpp::Time &startTime) override;
        TrajectoryPoint get(const rclcpp::Time &currentTime) override;

        bool reloadParameters() override;

        struct parameters_t {
            float transitionKp = 1.0f;
            float transitionKd = 0.05f;
            float transitionTime = 0.4f;

            float baseHeight = 180.0f;
            float crouchDepth = 50.0f;
            float pushStroke = 60.0f;

            float crouchDuration = 0.30f;
            float pushDuration = 0.10f;
            float flightDuration = 0.30f;
            float landDuration = 0.25f;

            float flightApexZOffset = 80.0f;
            float landingPrepX = 30.0f;

            float pushExponentZ = 3.0f;
            float pushExponentX = 2.0f;
            std::string pushXMode = "smooth";

            float crouchForwardDist = 0.0f;
            float pushBackwardDist = 0.0f;
            float pushVelocityGain = 1.0f;

            float phaseKpCrouch = 0.6f;
            float phaseKdCrouch = 0.01f;
            float phaseKpPush = 1.2f;
            float phaseKdPush = 0.01f;
            float phaseKpFlight = 0.3f;
            float phaseKdFlight = 0.01f;
            float phaseKpLand = 0.3f;
            float phaseKdLand = 0.05f;
        };

        const std::map<jumpIndex, std::string> jumpMapping = {
            {0.0f, "default"},
            {1.0f, "high_jump"},
            {2.0f, "long_jump"},
            {3.0f, "custom1"},
            {4.0f, "custom2"},
        };

      private:
        enum class Phase { Crouching, PushingOff, InFlight, Landing };

        parameters_t parameters_;
        std::map<std::string, parameters_t> configMapping_;

        float initialFootZ_ = 180.0f;

        static float clamp01(float x);
        static float smoothInterp(float s);
        static float smoothInterpDeriv(float s, float duration);
        static float smoothInterpDeriv2(float s, float duration);

        static float explosiveInterp(float s, float exponent);
        static float explosiveInterpDeriv(float s, float exponent, float duration);
        static float explosiveInterpDeriv2(float s, float exponent, float duration);

        static void fillPoint(TrajectoryPoint &pt,
                              float x,
                              float z,
                              float vx,
                              float vz,
                              float kp,
                              float kd);

        void samplePhase(Phase phase,
                         float timeInPhase,
                         float &targetX,
                         float &targetZ,
                         float &targetVx,
                         float &targetVz) const;

        void appendPhase(Phase phase, float duration, Trajectory &traj) const;

        rclcpp::Logger logger_ = rclcpp::get_logger("jumpTrajectoryLegacy");
    };

} // namespace Dog8Dof