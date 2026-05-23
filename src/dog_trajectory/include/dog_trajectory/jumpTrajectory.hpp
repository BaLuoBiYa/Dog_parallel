#pragma once

#include "dog_trajectory/base.hpp"
#include "dog_trajectory/standTrajectory.hpp"
#include <Eigen/Dense>
#include <memory>

namespace Dog8Dof {

    /**
     * @brief 跳跃步态轨迹生成器
     *
     * Z=0 为髋部，正 Z 向下（远离髋部）。
     *
     * 流程：
     *   PUSH_OFF → FLIGHT → ABSORPTION → FINISHED
     *
     * 接口：setTarget() → generate() → get(now, currentTip) 每控制周期
     */
    class JumpTrajectory : public BaseTrajectory {
      public:
        explicit JumpTrajectory(const std::string &configPath, rclcpp::Duration period);

        enum Phase { CROUCH, PUSH, FLIGHT, ABSORPTION, RECOVER, FINISHED };

        bool setTarget(uint8_t parID);
        TrajectoryPoint generate(const Body &currentTip, const rclcpp::Time now);
        TrajectoryPoint get(const rclcpp::Time &currentTime,
                            const Body *currentTip = nullptr) override;
        bool reloadParameters() override;
        Phase getPhase() const {
            return phase_;
        }

        struct parameters_t {
            // 全局参数
            float smoothKp = 2.0f;  // 平滑 PD-Kp (用于 CROUCH/RECOVER/FLIGHT 过渡)
            float smoothKd = 0.02f; // 平滑 PD-Kd (用于 CROUCH/RECOVER/FLIGHT 过渡)

            // 下蹲阶段
            float crouchDuration = 0.20f; // 下蹲总时长 (s)

            // 蹬地阶段
            float launchSpeed = 0.0f;   // 起跳速度
            float launchAngle = 3.0f;   // 起跳角度
            float pushDuration = 0.15f; // 蹬地总时长 (s)
            float pushKp = 3.0f;        // 蹬地 PD-Kp
            float pushKd = 0.05f;       // 蹬地 PD-Kd
            float startZ = 30.0f;       // 下蹲足端高度 (mm)
            float finishZ = 100.0f;     // 起跳足端高度 (mm)

            // 空中阶段
            float flightDuration = 0.30f; // 空中总时长 (s)
            float airTipLevelFactor = 0.1f; // 空中足端系数（影响空中状态切换的抬腿高度）

            // 缓冲阶段
            float absorbDuration = 0.10f; // 缓冲总时长 (s)
            float absorbDamp = 4.0f;      // 缓冲阻尼系数

            // 恢复阶段
            float recoverDuration = 0.25f; // 恢复总时长 (s)
            float legLengthStand = 270.0f; // 正常站立腿长 (mm)
        };

      private:
        uint8_t currentParID_ = 0;
        std::map<uint8_t, parameters_t> parameters_;
        Phase phase_ = FINISHED;
        TrajectoryPoint target_;
        rclcpp::Time lastUpdateTime_;
        Body idelTipState_;
        bool isInit_ = false;
        bool targetSet_ = false;

        // 组合 StandTrajectory 实例，用于 CROUCH/RECOVER/FLIGHT 阶段的平滑过渡
        std::unique_ptr<StandTrajectory> smooth_;

        /**
         * @brief PUSH 阶段五次多项式规划器
         *
         * 基于动量定理 M*ΔV = ∫F dt，使用五次多项式 s(t) 规划足端轨迹：
         *   s(t) = a0 + a1·t + a2·t² + a3·t³ + a4·t⁴ + a5·t⁵
         *
         * 边界条件（6 个约束 → 6 个系数）：
         *   s(0)=s₀,  s(T)=s_T          位置
         *   v(0)=v₀,  v(T)=v_T          速度
         *   a(0)=0,   a(T)=0            加速度=0 → 始终力=0
         *
         * 足端力由牛顿第二定律给出：F_perLeg(t) = (mass/4) · a(t)
         * 满足动量守恒：4·∫₀ᵀ F_perLeg dt = mass · (v_T − v₀)
         */
        class PushPlanner {
          public:
            PushPlanner(const Body &startState,
                        const Body &targetState,
                        const float &startTime,
                        float duration,
                        float mass,
                        float kp,
                        float kd);
            TrajectoryPoint get(const rclcpp::Time &currentTime);

          private:
            const Body startState_;
            const float startTime_;
            const float duration_;
            const float massPerLeg_; ///< 单腿承重 = 总质量/4
            TrajectoryPoint target_;

            /// 五次多项式系数，dim: 0=X轴, 1=Z轴
            Eigen::Matrix<float, 6, 1> coeff_[2];
        };
        std::unique_ptr<PushPlanner> pushPlanner_;

        // 绝对时间锚点，用于 phaseTime 相位计算
        rclcpp::Time launchTime_; ///< 跳跃整体开始时间

        rclcpp::Logger logger_ = rclcpp::get_logger("jumpTrajectory");

        Phase getCurrentPhase(const rclcpp::Time &currentTime);
        Body
        getFlightTarget(const Body &currentTip, float launchAngle, float launchSpeed);

        TrajectoryPoint getAbsorptionTarget(float absorbDamp);
    };

} // namespace Dog8Dof
