#pragma once

#include "dog_trajectory/base.hpp"
#include <Eigen/Dense>

namespace Dog8Dof {

    /**
     * @brief 跳跃步态轨迹生成器 — 三段式状态机，全髋坐标系输出
     *
     * Z=0 为髋部，正 Z 向下（远离髋部）。
     *
     * 三段流程：
     *   PUSH_OFF → FLIGHT → ABSORPTION → FINISHED
     *
     * 接口：setTarget() → generate() → get(now, currentTip) 每控制周期
     */
    class JumpTrajectory : public BaseTrajectory {
      public:
        explicit JumpTrajectory(const std::string &configPath, rclcpp::Duration period);
        enum Phase { PUSH_OFF = 0, FLIGHT = 1, ABSORPTION = 2, FINISHED = 3 };

        bool setTarget(float jumpLength, float jumpHeight);
        TrajectoryPoint generate(const Body &currentTip, const rclcpp::Time now);
        TrajectoryPoint get(const rclcpp::Time &currentTime,
                            const Body *currentTip = nullptr) override;
        bool reloadParameters() override;
        Phase getPhase() const {
            return phase_;
        }

        struct parameters_t {
            // 蹬地阶段
            float pushDuration = 0.15f;   // 蹬地总时长 (s)
            float crouchFraction = 0.30f; // 下蹲段占比 [0.2~0.4]
            float pushKp = 3.0f;          // 蹬地 PD-Kp
            float pushKd = 0.05f;         // 蹬地 PD-Kd
            float compressDepth = 30.0f;  // 下蹲最大压缩深度 (mm) [10~60]
            float toeOffLift = 3.0f;      // 离地微升量 (mm) [1~5]

            // 空中阶段
            float flightKp = 1.5f;         // 空中 PD-Kp
            float flightKd = 0.03f;        // 空中 PD-Kd
            float flightClearance = 20.0f; // 空中额外抬脚 (mm) [10~60]

            // 缓冲阶段
            float absorbDuration = 0.10f;  // 缓冲总时长 (s)
            float absorbKp = 4.0f;         // 缓冲 PD-Kp
            float absorbKd = 0.08f;        // 缓冲 PD-Kd
            float absorbCompress = 25.0f;  // 缓冲最大压缩量 (mm) [10~40]
            float legLengthStand = 270.0f; // 正常站立腿长 (mm)
        };

        friend class GaitManager;

      private:
        parameters_t parameters_;
        float jumpLength_ = 0.0f, jumpHeight_ = 0.0f;
        Phase phase_ = FINISHED;
        TrajectoryPoint target_;
        rclcpp::Time lastUpdateTime_;

        // ---- 蹬地阶段 ----
        float t_push_ = 0.0f, T_push_, T_crouch_, T_extend_;
        float x_rel_0_, z_rel_0_;
        float push_end_x_, push_end_vel_x_, push_end_vel_z_;
        Eigen::Matrix<float, 6, 1> coeff_push_x_; // X 轴（全程一段五次多项式）
        Eigen::Matrix<float, 6, 1> coeff_push_z1_; // Z 下蹲段
        Eigen::Matrix<float, 6, 1> coeff_push_z2_; // Z 伸展段

        // ---- 空中阶段 ----
        float t_flight_ = 0.0f, T_flight_;
        Eigen::Matrix<float, 6, 1> coeff_flight_x_;      // X 固定多项式（回放）
        Eigen::Matrix<float, 6, 1> coeff_flight_z_base_; // Z 基线多项式
        float x_flight_target_; // 落地 X（髋系，= x_rel_0_）
        float z_flight_land_;   // 落地 Z（髋系，= legLengthStand）
        float z_flight_launch_; // 蹬地结束 Z（髋系）

        // ---- 缓冲阶段 ----
        float t_absorb_ = 0.0f, T_absorb_;
        float L_land_;  // 落地腿长（髋系 Z）
        float L_stand_; // 站立腿长

        // ---- 私有方法 ----
        static Eigen::Matrix<float, 6, 1> quinticFromTo6(
            float p0, float v0, float a0, float pT, float vT, float aT, float T);

        float flightZ(float t, float T, float zStart, float zLand, float H_air) const;

        float
        absorbLegLength(float t, float T, float L0, float L_stand, float deltaL) const;

        rclcpp::Logger logger_ = rclcpp::get_logger("jumpTrajectory");
    };

} // namespace Dog8Dof
