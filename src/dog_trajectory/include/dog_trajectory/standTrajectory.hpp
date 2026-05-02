#pragma once

#include "dog_trajectory/base.hpp"
#include <Eigen/Dense>

namespace Dog8Dof {

    /**
     * @brief 站立步态轨迹生成器 — 每周期在线重规划
     *
     * 从当前足端位置平滑过渡到目标站立位置，分两阶段：
     *   - TRANSITION：五次多项式 + Z 轴拱形，每周期基于最新状态重规划
     *   - HOLD：到达目标后保持不动
     *
     * 与 WalkTrajectory 一致：generate() 初始化，get() 每控制周期在线重规划。
     */
    class StandTrajectory : public BaseTrajectory {
      public:
        explicit StandTrajectory(const std::string &configPath, rclcpp::Duration period);

        /** @brief 设置站立目标位置（机体坐标系） */
        bool setTarget(float tipX, float tipZ);

        /**
         * @brief 初始化过渡轨迹（以当前足端为起点）
         * @param currentTip 当前足端状态（机体坐标系）
         * @param now        当前时间
         * @return 初始轨迹点
         */
        TrajectoryPoint generate(const Body &currentTip, const rclcpp::Time now);

        /**
         * @brief 每控制周期获取期望轨迹点（内部在线重规划）
         * @param currentTime 当前 ROS 时间
         * @param currentTip  当前足端反馈（可空，则用轨迹内推状态）
         * @return 本周期期望的足端状态
         */
        TrajectoryPoint get(const rclcpp::Time &currentTime,
                            const Body *currentTip = nullptr) override;

        bool reloadParameters() override;

        /** @brief 过渡是否已完成（进入 HOLD 阶段） */
        bool isHolding() const {
            return phase_ == HOLD;
        }

        /// 步态参数（从 YAML 加载）
        struct parameters_t {
            float standKp = 2.0f;
            float standKd = 0.01f;
            float transitionKp = 2.0f;
            float transitionKd = 0.01f;
            float transitionVelMax = 500.0f;
            float reachThresholdSQ = 0.0001f; ///< 到达判定阈值² (mm²)
            float stepHeightFactor = 0.1f;    ///< 步高 = 水平位移 × 此因子
            float transitionTime = 5.0f;      ///< 过渡总时间 (s)
            float minReplanTime = 0.02f;      ///< 最小重规划时间阈值 (s)
        };

      private:
        enum Phase { TRANSITION, HOLD };

        /// Z 轴独立光滑弧线（复用 SwingTrajectory 的 smoothstep + 拱形）
        float smoothArcZ(float phase, float z0, float zT, float H) const;

        parameters_t parameters_;
        Eigen::Vector2f targetTipPos_;

        Phase phase_ = TRANSITION;

        // 过渡阶段状态
        float t_elapsed_ = 0.0f; ///< 已耗时 (s)
        float T_transition_;     ///< 名义过渡时间 (s)
        float z_start_;          ///< 起始 Z (mm)
        float stepHeight_;       ///< 步高 (mm)

        // 上次重规划的五次多项式系数
        Eigen::Matrix<float, 6, 1> coeff_x_;
        Eigen::Matrix<float, 6, 1> coeff_z_;

        TrajectoryPoint target_;
        rclcpp::Time lastUpdateTime_;
        float lastTipX_ = 0.0f;

        rclcpp::Logger logger_ = rclcpp::get_logger("standTrajectory");
    };

} // namespace Dog8Dof
