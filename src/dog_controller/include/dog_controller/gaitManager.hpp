#pragma once

#include "dog_controller/leg.hpp"
#include "dog_trajectory/jumpTrajectory.hpp"
#include "dog_trajectory/standTrajectory.hpp"
#include "dog_trajectory/walkTrajectory.hpp"

namespace Dog8Dof {

    /**
     * @brief 统一步态管理器 — 在线重规划接口
     *
     * 每控制周期调用 update()，内部：
     *   1. 从电机状态更新腿模型 → currentTip_
     *   2. 调用 currentTrajectory_->get(now, &currentTip_) 在线重规划并获取期望足端
     *   3. 逆运动学 + PD 增益下发到腿模型
     *   4. 自动检测周期结束并衔接下一周期（walk 步态自循环）
     */
    class GaitManager {
      public:
        GaitManager(rclcpp::Duration period,

                    std::string walkConfig,
                    std::string standConfig,
                    std::string jumpConfig,

                    float upperLength = Const::UPPER_LENGTH,
                    float lowerLength = Const::LOWER_LENGTH,
                    float tipLength = 0.0f);

        // ================================================================
        //  步态切换
        // ================================================================

        /**
         * @brief 切换到步行步态
         * @param stepLength  步长 (mm)，正值为前进
         * @param stepHeight  步高 (mm)
         * @param baseHeight  基座高度 (mm)
         * @param totalTime   完整步态周期时间 (s)
         * @param phaseOffset 相位偏移 [0,1)
         */
        bool setWalkGait(float stepLength,
                         float stepHeight,
                         float baseHeight,
                         float totalTime,
                         float phaseOffset = 0.0f);

        /**
         * @brief 切换到站立步态
         * @param tipX 目标足端 X (mm, 机体系)
         * @param tipZ 目标足端 Z (mm, 机体系)
         * @param useInitTransition 是否使用初始化站起的缓冲时间（默认false）
         */
        bool setStandGait(float tipX, float tipZ, bool useInitTransition = false);

        /**
         * @brief 切换到跳跃步态
         * @param parID 跳跃配置参数索引（对应 YAML 中的 key，由上位机通过
         * GaitTwist.step_height 传入）
         * @note 跳跃步态一旦进入不可打断，直到完成才回落 Stand
         */
        bool setJumpGait(uint8_t parID);

        /**
         * @brief 查询是否正在执行跳跃（用于上层阻止打断）
         */
        bool isJumping() const;

        // ================================================================
        //  初始化 & 周期更新
        // ================================================================

        /**
         * @brief 初始化腿模型（首次建立 currentTip_）
         * @return 是否成功
         */
        bool init(const Eigen::Vector2f &pos,
                  const Eigen::Vector2f &vel,
                  const Eigen::Vector2f &tau);

        /**
         * @brief 每控制周期更新（在线重规划 + 逆运动学）
         * @return 是否成功
         */
        bool update(const Eigen::Vector2f &pos,
                    const Eigen::Vector2f &vel,
                    const Eigen::Vector2f &tau);

        /**
         * @brief 获取当前步态是否完成（stand 到达目标；walk 永不完成）
         */
        bool isFinished() const;

        // ================================================================
        //  电机输出
        // ================================================================

        Motor getFrontTarget() const;
        Motor getRearTarget() const;

      private:
        enum class GaitType { STAND, WALK, JUMP };

        rclcpp::Duration period_;
        rclcpp::Time current_time_{0, 0, RCL_ROS_TIME};
        Leg2Dof leg_;
        Body currentTip_;
        TrajectoryPoint targetPoint_;

        std::shared_ptr<WalkTrajectory> walkTrajectory_;
        std::shared_ptr<StandTrajectory> standTrajectory_;
        std::shared_ptr<JumpTrajectory> jumpTrajectory_;

        /// 当前活跃轨迹（Walk / Stand / Jump）
        std::shared_ptr<BaseTrajectory> currentTrajectory_;

        /// 上一个步态类型（用于确定切换时的缓冲时间）
        GaitType previousGaitType_ = GaitType::STAND;

        rclcpp::Logger logger_ = rclcpp::get_logger("GaitManager");
    };

} // namespace Dog8Dof
