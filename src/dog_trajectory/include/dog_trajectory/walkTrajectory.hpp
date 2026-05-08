#pragma once
#include "dog_trajectory/base.hpp"

namespace Dog8Dof {

    /**
     * @brief 步行步态轨迹生成器
     *
     * X 轴（前进方向）与 Z 轴（抬腿方向）完全解耦：
     *   - X：五次多项式，每周期基于最新状态在线重规划，保证 C² 连续
     *   - Z：独立光滑弧线（quintic smoothstep + 余弦拱形），保证零速起落
     */
    class WalkTrajectory : public BaseTrajectory {
      public:
        explicit WalkTrajectory(const std::string &configPath, rclcpp::Duration period);
        enum GaitPhase { SWING, STANCE };

        /**
         * @brief 设置步行步态的目标
         * @param stepLength  步长 (mm)，正值为前进
         * @param stepHeight  步高 (mm)
         * @param baseHeight  基座高度 / 目标站立高度 (mm)
         * @param totalTime   完整步态周期时间 (s)
         * @param phaseOffset 步态相位偏移，范围 [0, 1)，默认为 0
         */
        bool setTarget(float stepLength,
                       float stepHeight,
                       float baseHeight,
                       float totalTime,
                       float phaseOffset = 0.0f);
        TrajectoryPoint generate(const Body &currentTip, const rclcpp::Time now);
        bool reloadParameters() override;

        /**
         * @brief 获取当前时刻的期望轨迹点（每控制周期调用，内部在线重规划）
         * @param currentTime 当前 ROS 时间
         * @param currentTip  当前足端传感器反馈（可空，则用轨迹内推状态）
         * @return 本周期期望的足端状态（位置、速度、PD 增益）
         */
        TrajectoryPoint get(const rclcpp::Time &currentTime,
                            const Body *currentTip = nullptr) override;

        /** @brief 获取摆动相占空比 */
        float swingDuty() const {
            return parameters_.swingDutyCycle;
        }

        /// 步态参数（从 YAML 加载）
        struct parameters_t {
            float walkKp = 3.0f;
            float walkKd = 0.05f;
            float swingDutyCycle = 0.5f; // 摆动相占空比
            float minReplanTime = 0.02f; // 最小重规划时间阈值 (s)，如 20ms
            float lowPassAlpha = 0.3f;   // 落脚点低通滤波系数
            float horizontalPushGain = 0.02f; // Raibert 落脚点水平增益
            float verticalPushGain = 0.02f;   // Raibert 落脚点垂直增益（预留）
            float stanceTimeMin = 0.05f;      // 最小支撑时间 (s)
            float stanceTimeMax = 1.0f;       // 最大支撑时间 (s)
        };

        friend class StanceTrajectory;
        friend class SwingTrajectory;

      protected:
        // ======================== WalkTrajectory 成员 ========================
        parameters_t parameters_;
        float stepLength_ = 0.0f, stepHeight_ = 0.0f, baseHeight_ = 0.0f;
        float phaseOffset_ = 0.0f;
        float swingTime_ = 0.5f, stanceTime_ = 0.5f;
        GaitPhase gaitPhase_ = SWING;

        TrajectoryPoint target_;
        rclcpp::Time lastUpdateTime_;
        rclcpp::Time gaitEpoch_; ///< 步态纪元时刻，用于锁定全局相位
        float vDes_ = 0.0f;     ///< 当前期望前进速度 (mm/s)
        float lastTipX_ = 0.0f; ///< 上一周期足端 X（用于速度估计）

        rclcpp::Logger logger_ = rclcpp::get_logger("walkTrajectory");

      private:
        // ======================== 摆动相轨迹 ========================
        /**
         * @brief 摆动相轨迹：管理从离地到落地的全过程
         *
         * 每周期调用 update()，基于最新状态在线重规划 X 轴五次多项式，
         * Z 轴使用独立的光滑弧线。支持滑动落脚点（Raibert 公式 + 低通滤波）。
         */
        class SwingTrajectory {
          public:
            SwingTrajectory() = default;

            /**
             * @brief 初始化摆动相
             * @param zStart      离地时足端 Z 坐标 (mm)
             * @param zTarget     目标落地 Z 坐标 / 地形估计 (mm)
             * @param stepH       期望步高 (mm)
             * @param swingT      名义摆动时间 (s)
             * @param alpha       落脚点低通滤波系数 [0,1]
             * @param minReplan   最小重规划时间阈值 (s)
             * @param xStart      起始 X 位置 (mm)
             * @param vxStart     起始 X 速度 (mm/s)
             * @param axStart     起始 X 加速度 (mm/s²)
             * @param stepLength 期望步长 (mm)
             */
            void init(float zStart,
                      float zTarget,
                      float stepH,
                      float swingT,
                      float alpha,
                      float minReplan,
                      float xStart,
                      float vxStart,
                      float axStart,
                      float stepLength);

            /**
             * @brief 每控制周期更新，在线重规划
             * @param dt          控制周期 (s)
             * @param vDes        期望前进速度 (mm/s)，由步长/周期决定
             * @param vAct        实际前进速度估计 (mm/s)
             * @param pushGain    Raibert 水平增益
             * @param currentTip  足端传感器反馈（可空，空则从轨迹反算）
             * @param[out] xDes   期望 X 位置 (mm)
             * @param[out] zDes   期望 Z 位置 (mm)
             * @param[out] vxDes  期望 X 速度 (mm/s)
             * @param[out] vzDes  期望 Z 速度 (mm/s)
             * @param stepLength 期望步长 (mm)
             */
            void update(float dt,
                        float vDes,
                        float vAct,
                        float pushGain,
                        const Body *currentTip,
                        float &xDes,
                        float &zDes,
                        float &vxDes,
                        float &vzDes,
                        float stepLength);

            bool isLanded() const {
                return landed_;
            }

            /// 获取最终滤波落脚点 X（用于落地衔接）
            float getTargetX() const {
                return p_target_x_;
            }
            /// 获取目标落地 Z
            float getZTarget() const {
                return z_target_;
            }

          private:
            // ---- 参数 ----
            float H_;                  ///< 步高 (mm)
            float T_swing_nominal_;    ///< 名义摆动时间 (s)
            float T_min_;              ///< 最小重规划时间阈值 (s)
            float alpha_;              ///< 落脚点低通滤波系数
            float z_start_, z_target_; ///< 离地和目标落地 Z (mm)
            float step_length_;        ///< 期望步长 (mm)
            float t_elapsed_;          ///< 已耗时 (s)

            // ---- 状态 ----
            Eigen::Matrix<float, 6, 1> last_coeffs_x_; ///< 上一周期 X 轴五次多项式系数
            float p_target_x_; ///< 滤波后的目标落脚点 X (mm)
            bool landed_ = false;
            bool foothold_locked_ = false; ///< 剩余时间 < T_min 时锁定落脚点

            /// Z 轨迹：phase ∈ [0,1]
            float smoothArcZ(float phase) const;
        };

        // ======================== 支撑相轨迹 ========================
        /**
         * @brief 支撑相轨迹：足端世界静止，输出机体坐标
         *
         * 核心：落地瞬间记录落脚点在机体坐标系的位置 (x_land_body, z_land_body)。
         * 支撑期间机体向前运动，足端在世界系中固定，因此在机体系中向后运动：
         *   xDes = x_land_body - body_disp_x
         *   zDes = z_land_body
         * 每周期更新计时、累加机体位移并判断离地条件。
         */
        class StanceTrajectory {
          public:
            StanceTrajectory() = default;

            /**
             * @brief 初始化支撑相
             * @param xLand       落地 X 位置 (mm)
             * @param zLand       落地 Z 位置 (mm)
             * @param stanceT     名义支撑时间 (s)
             * @param tMin        最小支撑时间 (s)
             * @param tMax        最大支撑时间 (s)
             */
            void init(float xLand, float zLand, float stanceT, float tMin, float tMax);

            /**
             * @brief 每控制周期更新
             * @param dt          控制周期 (s)
             * @param bodyVelX    机体 X 方向速度 (mm/s)，用于累积位移
             * @param currentTip  足端传感器反馈（可空，用于提前离地判断）
             * @param[out] xDes   期望 X 位置 (mm, 机体系)
             * @param[out] zDes   期望 Z 位置 (mm, 机体系)
             */
            void update(float dt,
                        float bodyVelX,
                        const Body *currentTip,
                        float &xDes,
                        float &zDes);

            bool isLifted() const {
                return lifted_;
            }

            /// 支撑进度 [0, 1]
            float getStanceProgress() const {
                return (T_stance_nominal_ > 0.0f) ? t_stance_ / T_stance_nominal_ : 1.0f;
            }

          private:
            // 轨迹参数
            float T_stance_nominal_;
            float T_stance_min_, T_stance_max_;
            Eigen::Vector2f foot_pos_land_; // 支撑相开始时的足端位置

            // 状态
            float t_stance_;
            float body_disp_x_;                // 机体在支撑相期间的累积位移
            float early_lift_grace_s_ = 0.03f; // 提前离地静默期 (s)，避免落地瞬间误触
            bool lifted_ = false;
        };

        SwingTrajectory swing_;
        StanceTrajectory stance_;
    };

} // namespace Dog8Dof
