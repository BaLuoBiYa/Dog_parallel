#pragma once
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dog_controller/gaitManager.hpp"
#include "dog_controller/gaitTwistConverter.hpp"
#include "dog_controller/threadSafeMessageBox.hpp"
#include "dog_msg/msg/gait_twist.hpp"

namespace Dog8Dof {

    /**
     * @brief ROS2 单腿控制器 — 基于在线重规划的步态轨迹追踪
     *
     * 与 TrajectoryTracker 对应，但仅管理一条腿（LF, 2 个关节）：
     *   - 电机反馈 → LF的腿模型 → get(now, &currentTip) 在线重规划 → 逆运动学 → 关节命令
     *   - Walk 步态在 GaitManager 内部自循环
     *   - Stand 步态到达目标后自动 HOLD
     */
    class TrajectoryTrackerSingle : public controller_interface::ControllerInterface {
      public:
        controller_interface::InterfaceConfiguration
        command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration
        state_interface_configuration() const override;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn
        on_error(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::return_type update(const rclcpp::Time &time,
                                                 const rclcpp::Duration &period) override;

      private:
        enum class ControlMode { STAND, WALK, JUMP };

        // ---- 参数 ----
        std::string robotName_;
        std::string standProfile_, walkProfile_, jumpProfile_;
        float updateDuration_;
        float idleTipX_, idleTipZ_;
        std::string commandTopic_;

        // ---- 步态管理器（仅 LF_ 活跃） ----
        std::unique_ptr<GaitManager> LF_;

        // ---- 关节名（单腿 2 个关节） ----
        const std::string jointNames_[2] = {"LF_Front", "LF_Rear"};

        // ---- 硬件接口索引 ----
        struct JointStateIndex {
            size_t pos;
            size_t vel;
            size_t eff;
        };
        struct JointCmdIndex {
            size_t pos;
            size_t vel;
            size_t eff;
            size_t kp;
            size_t kd;
        };
        std::array<JointStateIndex, 2> stateIdx_{};
        std::array<JointCmdIndex, 2> cmdIdx_{};

        // ---- 命令处理 ----
        ThreadSafeMessageBox<dog_msg::msg::GaitTwist> commandBox_;
        rclcpp::Subscription<dog_msg::msg::GaitTwist>::SharedPtr commandSub_;
        GaitTwistConverter converter_;
        std::atomic_bool newCmd_{false};
        ControlMode currentMode_ = ControlMode::STAND;

        // ---- 私有方法 ----
        bool cacheInterfaceIndices();
        bool updateAllGaitManagers();
        bool applyCommand(const dog_msg::msg::GaitTwist &msg);
        bool switchToStandMode();
        bool setJointCommands(size_t legIndex, const Motor &target);
        bool setAllJointCommands();
        float getPos(size_t i);
        float getVel(size_t i);
        float getTau(size_t i);

        rclcpp::Logger logger_ = rclcpp::get_logger("TrajectoryTrackerSingle");
    };

} // namespace Dog8Dof