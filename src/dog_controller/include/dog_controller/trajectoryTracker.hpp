#pragma once
#include <controller_interface/controller_interface.hpp>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>

#include "dog_controller/gaitManager.hpp"
#include "dog_controller/gaitTwistConverter.hpp"
#include "dog_controller/stateMonitor.hpp"
#include "dog_controller/threadSafeMessageBox.hpp"
#include "dog_msg/msg/gait_twist.hpp"
#include "dog_msg/msg/system_state.hpp"

namespace Dog8Dof {

    namespace {
        /** 四腿步态相位偏移（对角小跑步态） */
        constexpr std::array<float, 4> kWalkPhaseOffset = {
            0.5f, // LF
            0.0f, // LR
            0.0f, // RF
            0.5f  // RR
        };
    } // namespace

    /**
     * @brief ROS2 控制器 — 基于在线重规划的步态轨迹追踪
     *
     * 架构：
     *   - 4 个 GaitManager（每条腿独立），每周期调用 update()：
     *       电机反馈 → 腿模型 → get(now, &currentTip) 在线重规划 → 逆运动学 → 关节命令
     *   - Walk 步态在 GaitManager 内部自循环，无需控制器层管理周期衔接
     *   - Stand 步态到达目标后自动 HOLD
     */
    class TrajectoryTracker : public controller_interface::ControllerInterface {
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
        uint8_t maxfailure_, failureCounter_ = 0;

        // ---- 步态管理器（每条腿独立） ----
        std::unique_ptr<GaitManager> LF_, LR_, RF_, RR_;

        // ---- 关节名 ----
        std::array<std::string, 8> jointNames_{"LF_Front",
                                               "LF_Rear",
                                               "RF_Front",
                                               "RF_Rear",
                                               "LR_Front",
                                               "LR_Rear",
                                               "RR_Front",
                                               "RR_Rear"};

        // ---- 硬件接口索引 ----
        struct JointStateIndex {
            size_t pos;
            size_t vel;
            size_t eff;
            size_t tmp;
        };
        struct JointCmdIndex {
            size_t pos;
            size_t vel;
            size_t eff;
            size_t kp;
            size_t kd;
        };
        std::array<JointStateIndex, 8> stateIdx_{};
        std::array<JointCmdIndex, 8> cmdIdx_{};

        // ---- 系统监控 ----
        StateMonitor monitor_{jointNames_};
        std::string monitorTopic_;
        rclcpp::Publisher<dog_msg::msg::SystemState>::SharedPtr monitorPub_;
        size_t monitorPubCounter_ = 0;
        static constexpr size_t kMonitorPubDecimation = 10;

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
        void registerMonitorPublisher();
        void publishMonitorState();
        bool setJointCommands(size_t legIndex, const Motor &target);
        bool setAllJointCommands();
        float getPos(size_t i);
        float getVel(size_t i);
        float getTau(size_t i);
        float getTemp(size_t i);
    };

} // namespace Dog8Dof