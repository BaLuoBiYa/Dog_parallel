#pragma once
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dog_controller/gaitManager.hpp"
#include "dog_controller/threadSafeMessageBox.hpp"
#include "dog_msg/msg/trajectory_command.hpp"

namespace Dog8Dof {

    namespace {
        constexpr std::array<float, 4> kWalkPhaseOffset = {
            0.0f, // LF
            0.5f, // LR
            0.5f, // RF
            0.0f  // RR
        };
    }

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

        std::string robotName_;
        std::string standProfile_, walkProfile_, jumpProfile_;

        // 保留所有四个 gait manager 成员（仅使用其中一个表示单腿场景）
        std::unique_ptr<GaitManager> LF_, LR_, RF_, RR_;

        // 仅保留单腿两个关节名称（范围裁切）
        std::string jointNames_[2] = {"LF_Front", "LF_Rear"};

        // Subscriber
        ThreadSafeMessageBox<dog_msg::msg::TrajectoryCommand> commandBox_;
        rclcpp::Subscription<dog_msg::msg::TrajectoryCommand>::SharedPtr commandSub_;
        std::atomic_bool newCmd_{false};
        std::string commandTopic_;

        float updateDuration_;
        float idleTipX_, idleTipZ_;

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

        // 将索引数组范围裁切为与关节数一致（2）
        std::array<JointStateIndex, 2> stateIdx_{};
        std::array<JointCmdIndex, 2> cmdIdx_{};
        ControlMode currentMode_ = ControlMode::STAND;

        // 保留原有接口签名（便于复用上层调用逻辑）
        bool cacheInterfaceIndices();
        bool setJointCommands(size_t legIndex, const Motor &target);
        bool setAllJointCommands();
        bool updateAllGaitManagers();

        float getPos(size_t i);
        float getVel(size_t i);
        float getTau(size_t i);
    };
} // namespace Dog8Dof