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

        std::string robotName_;
        std::string standProfile_, walkProfile_, jumpProfile_;
        std::unique_ptr<GaitManager> LF_, LR_, RF_, RR_;
        std::string jointNames_[8] = {"LF_Front",
                                      "LF_Rear",
                                      "RF_Front",
                                      "RF_Rear",
                                      "LR_Front",
                                      "LR_Rear",
                                      "RR_Front",
                                      "RR_Rear"};

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

        std::array<JointStateIndex, 8> stateIdx_{};
        std::array<JointCmdIndex, 8> cmdIdx_{};
        ControlMode currentMode_ = ControlMode::STAND;

        bool initialStandPending_ = false;

        // 缓存的待执行命令（收到新消息后先缓存，完成当前步态后再应用）
        bool pendingCmd_ = false;
        ControlMode pendingMode_ = ControlMode::STAND;
        std::array<std::array<float, 4>, 4> pendingParams_{};

        // 步行模式循环参数（当前周期结束后自动重启下一周期）
        std::array<std::array<float, 4>, 4> lastWalkParams_{};
        bool hasLastWalkParams_ = false;

        bool
        isSameWalkCommand(const std::array<std::array<float, 4>, 3> &walkParams) const;

        bool cacheInterfaceIndices();
        bool switchToStandMode();
        bool setJointCommands(size_t legIndex, const Motor &target);
        bool setAllJointCommands();
        bool updateAllGaitManagers();
        float getPos(size_t i);
        float getVel(size_t i);
        float getTau(size_t i);
    };
} // namespace Dog8Dof