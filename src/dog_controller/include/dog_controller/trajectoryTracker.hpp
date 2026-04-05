#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

#include "dog_controller/gaitManager.hpp"
#include "dog_msg/msg/trajectory_command.hpp"

namespace Dog8Dof {
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
        realtime_tools::RealtimeThreadSafeBox<dog_msg::msg::TrajectoryCommand>
            commandBox_;
        rclcpp::Subscription<dog_msg::msg::TrajectoryCommand>::SharedPtr commandSub_;
        std::atomic_bool newCmd_{false};
        std::string commandTopic_;

        float generateDuration_;
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

        bool cacheInterfaceIndices();
        bool setJointCommands(size_t legIndex, const Motor &target);
        bool setAllJointCommands();
    };
} // namespace Dog8Dof