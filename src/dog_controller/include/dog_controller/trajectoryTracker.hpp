#include "dog_controller/leg.hpp"
#include "dog_trajectory/jumpTrajectoryLegacy.hpp"
#include "dog_trajectory/standTrajectory.hpp"
#include "dog_trajectory/walkTrajectory.hpp"

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

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

        // controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State
        // &previous_state) override;

        controller_interface::return_type update(const rclcpp::Time &time,
                                                 const rclcpp::Duration &period) override;

      private:
        struct {
            Leg2Dof LF;
            Leg2Dof LR;
            Leg2Dof RF;
            Leg2Dof RR;
        } legs;

        std::string robotName_;
        std::string standProfile_, walkProfile_, jumpProfile_;
        std::string jointNames_[8] = {"LF_Front",
                                      "LF_Rear",
                                      "RF_Front",
                                      "RF_Rear",
                                      "LR_Front",
                                      "LR_Rear",
                                      "RR_Front",
                                      "RR_Rear"};
    };

} // namespace Dog8Dof