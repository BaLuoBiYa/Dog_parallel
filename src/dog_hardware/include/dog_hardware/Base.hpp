#pragma once
#include "goM8010.hpp"
#include "unitree_actuator_sdk/serialPort/SerialPort.h"
#include <cstdint>
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <map>
#include <memory>
#include <string>

namespace Dog_Hardware {
    class BaseInterface : public hardware_interface::SystemInterface {
      public:
        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

      private:
        std::map<std::string, uint8_t> jointMapping_;
        std::string jointNames_[8] = {"LF_HFE", "LF_KFE", "LH_HFE", "LH_KFE", "RF_HFE", "RF_KFE", "RH_HFE", "RH_KFE"};
        std::string portName_;
        std::shared_ptr<SerialPort> port_;

        std::unique_ptr<goM8010Interface> motor_[8];
    };
} // namespace Dog_Hardware