#pragma once
#include "goM8010.hpp"
#include "unitree_actuator_sdk/serialPort/SerialPort.h"
#include <array>
#include <cstdint>
#include <hardware_interface/system_interface.hpp>
#include <map>
#include <memory>
#include <string>

namespace Dog8Dof {
    class BaseInterface : public hardware_interface::SystemInterface {
      public:
        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

        std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

        hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn
        on_error(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time,
                                              const rclcpp::Duration &period) override;

      private:
        std::map<std::string, uint8_t> jointMapping_;
        std::string jointNames_[8] = {"LF_Front",
                                      "LF_Rear",
                                      "RF_Front",
                                      "RF_Rear",
                                      "LR_Front",
                                      "LR_Rear",
                                      "RR_Front",
                                      "RR_Rear"};
        std::string portName_;
        std::shared_ptr<SerialPort> port_;

        std::vector<goM8010Interface> motor_;
        std::string configPath_;
        std::array<float, 8> zeroOffsetByIndex_{};

        std::array<double, 8> stPos_{};
        std::array<double, 8> stVel_{};
        std::array<double, 8> stTau_{};

        std::array<double, 8> cmdPos_{};
        std::array<double, 8> cmdVel_{};
        std::array<double, 8> cmdTau_{};
        std::array<double, 8> cmdKp_{};
        std::array<double, 8> cmdKd_{};
    };
} // namespace Dog8Dof