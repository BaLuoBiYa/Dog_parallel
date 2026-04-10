#pragma once
#include "goM8010.hpp"
#include "unitree_actuator_sdk/serialPort/SerialPort.h"
#include <cstdint>
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <map>
#include <memory>
#include <string>

namespace Dog8Dof {
    class BaseInterface : public hardware_interface::SystemInterface {
      public:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;

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
        bool cacheInterfaceHandles();

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

        std::array<hardware_interface::StateInterface::SharedPtr, 8> stPos_{};
        std::array<hardware_interface::StateInterface::SharedPtr, 8> stVel_{};
        std::array<hardware_interface::StateInterface::SharedPtr, 8> stTau_{};

        std::array<hardware_interface::CommandInterface::SharedPtr, 8> cmdPos_{};
        std::array<hardware_interface::CommandInterface::SharedPtr, 8> cmdVel_{};
        std::array<hardware_interface::CommandInterface::SharedPtr, 8> cmdTau_{};
        std::array<hardware_interface::CommandInterface::SharedPtr, 8> cmdKp_{};
        std::array<hardware_interface::CommandInterface::SharedPtr, 8> cmdKd_{};
    };
} // namespace Dog8Dof