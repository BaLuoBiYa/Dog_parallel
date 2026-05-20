#pragma once

#include "dog_hardware/serial.hpp"
#include "goM8010.hpp"
#include <array>
#include <cstdint>
#include <hardware_interface/system_interface.hpp>
#include <map>
#include <memory>
#include <string>

namespace Dog8Dof {

    class SingleLegInterface : public hardware_interface::SystemInterface {
      public:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;

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
        // 默认使用左前腿的两个关节，必要时替换为目标关节名
        std::string jointNames_[2] = {"LF_Front", "LF_Rear"};
        std::string portName_;
        std::shared_ptr<Dog8Dof::SerialPort> port_;

        std::vector<goM8010Interface> motor_;
        std::string configPath_;
        std::array<float, 2> zeroOffsetByIndex_{};

        std::array<double, 2> stPos_{};
        std::array<double, 2> stVel_{};
        std::array<double, 2> stTau_{};

        std::array<double, 2> cmdPos_{};
        std::array<double, 2> cmdVel_{};
        std::array<double, 2> cmdTau_{};
        std::array<double, 2> cmdKp_{};
        std::array<double, 2> cmdKd_{};
    };

} // namespace Dog8Dof