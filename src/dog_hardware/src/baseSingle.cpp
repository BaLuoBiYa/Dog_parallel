#include "dog_hardware/baseSingle.hpp"
#include "unitree_actuator_sdk/serialPort/SerialPort.h"
#include <cstddef>
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>

namespace Dog8Dof {

    hardware_interface::CallbackReturn SingleLegInterface::on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) {
        if (hardware_interface::SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        configPath_ = info_.hardware_parameters["configPath"];
        // trim left
        configPath_.erase(
            configPath_.begin(),
            std::find_if(configPath_.begin(), configPath_.end(), [](unsigned char ch) {
                return !std::isspace(ch);
            }));
        // trim right
        configPath_.erase(std::find_if(configPath_.rbegin(),
                                       configPath_.rend(),
                                       [](unsigned char ch) { return !std::isspace(ch); })
                              .base(),
                          configPath_.end());

        try {
            const YAML::Node root = YAML::LoadFile(configPath_);

            portName_ = root["RS485_Port"].as<std::string>();

            for (size_t i = 0; i < 2; ++i) {
                const YAML::Node joint_node = root[jointNames_[i]];
                if (!joint_node) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Missing joint node: %s",
                                 jointNames_[i].c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                const int id = joint_node["id"].as<int>();
                const float offset = joint_node["offset"].as<float>();

                jointMapping_[jointNames_[i]] = static_cast<uint8_t>(id);
                zeroOffsetByIndex_[i] = offset;
            }

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "on_init exception: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    SingleLegInterface::on_configure(const rclcpp_lifecycle::State &pre) {
        (void)pre;

        port_.reset();
        motor_.clear();

        try {
            port_ = std::make_shared<SerialPort>(portName_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "RS485 connection failed: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        for (size_t i = 0; i < 2; ++i) {
            try {
                motor_.push_back(goM8010Interface(jointMapping_[jointNames_[i]], port_));
                motor_[i].setZero(zeroOffsetByIndex_[i]);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(),
                             "Motor %s connection failed: %s",
                             jointNames_[i].c_str(),
                             e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    SingleLegInterface::on_activate(const rclcpp_lifecycle::State &pre) {
        (void)pre;

        if (!cacheInterfaceHandles()) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    SingleLegInterface::on_deactivate(const rclcpp_lifecycle::State &pre) {
        (void)pre;
        for (size_t i = 0; i < 2; ++i) {
            motor_[i].zeroTor();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    SingleLegInterface::on_error(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        for (size_t i = 0; i < 2; ++i) {
            motor_[i].emergencyStop();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
    SingleLegInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;

        for (size_t i = 0; i < 2; ++i) {
            if (!set_state<float>(stPos_[i], motor_[i].getPos(), false) ||
                !set_state<float>(stVel_[i], motor_[i].getVel(), false) ||
                !set_state<float>(stTau_[i], motor_[i].getTor(), false)) {
                RCLCPP_ERROR_STREAM(get_logger(),
                                    "Set state failed on " << jointNames_[i]);
                return hardware_interface::return_type::ERROR;
            }

            if (motor_[i].getError() != "OK") {
                RCLCPP_ERROR_STREAM(
                    get_logger(),
                    "Motor " << jointNames_[i] << " " << motor_[i].getError());
                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
    SingleLegInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;

        float pos = 0.0f, vel = 0.0f, tor = 0.0f, kp = 0.0f, kd = 0.0f;
        for (size_t i = 0; i < 2; ++i) {
            if (!get_command<float>(cmdPos_[i], pos, false) ||
                !get_command<float>(cmdVel_[i], vel, false) ||
                !get_command<float>(cmdTau_[i], tor, false) ||
                !get_command<float>(cmdKp_[i], kp, false) ||
                !get_command<float>(cmdKd_[i], kd, false)) {
                RCLCPP_ERROR_STREAM(get_logger(),
                                    "Get command failed on " << jointNames_[i]);
                return hardware_interface::return_type::ERROR;
            }

            if (!motor_[i].setHybrid(tor, pos, vel, kp, kd)) {
                RCLCPP_ERROR_STREAM(get_logger(), "Time out on " << jointNames_[i]);
                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }

    bool SingleLegInterface::cacheInterfaceHandles() {
        try {
            for (size_t i = 0; i < 2; ++i) {
                stPos_[i] = get_state_interface_handle(jointNames_[i] + "/position");
                stVel_[i] = get_state_interface_handle(jointNames_[i] + "/velocity");
                stTau_[i] = get_state_interface_handle(jointNames_[i] + "/effort");

                cmdPos_[i] = get_command_interface_handle(jointNames_[i] + "/position");
                cmdVel_[i] = get_command_interface_handle(jointNames_[i] + "/velocity");
                cmdTau_[i] = get_command_interface_handle(jointNames_[i] + "/effort");
                cmdKp_[i] = get_command_interface_handle(jointNames_[i] + "/kp");
                cmdKd_[i] = get_command_interface_handle(jointNames_[i] + "/kd");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(
                this->get_logger(), "Cache interface handles failed: %s", e.what());
            return false;
        }
        return true;
    }

} // namespace Dog8Dof

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(Dog8Dof::SingleLegInterface, hardware_interface::SystemInterface);