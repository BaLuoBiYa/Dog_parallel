#include "dog_hardware/baseSingle.hpp"
#include "unitree_actuator_sdk/serialPort/SerialPort.h"

#include <cstddef>
#include <hardware_interface/system_interface.hpp>
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

        configPath_ = params.hardware_info.hardware_parameters.find("configPath")->second;
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

    std::vector<hardware_interface::StateInterface>
    SingleLegInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.reserve(2 * 3);

        for (size_t i = 0; i < 2; ++i) {
            state_interfaces.emplace_back(jointNames_[i], "position", &stPos_[i]);
            state_interfaces.emplace_back(jointNames_[i], "velocity", &stVel_[i]);
            state_interfaces.emplace_back(jointNames_[i], "effort", &stTau_[i]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    SingleLegInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.reserve(2 * 5);

        for (size_t i = 0; i < 2; ++i) {
            command_interfaces.emplace_back(jointNames_[i], "position", &cmdPos_[i]);
            command_interfaces.emplace_back(jointNames_[i], "velocity", &cmdVel_[i]);
            command_interfaces.emplace_back(jointNames_[i], "effort", &cmdTau_[i]);
            command_interfaces.emplace_back(jointNames_[i], "kp", &cmdKp_[i]);
            command_interfaces.emplace_back(jointNames_[i], "kd", &cmdKd_[i]);
        }

        return command_interfaces;
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

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    SingleLegInterface::on_deactivate(const rclcpp_lifecycle::State &pre) {
        (void)pre;
        for (size_t i = 0; i < 2; ++i) {
            motor_[i].damp(0.1f);
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
            stPos_[i] = static_cast<double>(motor_[i].getPos());
            stVel_[i] = static_cast<double>(motor_[i].getVel());
            stTau_[i] = static_cast<double>(motor_[i].getTor());

            goM8010Interface::ErrorCode err = motor_[i].getError();
            if (err == goM8010Interface::ErrorCode::OK) {
                continue;
            } else {
                std::string errStr;
                switch (err) {
                case goM8010Interface::ErrorCode::OK:
                    errStr = "OK";
                    break;
                case goM8010Interface::ErrorCode::OVER_HEAT:
                    errStr = "Over Heat";
                    break;
                case goM8010Interface::ErrorCode::OVER_CURRENT:
                    errStr = "Over Current";
                    break;
                case goM8010Interface::ErrorCode::OVER_VOLTAGE:
                    errStr = "Over Voltage";
                    break;
                case goM8010Interface::ErrorCode::ENCODER_ERROR:
                    errStr = "Encoder Error";
                    break;
                case goM8010Interface::ErrorCode::UNDER_VOLTAGE:
                    errStr = "Under Voltage";
                    break;
                case goM8010Interface::ErrorCode::OVER_HEAT_COIL:
                    errStr = "Over Heat Coil";
                    break;
                default:
                    errStr = "Unknown Error";
                    break;
                }
                RCLCPP_ERROR_STREAM(get_logger(),
                                    "Motor " << jointNames_[i] << " " << errStr);
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
    SingleLegInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;

        for (size_t i = 0; i < 2; ++i) {
            const float pos = static_cast<float>(cmdPos_[i]);
            const float vel = static_cast<float>(cmdVel_[i]);
            const float tor = static_cast<float>(cmdTau_[i]);
            const float kp = static_cast<float>(cmdKp_[i]);
            const float kd = static_cast<float>(cmdKd_[i]);

            if (!motor_[i].setHybrid(tor, pos, vel, kp, kd)) {
                RCLCPP_ERROR_STREAM(get_logger(), "Time out on " << jointNames_[i]);
                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }

} // namespace Dog8Dof

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(Dog8Dof::SingleLegInterface, hardware_interface::SystemInterface);