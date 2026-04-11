#include "dog_hardware/base.hpp"
#include "unitree_actuator_sdk/serialPort/SerialPort.h"

#include <cstddef>
#include <hardware_interface/system_interface.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>

namespace Dog8Dof {

    hardware_interface::CallbackReturn BaseInterface::on_init(
        const hardware_interface::HardwareInfo &info) {
        if (hardware_interface::SystemInterface::on_init(info) !=
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

            for (size_t i = 0; i < 8; ++i) {
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
            // 从YAML文件读取参数

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "on_init exception: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    BaseInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.reserve(8 * 3);

        for (size_t i = 0; i < 8; ++i) {
            state_interfaces.emplace_back(
                jointNames_[i], hardware_interface::HW_IF_POSITION, &stPos_[i]);
            state_interfaces.emplace_back(
                jointNames_[i], hardware_interface::HW_IF_VELOCITY, &stVel_[i]);
            state_interfaces.emplace_back(
                jointNames_[i], hardware_interface::HW_IF_EFFORT, &stTau_[i]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    BaseInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.reserve(8 * 5);

        for (size_t i = 0; i < 8; ++i) {
            command_interfaces.emplace_back(
                jointNames_[i], hardware_interface::HW_IF_POSITION, &cmdPos_[i]);
            command_interfaces.emplace_back(
                jointNames_[i], hardware_interface::HW_IF_VELOCITY, &cmdVel_[i]);
            command_interfaces.emplace_back(
                jointNames_[i], hardware_interface::HW_IF_EFFORT, &cmdTau_[i]);
            command_interfaces.emplace_back(jointNames_[i], "kp", &cmdKp_[i]);
            command_interfaces.emplace_back(jointNames_[i], "kd", &cmdKd_[i]);
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn
    BaseInterface::on_configure(const rclcpp_lifecycle::State &pre) {
        (void)pre;

        port_.reset();
        motor_.clear();
        // 清空智能指针

        try {
            port_ = std::make_shared<SerialPort>(portName_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "RS485 connection failed: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        // 创建串口通信指针

        for (size_t i = 0; i < 8; ++i) {
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
        // 创建电机指针

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    BaseInterface::on_activate(const rclcpp_lifecycle::State &pre) {
        (void)pre;

        for (size_t i = 0; i < 8; ++i) {
            bool res;
            res = motor_[i].release();
            if (!res) {
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        // 解除电机锁定

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    BaseInterface::on_deactivate(const rclcpp_lifecycle::State &pre) {
        (void)pre;
        for (size_t i = 0; i < 8; ++i) {
            motor_[i].zeroTor();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    BaseInterface::on_error(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        for (size_t i = 0; i < 8; ++i) {
            motor_[i].emergencyStop();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type BaseInterface::read(const rclcpp::Time &time,
                                                        const rclcpp::Duration &period) {
        (void)time;
        (void)period;

        for (size_t i = 0; i < 8; ++i) {
            stPos_[i] = static_cast<double>(motor_[i].getPos());
            stVel_[i] = static_cast<double>(motor_[i].getVel());
            stTau_[i] = static_cast<double>(motor_[i].getTor());

            if (motor_[i].getError() != "OK") {
                RCLCPP_ERROR_STREAM(
                    get_logger(),
                    "Motor " << jointNames_[i] << " " << motor_[i].getError());
                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BaseInterface::write(const rclcpp::Time &time,
                                                         const rclcpp::Duration &period) {
        (void)time;
        (void)period;

        for (size_t i = 0; i < 8; ++i) {
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
PLUGINLIB_EXPORT_CLASS(Dog8Dof::BaseInterface, hardware_interface::SystemInterface);