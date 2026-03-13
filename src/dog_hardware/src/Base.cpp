#include "dog_hardware/Base.hpp"
#include "unitree_actuator_sdk/serialPort/SerialPort.h"
#include <cstddef>
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <memory>
#include <rclcpp/logging.hpp>

namespace Dog8Dof {
    hardware_interface::CallbackReturn
    BaseInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
        if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        uint8_t jointID;
        for (size_t i = 0; i < 8; i++) {
            jointID = std::stoi(info_.hardware_parameters[jointNames_[i] + "_ID"]);
            jointMapping_.insert(std::make_pair(jointNames_[i], jointID));
        }
        // 建立关节名称-电机ID映射

        portName_ = info_.hardware_parameters["RS485_Port"];

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BaseInterface::on_configure(const rclcpp_lifecycle::State &pre) {
        (void)pre;
        try {
            port_ = std::make_shared<SerialPort>(portName_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "RS485 Connection Failed: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        // 注册串口

        for (size_t i = 0; i < 8; i++) {
            try {
                motor_[i] = std::make_unique<goM8010Interface>(jointMapping_[jointNames_[i]], port_);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Motor Connection Failed: %s", e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        // 注册电机

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BaseInterface::on_activate(const rclcpp_lifecycle::State &pre) {
        (void)pre;
        for (size_t i = 0; i < 8; i++) {
            motor_[i]->zeroTor();
            motor_[i]->setZero();
            // 设置当前位置为零位
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BaseInterface::on_error(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        motor_[0]->emergencyStop();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type BaseInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;
        for (size_t i = 0; i < 8; i++) {
            set_state<float>(jointNames_[i] + "/position", motor_[i]->getPos());
            set_state<float>(jointNames_[i] + "/velocity", motor_[i]->getVel());
            set_state<float>(jointNames_[i] + "/effort", motor_[i]->getTor());
            if (motor_[i]->getError() != "OK") {
                RCLCPP_ERROR_STREAM(get_logger(), "Motor" << jointNames_[i] << motor_[i]->getError());
                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BaseInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;

        float pos, vel, tor, kp, kd;
        for (size_t i = 0; i < 8; i++) {
            pos = get_command<float>(jointNames_[i] + "/position");
            vel = get_command<float>(jointNames_[i] + "/velocity");
            tor = get_command<float>(jointNames_[i] + "/effort");
            kp = get_command<float>(jointNames_[i] + "/kp");
            kd = get_command<float>(jointNames_[i] + "/kd");
            if (!motor_[i]->setHybrid(tor, pos, vel, kp, kd)) {
                RCLCPP_ERROR_STREAM(get_logger(), "Time out on " << jointNames_[i]);
                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }

} // namespace Dog8Dof

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(Dog8Dof::BaseInterface, hardware_interface::SystemInterface);