#include "dog_hardware/base.hpp"
#include "dog_hardware/serial.hpp"

#include <cstddef>
#include <fcntl.h>
#include <hardware_interface/system_interface.hpp>
#include <linux/serial.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace Dog8Dof {

    hardware_interface::CallbackReturn BaseInterface::on_init(
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

        retryCountL_ = 0;
        retryCountR_ = 0;

        try {
            const YAML::Node root = YAML::LoadFile(configPath_);

            // 两路独立 485 端口
            portNameLeft_ = root["RS485_Port_Left"].as<std::string>();
            portNameRight_ = root["RS485_Port_Right"].as<std::string>();
            maxRetry_ = root["maxRetry"].as<size_t>(3);
            deactiveDamp_ = root["deactiveDamp"].as<float>(0.1);
            deactiveDampTime_ = root["deactiveDampTime"].as<long>(2);

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
            state_interfaces.emplace_back(jointNames_[i], "temperature", &stTemp_[i]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    BaseInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.reserve(8 * 5);

        for (size_t i = 0; i < 8; ++i) {
            command_interfaces.emplace_back(jointNames_[i], "position", &cmdPos_[i]);
            command_interfaces.emplace_back(jointNames_[i], "velocity", &cmdVel_[i]);
            command_interfaces.emplace_back(jointNames_[i], "effort", &cmdTau_[i]);
            command_interfaces.emplace_back(jointNames_[i], "kp", &cmdKp_[i]);
            command_interfaces.emplace_back(jointNames_[i], "kd", &cmdKd_[i]);
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn
    BaseInterface::on_configure(const rclcpp_lifecycle::State &pre) {
        (void)pre;

        portLeft_.reset();
        portRight_.reset();
        motor_.clear();
        // 清空智能指针

        try {
            portLeft_ = std::make_shared<Dog8Dof::SerialPort>(portNameLeft_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(
                this->get_logger(), "RS485 left port connection failed: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        try {
            portRight_ = std::make_shared<Dog8Dof::SerialPort>(portNameRight_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(
                this->get_logger(), "RS485 right port connection failed: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        for (size_t i = 0; i < 8; ++i) {
            try {
                // 左组电机使用左端口，右组电机使用右端口
                std::shared_ptr<Dog8Dof::SerialPort> port =
                    (i == LEFT_MOTOR_INDICES[0] || i == LEFT_MOTOR_INDICES[1] ||
                     i == LEFT_MOTOR_INDICES[2] || i == LEFT_MOTOR_INDICES[3])
                        ? portLeft_
                        : portRight_;
                motor_.push_back(goM8010Interface(jointMapping_[jointNames_[i]], port));
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
    BaseInterface::on_activate(const rclcpp_lifecycle::State &pre) {
        (void)pre;

        for (size_t i = 0; i < 8; ++i) {
            if (!motor_[i].release()) {
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    BaseInterface::on_deactivate(const rclcpp_lifecycle::State &pre) {
        (void)pre;
        for (size_t i = 0; i < 8; ++i) {
            motor_[i].damp(deactiveDamp_);
        }
        rclcpp::sleep_for(std::chrono::seconds(deactiveDampTime_));
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
        bool hasError = false;

        for (size_t i = 0; i < 8; ++i) {
            stPos_[i] = static_cast<double>(motor_[i].getPos());
            stVel_[i] = static_cast<double>(motor_[i].getVel());
            stTau_[i] = static_cast<double>(motor_[i].getTor());
            stTemp_[i] = static_cast<double>(motor_[i].getTemp());

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
                retryCountL_++;
                hasError = true;
            }
        }

        if (retryCountL_ + retryCountR_ > maxRetry_) {
            return hardware_interface::return_type::ERROR;
        }

        if (!hasError) {
            retryCountL_ = 0;
            retryCountR_ = 0;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BaseInterface::write(const rclcpp::Time &time,
                                                         const rclcpp::Duration &period) {
        (void)time;
        (void)period;

        bool leftOk = true;
        bool rightOk = true;

        // 左右两路 485 并行通信
        std::thread leftThread([&]() {
            for (size_t idx : LEFT_MOTOR_INDICES) {
                const float pos = static_cast<float>(cmdPos_[idx]);
                const float vel = static_cast<float>(cmdVel_[idx]);
                const float tor = static_cast<float>(cmdTau_[idx]);
                const float kp = static_cast<float>(cmdKp_[idx]);
                const float kd = static_cast<float>(cmdKd_[idx]);

                if (!motor_[idx].setHybrid(tor, pos, vel, kp, kd)) {
                    RCLCPP_ERROR_STREAM(get_logger(), "Time out on " << jointNames_[idx]);
                    leftOk = false;
                }
            }
        });

        std::thread rightThread([&]() {
            for (size_t idx : RIGHT_MOTOR_INDICES) {
                const float pos = static_cast<float>(cmdPos_[idx]);
                const float vel = static_cast<float>(cmdVel_[idx]);
                const float tor = static_cast<float>(cmdTau_[idx]);
                const float kp = static_cast<float>(cmdKp_[idx]);
                const float kd = static_cast<float>(cmdKd_[idx]);

                if (!motor_[idx].setHybrid(tor, pos, vel, kp, kd)) {
                    RCLCPP_ERROR_STREAM(get_logger(), "Time out on " << jointNames_[idx]);
                    rightOk = false;
                }
            }
        });

        leftThread.join();
        rightThread.join();

        if (leftOk && rightOk) {
            retryCountL_ = 0;
            retryCountR_ = 0;
            return hardware_interface::return_type::OK;
        }

        if (leftOk + leftOk >= maxRetry_) {
            return hardware_interface::return_type::OK;
        }

        return hardware_interface::return_type::OK;
    }

} // namespace Dog8Dof

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(Dog8Dof::BaseInterface, hardware_interface::SystemInterface);