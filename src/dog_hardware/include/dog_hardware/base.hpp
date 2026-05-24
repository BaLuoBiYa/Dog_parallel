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
    class BaseInterface : public hardware_interface::SystemInterface {
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
        template <typename T> class LowPassFilter {
          public:
            LowPassFilter(float alpha = 0.5f)
                : alpha_(alpha), initialized_(false), preValue_(0.0f) {
            }

            T filter(float value) {
                if (!initialized_) {
                    preValue_ = value;
                    initialized_ = true;
                    return value;
                }
                float filteredValue = alpha_ * value + (1 - alpha_) * preValue_;
                preValue_ = filteredValue;
                return static_cast<T>(filteredValue);
            }

          private:
            float alpha_;
            bool initialized_;
            float preValue_;
        };

        static constexpr std::array<size_t, 4> LEFT_MOTOR_INDICES{0, 1, 4, 5};
        static constexpr std::array<size_t, 4> RIGHT_MOTOR_INDICES{2, 3, 6, 7};

        std::map<std::string, uint8_t> jointMapping_;
        std::string jointNames_[8] = {"LF_Front",
                                      "LF_Rear",
                                      "RF_Front",
                                      "RF_Rear",
                                      "LR_Front",
                                      "LR_Rear",
                                      "RR_Front",
                                      "RR_Rear"};

        // 两路独立 485 端口
        std::string portNameLeft_;
        std::string portNameRight_;
        std::shared_ptr<Dog8Dof::SerialPort> portLeft_;
        std::shared_ptr<Dog8Dof::SerialPort> portRight_;

        std::vector<goM8010Interface> motor_;
        std::string configPath_;
        std::array<float, 8> zeroOffsetByIndex_{};

        uint8_t maxRetry_, retryCountL_, retryCountR_;
        float deactiveDamp_;
        long deactiveDampTime_;

        float alpha_;

        std::array<double, 8> stPos_{};
        std::array<double, 8> stVel_{};
        std::array<double, 8> stTau_{};
        std::array<double, 8> stTemp_{};

        std::array<LowPassFilter<double>, 8> stPosF_{};
        std::array<LowPassFilter<double>, 8> stVelF_{};
        std::array<LowPassFilter<double>, 8> stTauF_{};
        std::array<LowPassFilter<double>, 8> stTempF_{};

        std::array<double, 8> cmdPos_{};
        std::array<double, 8> cmdVel_{};
        std::array<double, 8> cmdTau_{};
        std::array<double, 8> cmdKp_{};
        std::array<double, 8> cmdKd_{};
    };
} // namespace Dog8Dof