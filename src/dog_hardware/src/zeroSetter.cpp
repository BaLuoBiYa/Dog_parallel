#include "dog_hardware/zeroSetter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

namespace Dog8Dof {
    ZeroSetter::ZeroSetter(const std::string configPath)
        : rclcpp::Node("dog_initializer") {
        configPath_ = configPath;

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

        RCLCPP_INFO_STREAM(logger_,
                           "Settting zero points for " << motorNumber_ << " motors");
        RCLCPP_INFO_STREAM(logger_, "Using " << repeatLoops_ << " Sample points");

        try {
            node_ = YAML::LoadFile(configPath_);

            portNameLeft_ = node_["RS485_Port_Left"].as<std::string>();
            portNameRight_ = node_["RS485_Port_Right"].as<std::string>();

            frontAngleIdle_ = node_["idleAngle"]["front"].as<float>();
            rearAngleIdle_ = node_["idleAngle"]["rear"].as<float>();

            for (size_t i = 0; i < motorNumber_; ++i) {
                const YAML::Node joint_node = node_[jointNames_[i]];
                if (!joint_node) {
                    RCLCPP_ERROR(
                        logger_, "Missing joint node: %s", jointNames_[i].c_str());
                    throw std::runtime_error("Missing joint node");
                }

                const int id = joint_node["id"].as<int>();
                const float offset = joint_node["offset"].as<float>();

                jointMapping_[jointNames_[i]] = static_cast<uint8_t>(id);
                zeroOffsetByIndex_[i] = offset;
            }
            // 从YAML文件读取参数

        } catch (const std::exception &e) {
            RCLCPP_ERROR(logger_, "on_init exception: %s", e.what());
            throw std::runtime_error("Can't load joint configs");
        }

        portLeft_.reset();
        portRight_.reset();
        motor_.clear();
        // 清空智能指针
    }

    bool ZeroSetter::init() {
        size_t attempt = 0;
        const std::chrono::seconds retryDelay(3);

        // ---- 打开左路 485 ----
        while (rclcpp::ok()) {
            ++attempt;
            try {
                portLeft_ = std::make_shared<Dog8Dof::SerialPort>(portNameLeft_);
                RCLCPP_INFO_STREAM(logger_,
                                   "Opened left serial port "
                                       << portNameLeft_ << " on attempt " << attempt);
                break;
            } catch (const std::exception &e) {
                RCLCPP_WARN(
                    logger_, "RS485 left port attempt %zu failed: %s", attempt, e.what());
                RCLCPP_INFO_STREAM(
                    logger_,
                    "Retrying in " << retryDelay.count() << "s (press Ctrl-C to abort)");
                std::this_thread::sleep_for(retryDelay);
            }
        }
        if (!portLeft_) {
            RCLCPP_ERROR(logger_, "RS485 left port connection aborted or failed");
            throw std::runtime_error("RS485 left port connection failed or aborted");
        }

        // ---- 打开右路 485 ----
        attempt = 0;
        while (rclcpp::ok()) {
            ++attempt;
            try {
                portRight_ = std::make_shared<Dog8Dof::SerialPort>(portNameRight_);
                RCLCPP_INFO_STREAM(logger_,
                                   "Opened right serial port "
                                       << portNameRight_ << " on attempt " << attempt);
                break;
            } catch (const std::exception &e) {
                RCLCPP_WARN(logger_,
                            "RS485 right port attempt %zu failed: %s",
                            attempt,
                            e.what());
                RCLCPP_INFO_STREAM(
                    logger_,
                    "Retrying in " << retryDelay.count() << "s (press Ctrl-C to abort)");
                std::this_thread::sleep_for(retryDelay);
            }
        }
        if (!portRight_) {
            RCLCPP_ERROR(logger_, "RS485 right port connection aborted or failed");
            throw std::runtime_error("RS485 right port connection failed or aborted");
        }

        // ---- 连接电机 ----
        attempt = 0;
        while (rclcpp::ok()) {
            ++attempt;
            motor_.clear();
            try {
                for (size_t i = 0; i < motorNumber_; ++i) {
                    // 左组电机用左端口，右组用右端口
                    std::shared_ptr<Dog8Dof::SerialPort> port =
                        (i == LEFT_MOTOR_INDICES[0] || i == LEFT_MOTOR_INDICES[1] ||
                         i == LEFT_MOTOR_INDICES[2] || i == LEFT_MOTOR_INDICES[3])
                            ? portLeft_
                            : portRight_;
                    motor_.emplace_back(jointMapping_[jointNames_[i]], port);
                    motor_[i].setZero(0.0f);
                }
            } catch (const std::exception &e) {
                RCLCPP_WARN_STREAM(logger_, "Connecting to motor failed: " << e.what());
                RCLCPP_INFO_STREAM(
                    logger_,
                    "Retrying in " << retryDelay.count() << "s (press Ctrl-C to abort)");
                std::this_thread::sleep_for(retryDelay);
                continue;
            }

            // 检查所有电机是否连接成功
            if (motor_.size() == motorNumber_) {
                RCLCPP_INFO_STREAM(
                    logger_,
                    "Successfully connected to all motors on attempt " << attempt);
                return true;
            }
        }

        RCLCPP_ERROR(this->get_logger(), "Failed to initilize motors");
        return false;
    }

    bool ZeroSetter::callibrateMotorInteractive() {
        std::string dummy;

        RCLCPP_WARN_STREAM(
            logger_, "Move front motors to true zero spot! Press any to continue...");
        std::getline(std::cin, dummy);

        for (size_t i = 0; i < motorNumber_; i = i + 2) {
            motor_[i].zeroTor();
            motor_[i + 1].zeroTor();

            RCLCPP_INFO_STREAM(logger_,
                               "Setting for " << jointNames_[i] << " and "
                                              << jointNames_[i + 1]
                                              << "Press any key to continue...");
            std::getline(std::cin, dummy);

            if (!getOffsetInteractive(i)) {
                return false;
            }
            if (!getOffsetInteractive(i + 1)) {
                return false;
            }
        }

        RCLCPP_INFO_STREAM(logger_, "Finish callibrating");
        flushToFile();
        return true;
    }

    bool ZeroSetter::callibrateMotorNoneInteractive() {
        if (repeatLoops_ == 0) {
            RCLCPP_ERROR(logger_, "repeatLoops_==0");
            return false;
        }

        RCLCPP_WARN_STREAM(logger_, "Non-interactive mode,callibrating....");

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        for (size_t i = 0; i < motorNumber_; ++i) {
            RCLCPP_INFO_STREAM(logger_, "Checking " << jointNames_[i]);

            float currentPosSum = 0.0f;
            motor_[i].zeroTor();
            for (size_t j = 0; j < repeatLoops_; ++j) {
                motor_[i].zeroTor();
                currentPosSum += motor_[i].getPos(); // 这里是原始电机角(当前 setZero(0))
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            const float measuredRaw = currentPosSum / static_cast<float>(repeatLoops_);
            // const float oldOffset = zeroOffsetByIndex_[i];
            const float desiredCtrl = (i % 2 == 0) ? frontAngleIdle_ : rearAngleIdle_;
            // const float expectedRaw =
            //     desiredCtrl
            const float offset = measuredRaw - desiredCtrl;

            RCLCPP_INFO_STREAM(logger_,
                               jointNames_[i] << ": measuredRaw=" << measuredRaw
                                              << ", expectedRaw=" << desiredCtrl
                                              << ", offset=" << offset);

            updateOffset(offset, i);
        }

        flushToFile();
        return true;
    }

    bool ZeroSetter::select(std::string input) {
        if (input == "y" || input == "Y" || input == "yes" || input == "Yes" ||
            input == "YES") {
            return true;
        } else {
            return false;
        }
    };

    bool ZeroSetter::getOffsetInteractive(size_t jointIdx) {
        float currentPosSum = 0.0f;
        for (size_t j = 0; j < repeatLoops_; j++) {
            motor_[jointIdx].zeroTor();
            currentPosSum += motor_[jointIdx].getPos();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        float measured = currentPosSum / static_cast<float>(repeatLoops_);
        float oldOffset = zeroOffsetByIndex_[jointIdx];
        float desired = 0.0f;
        // 交互模式下，应该手动将关节移动至真零位
        float newOffset = measured - desired;

        if (std::abs(newOffset - oldOffset) > warnThreshold_) {
            RCLCPP_WARN_STREAM(logger_,
                               "New computed offset ("
                                   << newOffset << ") differs from previous ("
                                   << oldOffset << ") ! Sure to continue?(y/n)");
            std::string userDecision;
            std::getline(std::cin, userDecision);
            if (select(userDecision)) {
                updateOffset(newOffset, jointIdx);
                return true;
            } else {
                RCLCPP_WARN_STREAM(logger_, "Stopping zero setting progress");
                return false;
            }
        } else {
            updateOffset(newOffset, jointIdx);
            return true;
        }
    }

    void ZeroSetter::updateOffset(float offSet, size_t jointIdx) {
        float oldOffset = zeroOffsetByIndex_[jointIdx];
        zeroOffsetByIndex_[jointIdx] = offSet;
        node_[jointNames_[jointIdx]]["offset"] = offSet;

        RCLCPP_INFO_STREAM(logger_,
                           "Updating zero offset of "
                               << jointNames_[jointIdx] << " from " << oldOffset
                               << " to " << offSet);
    }

    void ZeroSetter::flushToFile() {
        // 确保 idleAngle 也写回
        node_["idleAngle"]["front"] = frontAngleIdle_;
        node_["idleAngle"]["rear"] = rearAngleIdle_;

        std::ofstream outFile(configPath_ + ".tmp");
        if (!outFile.is_open()) {
            RCLCPP_ERROR(logger_, "Failed open tmp file for final write");
            return;
        }
        outFile << node_;
        outFile.close();
        std::rename((configPath_ + ".tmp").c_str(), configPath_.c_str());

        RCLCPP_INFO_STREAM(logger_, "All calibrations written to " << configPath_);
    }

} // namespace Dog8Dof

int main(int argc, char **argv) {

    const std::string configPath =
        ament_index_cpp::get_package_share_directory("dog_bringup") +
        "/parameters/motors.yaml";
    rclcpp::init(argc, argv);

    bool interactive = isatty(fileno(stdin));

    try {
        auto zs = std::make_unique<Dog8Dof::ZeroSetter>(configPath);
        if (!zs->init()) {
            RCLCPP_ERROR(zs->get_logger(), "Failed to initialize ZeroSetter");
            rclcpp::shutdown();
            return 1;
        }

        if (interactive) {
            RCLCPP_INFO(
                zs->get_logger(),
                "Interactive terminal detected — running interactive calibration");
            if (!zs->callibrateMotorInteractive()) {
                RCLCPP_ERROR(zs->get_logger(), "Interactive calibration failed");
                rclcpp::shutdown();
                return 2;
            }
        } else {
            RCLCPP_INFO(zs->get_logger(),
                        "No interactive terminal — running non-interactive calibration");
            if (!zs->callibrateMotorNoneInteractive()) {
                RCLCPP_ERROR(zs->get_logger(), "Non-interactive calibration failed");
                rclcpp::shutdown();
                return 3;
            }
        }

        RCLCPP_INFO(zs->get_logger(), "Calibration finished successfully");
        rclcpp::shutdown();
        return 0;

    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << "\n";
        rclcpp::shutdown();
        return 4;
    }
}