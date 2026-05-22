#include "dog_hardware/goM8010.hpp"

#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace Dog8Dof {
    class ZeroSetter : public rclcpp::Node {
      public:
        ZeroSetter(const std::string configFilePath);
        bool init();
        bool callibrateMotorInteractive();
        bool callibrateMotorNoneInteractive();

      private:
        static constexpr std::array<size_t, 4> LEFT_MOTOR_INDICES{0, 1, 4, 5};
        static constexpr std::array<size_t, 4> RIGHT_MOTOR_INDICES{2, 3, 6, 7};

        std::map<std::string, uint8_t> jointMapping_;
        const std::string jointNames_[8] = {"LF_Front",
                                            "LF_Rear",
                                            "RF_Front",
                                            "RF_Rear",
                                            "LR_Front",
                                            "LR_Rear",
                                            "RR_Front",
                                            "RR_Rear"};

        std::string portNameLeft_;
        std::string portNameRight_;
        std::shared_ptr<Dog8Dof::SerialPort> portLeft_;
        std::shared_ptr<Dog8Dof::SerialPort> portRight_;

        YAML::Node node_;
        std::string configPath_;

        std::vector<goM8010Interface> motor_;
        std::array<float, 8> zeroOffsetByIndex_{};

        const rclcpp::Logger logger_ = this->get_logger();
        const float warnThreshold_ = 10.0f;

        static const size_t repeatLoops_ = 100;
        static const size_t motorNumber_ = 8;

        float frontAngleIdle_; // 初始状态下前电机的理论位置
        float rearAngleIdle_;  // 初始状态下后电机的理论位置

        bool getOffsetInteractive(size_t jointIdx);
        bool select(std::string input);
        void updateOffset(float offSet, size_t jointIdx);
        void flushToFile();
    };

} // namespace Dog8Dof