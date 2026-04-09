#include "dog_hardware/goM8010.hpp"

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace Dog8Dof {
    class ZeroSetter : public rclcpp::Node {
      public:
        ZeroSetter(const std::string configFilePath);
        bool callibrateMotorInteractive();
        bool callibrateMotorNoneInteractive();

      private:
        std::map<std::string, uint8_t> jointMapping_;
        const std::string jointNames_[8] = {"LF_Front",
                                            "LF_Rear",
                                            "RF_Front",
                                            "RF_Rear",
                                            "LR_Front",
                                            "LR_Rear",
                                            "RR_Front",
                                            "RR_Rear"};
        std::string portName_;
        std::shared_ptr<SerialPort> port_;

        YAML::Node node_;
        std::string configPath_;

        std::unique_ptr<goM8010Interface> motor_[8];
        std::array<float, 8> zeroOffsetByIndex_{};

        const rclcpp::Logger logger_ = this->get_logger();
        const float warnThreshold_ = 10.0f;

        static const size_t repeatLoops_ = 100;
        static const size_t motorNumber_ = 2;

        float frontAngleIdle_ = (-2.7f); // 初始状态下前电机的理论位置
        float rearAngleIdle_ = 12.14f;   // 初始状态下后电机的理论位置

        bool getOffsetInteractive(size_t jointIdx);
        bool select(std::string input);
        void writeFile(float offSet, size_t jointIdx);
    };

} // namespace Dog8Dof