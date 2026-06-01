#pragma once

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <yaml-cpp/yaml.h>

#include <dog_msg/msg/gait_twist.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dog_remote/joystick.hh"

namespace Dog8Dof {

    /**
     * @brief 手柄控制器节点
     *
     * 读取 Linux 手柄设备 (/dev/input/jsX)，将摇杆/按键映射为
     * geometry_msgs/Twist 指令并发布，供下游 TwistToGait / CommandManager 消费。
     *
     * 默认映射 (Xbox 布局):
     *   左摇杆 上下 (axis 1)  →  linear.x  (前进/后退)
     *   右摇杆 左右 (axis 3)  →  angular.z (左转/右转)
     *   按键 A  (btn 0)       →  切换 WALK
     *   按键 B  (btn 1)       →  切换 STAND
     *   按键 X  (btn 2)       →  切换 JUMP
     *   左肩键 (btn 4)        →  降低步高
     *   右肩键 (btn 5)        →  升高步高
     *   十字键上 (axis 7)     →  增加身高
     *   十字键下 (axis 7)     →  降低身高
     *   十字键左 (axis 6)     →  降低步频
     *   十字键右 (axis 6)     →  升高步频
     */
    class JoyStickController : public rclcpp::Node {
      public:
        /** 步态模式枚举 */
        enum class GaitMode : uint8_t {
            STAND = 0,
            WALK = 1,
            JUMP = 2,
        };

        /**
         * @param devicePath  手柄设备路径，默认 /dev/input/js0
         * @param nodeName    节点名称
         * @param pollRateHz  手柄轮询频率 (Hz)
         */
        explicit JoyStickController(
            const std::string &devicePath = "/dev/input/js0",
            const std::string &nodeName = "joystick_controller",
            const std::string &yamlPath =
                "src/dog_remote/parameters/JoystickerController.yaml",
            double pollRateHz = 200.0);

        ~JoyStickController() override;

      private:
        // ── 手柄设备 ──
        std::string devicePath_;
        Joystick joystick_;
        std::thread pollThread_;
        std::atomic<bool> running_{true};

        bool DebugMode = true; // 是否启用调试模式（打印状态）

        // ── 死区 & 灵敏度 ──
        double deadzone_{0.10};         // 摇杆死区 (归一化后)
        double exponent_{1.5};          // 非线性响应指数 (>1 更细腻)
        double maxLinearVel_{0.8};      // 最大线速度 (m/s)
        double maxAngularVel_{0.3};     // 最大角速度 (rad/s)
        double bodyHeight_{250};        // 当前身高 (mm)
        double heightStep_{30};         // 身高调节步长 (mm)
        double MaxbodyHeight_{250};     // 身高上限 (mm)
        double MinbodyHeight_{150};     // 身高下限 (mm)
        double stepHeight_{100};        // 当前步高 (mm)
        double stepHeightStep_{10};     // 步高调节步长 (mm)
        double MaxstepHeight_{250};     // 步高上限 （mm）
        double MinstepHeight_{150};     // 步高下限 （mm）
        double stepDuration_{0.50};     // 当前步频周期 (s), 越小步频越高
        double stepDurationStep_{0.02}; // 步频周期调节步长 (s)

        // ── 状态 ──
        std::atomic<GaitMode> currentGait_{GaitMode::STAND};
        std::atomic<double> currentLinearX_{0.0};
        std::atomic<double> currentAngularZ_{0.0};
        std::atomic<double> currentAngularRoll_{0.0};
        std::atomic<double> currentAngularPitch_{0.0};

        bool lastBtnA_{false};
        bool lastBtnB_{false};
        bool lastBtnX_{false};
        bool lastBtnLB_{false};
        bool lastBtnRB_{false};
        std::atomic_bool jumpRequested_{false};  // JUMP 一次性命令标志
        std::chrono::steady_clock::time_point jumpStartTime_;  // 跳跃开始时间
        std::atomic_bool isJumping_{false};  // 是否正在跳跃

        // ── 十字键边沿检测 ──
        short lastDpadX_{0};
        short lastDpadY_{0};

        // ── ROS 接口 ──
        rclcpp::Publisher<dog_msg::msg::GaitTwist>::SharedPtr commandPub_;
        rclcpp::TimerBase::SharedPtr statusTimer_;

        // ── YAML 动态热加载 ──
        std::string yamlPath_;                        // YAML 配置文件路径
        time_t yamlLastMtime_{0};                     // 上次读取时的文件 mtime
        rclcpp::TimerBase::SharedPtr yamlWatchTimer_; // 文件变化检查定时器
        double yamlWatchInterval_{1.0};               // 检查间隔 (秒)

        // ── 方法 ──
        /** 轮询循环 (在独立线程中运行) */
        void pollLoop();

        /** 将摇杆原始值 (-32768~32767) 映射为归一化速度 (-1.0~1.0) */
        double mapAxis(short rawValue) const;

        /** 发布当前步态和身高信息的日志 */
        void printStatus();

        /** 声明并初始化 ROS 参数 */
        void declareParameters();

        /** 从 YAML 文件加载所有可调参数 (arm_control 风格) */
        void loadYamlConfig();

        /** 定时检查 YAML 文件是否被修改，若修改则热重载 */
        void watchYamlCallback();

        /*检查文件是否被修改*/
        static time_t file_mtime(const std::string &path);
    };

} // namespace Dog8Dof
