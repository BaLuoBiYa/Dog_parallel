#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "dog_msg/msg/gait_twist.hpp"
#include "dog_remote/sbusReader.hpp"

namespace Dog8Dof {

    /**
     * @brief SBUS 遥控器 → GaitTwist 转换节点
     *
     * 通过 SbusReader 读取 SBUS 串口数据，将各通道映射为 dog_msg/GaitTwist
     * 高层步态指令并发布到 /dog/command 话题。
     *
     * 默认通道映射 (Radiomaster 布局):
     *   CH0  RIGHTX        → roll          (副翼)
     *   CH1  RIGHTY        → pitch         (升降舵)
     *   CH2  LEFTY         → linear_x      (油门，前进/后退)
     *   CH3  LEFTX         → linear_y      (方向舵，侧向)
     *   CH4  LEFTSWITCH3   → gait 模式切换  (sw3: 上=WALK, 中=STAND, 下=PARALYSE)
     *   CH5  LEFTSWITCH2   → 使能控制       (sw2: 上=使能, 下=急停)
     *   CH6  LEFTROTARY    → base_height   (旋钮，高度调整)
     *   CH7  RIGHTROTARY   → step_duration  (旋钮，步频调整)
     *   CH8  LEFTBOTTOMX   → step_height   (旋钮，步高调整)
     *   CH12 LEFTTOPBOTTOM → JUMP 触发      (按钮，跳跃)
     *
     * 支持 YAML 参数热加载。
     */
    class SbusGaitTwistNode : public rclcpp::Node {
      public:
        /**
         * @param port       SBUS 串口设备路径，默认 /dev/ttyACM0
         * @param baudrate   波特率，默认 100000
         * @param nodeName   节点名称
         * @param yamlPath   YAML 配置文件路径 (可选)
         * @param publishRateHz  GaitTwist 发布频率 (Hz)
         */
        explicit SbusGaitTwistNode(
            const std::string &port = "/dev/ttyACM0",
            uint32_t baudrate = 100000,
            const std::string &nodeName = "sbus_gait_twist_node",
            const std::string &yamlPath = "",
            double publishRateHz = 50.0);

        ~SbusGaitTwistNode() override;

      private:
        // ── 通道索引 ──
        enum Channel : size_t {
            RIGHTX = 0,        // 副翼 → roll
            RIGHTY = 1,        // 升降舵 → pitch
            LEFTY = 2,         // 油门 → linear_x
            LEFTX = 3,         // 方向舵 → linear_y
            LEFTSWITCH3 = 4,   // 状态切换 (sw3)
            LEFTSWITCH2 = 5,   // 使能控制 (sw2)
            LEFTROTARY = 6,    // 高度调整旋钮
            RIGHTROTARY = 7,   // 步频调整旋钮
            LEFTBOTTOMX = 8,   // 步高调整旋钮
            RIGHTSWITCH2 = 9,  // 视觉模式 (未直接用于 GaitTwist)
            RIGHTSWITCH3 = 10, // 自动模式 (未直接用于 GaitTwist)
            ROWBOTTOM = 11,    // 模式切换 (未直接用于 GaitTwist)
            LEFTTOPBOTTOM = 12, // 跳跃按钮
            LRIGHTBOTTOMY = 13, // 前后伸展 (未直接用于 GaitTwist)
            LRIGHTBOTTOMX = 14, // 侧向伸展 (未直接用于 GaitTwist)
            BROKEN = 15,
        };

        // ── 步态枚举 (与 GaitTwist.msg 对齐) ──
        enum Gait : uint8_t {
            STAND = 0,
            WALK = 1,
            JUMP = 2,
        };

        // ── SBUS 硬件 ──
        std::unique_ptr<SbusReader> sbus_;
        std::string serialPort_;
        uint32_t baudrate_;

        // ── 发布定时器 ──
        rclcpp::TimerBase::SharedPtr publishTimer_;
        rclcpp::Publisher<dog_msg::msg::GaitTwist>::SharedPtr commandPub_;

        // ── 可调参数 (YAML 可覆盖) ──
        float maxLinearX_{0.8f};     // 最大前向速度 (m/s)
        float maxLinearY_{0.4f};     // 最大侧向速度 (m/s)
        float maxRoll_{0.4f};        // 最大 roll 角 (rad)
        float maxPitch_{0.4f};       // 最大 pitch 角 (rad)
        float baseHeightMin_{80.0f};  // 身高下限 (mm)
        float baseHeightMax_{300.0f}; // 身高上限 (mm)
        float stepHeightMin_{0.0f};   // 步高下限 (mm)
        float stepHeightMax_{150.0f}; // 步高上限 (mm)
        float stepDurationMin_{0.20f}; // 步频周期下限 (s)
        float stepDurationMax_{0.80f}; // 步频周期上限 (s)
        float stickDeadzone_{0.05f};   // 摇杆死区 (归一化)
        float stickExponent_{1.5f};    // 摇杆指数响应

        // SBUS 值范围
        static constexpr uint16_t kSbusMin = 192;
        static constexpr uint16_t kSbusMax = 1792;
        static constexpr uint16_t kSbusMid = 992;

        // ── 状态 ──
        std::atomic<bool> enabled_{false};
        bool lastJumpBtn_{false};

        // ── 发布频率 ──
        double publishRateHz_{50.0};

        // ── YAML 热加载 ──
        std::string yamlPath_;
        time_t yamlLastMtime_{0};
        rclcpp::TimerBase::SharedPtr yamlWatchTimer_;
        double yamlWatchInterval_{1.0};

        // ── 方法 ──
        /** 定时发布回调 */
        void publishCallback();

        /**
         * @brief 将 SBUS 值映射为归一化 [-1.0, 1.0] 带死区和指数
         * @param sbusVal 原始 SBUS 值 [192, 1792]
         * @return 归一化值 [-1.0, 1.0]
         */
        float mapStick(uint16_t sbusVal) const;

        /**
         * @brief 将 SBUS 旋钮值线性映射到目标范围
         * @param sbusVal 原始 SBUS 值 [192, 1792]
         * @param outMin  输出下限
         * @param outMax  输出上限
         * @return 映射后的值
         */
        float mapKnob(uint16_t sbusVal, float outMin, float outMax) const;

        /** 从 YAML 文件加载全部参数 (不使用 ROS 参数服务器) */
        void loadYamlConfig();

        /** 定时检查 YAML 文件是否更新 */
        void watchYamlCallback();

        /** 获取文件修改时间 */
        static time_t fileMtime(const std::string &path);
    };

} // namespace Dog8Dof
