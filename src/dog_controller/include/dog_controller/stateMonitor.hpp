#pragma once

#include <array>
#include <chrono>
#include <mutex>
#include <string>

namespace Dog8Dof {

    /**
     * @brief 单个电机的实时状态
     */
    struct MotorState {
        std::string name;   ///< 电机名称 (LF_Front, RF_Rear, ...)
        float position;     ///< 当前位置 (rad)
        float velocity;     ///< 当前速度 (rad/s)
        float torque;       ///< 当前力矩 (N·m)
        int8_t temperature; ///< 电机温度 (°C)，>90°C 触发保护
    };

    /**
     * @brief 系统整体实时状态
     *
     * 包含 CPU 使用率、温度、全部电机状态和控制回路频率，
     * 供监控、诊断、日志记录使用。
     */
    struct SystemState {
        float cpu_usage;                  ///< CPU 使用率百分比 (0.0 ~ 100.0)
        float cpu_temperature;            ///< CPU/SoC 温度 (°C)
        std::array<MotorState, 8> motors; ///< 8 个电机的状态
        float control_frequency;          ///< 主控制回路实时频率 (Hz)
        std::chrono::steady_clock::time_point timestamp; ///< 状态采集时间戳
    };

    /**
     * @brief 系统状态监控器
     *
     * 在控制回路中每个周期调用 update()，内部自动采集：
     * - CPU 使用率（基于 /proc/stat 差分量）
     * - CPU 温度（基于 /sys/class/thermal/thermal_zone0/temp）
     * - 各电机状态（由调用者传入）
     * - 控制回路实时频率（基于 update() 调用间隔的 EMA 滤波）
     *
     * 通过 getState() 获取线程安全的最新快照。
     *
     * @note CPU 使用率需要至少两次 update() 调用才能计算出有效值，
     *       首次调用仅初始化基准，返回 0.0。
     */
    class StateMonitor {
      public:
        /**
         * @brief 构造函数
         * @param motor_names 8 个电机的名称列表
         */
        explicit StateMonitor(const std::array<std::string, 8> &motor_names);

        /**
         * @brief 每控制周期调用一次，采集全部系统状态
         * @param motor_temps    8 个电机的温度 (°C)
         * @param motor_positions 8 个电机的位置 (rad)
         * @param motor_velocities 8 个电机的速度 (rad/s)
         * @param motor_torques   8 个电机的力矩 (N·m)
         */
        void update(const std::array<int8_t, 8> &motor_temps,
                    const std::array<float, 8> &motor_positions,
                    const std::array<float, 8> &motor_velocities,
                    const std::array<float, 8> &motor_torques);

        /**
         * @brief 获取最新的系统状态快照（线程安全）
         * @return 当前系统状态的拷贝
         */
        SystemState getState() const;

      private:
        // ---- CPU 使用率计算 ----
        struct CpuTimes {
            unsigned long long user;
            unsigned long long nice;
            unsigned long long system;
            unsigned long long idle;
            unsigned long long iowait;
            unsigned long long irq;
            unsigned long long softirq;
            unsigned long long steal;
        };

        /**
         * @brief 从 /proc/stat 读取 CPU 时间统计
         */
        static bool readCpuTimes(CpuTimes &out);

        /**
         * @brief 根据两次 CPU 时间差计算使用率百分比
         */
        static float computeCpuUsage(const CpuTimes &prev, const CpuTimes &curr);

        // ---- CPU 温度读取 ----
        /**
         * @brief 从 /sys/class/thermal/ 读取 CPU 温度
         * @return 温度 (°C)，读取失败返回 -1.0
         */
        static float readCpuTemperature();

        // ---- 控制频率计算 ----
        /**
         * @brief 根据本次与上次调用时间差，EMA 滤波更新频率估计
         */
        void updateFrequency();

        // ---- 成员变量 ----
        std::array<std::string, 8> motor_names_;
        SystemState state_;
        mutable std::mutex state_mutex_;

        // CPU 使用率计算上下文
        CpuTimes prev_cpu_times_{};
        bool cpu_baseline_valid_;

        // 频率计算上下文
        std::chrono::steady_clock::time_point last_update_time_;
        float ema_frequency_;                    ///< EMA 滤波后的频率估计
        static constexpr float kEmaAlpha = 0.1f; ///< EMA 平滑系数
    };

} // namespace Dog8Dof
