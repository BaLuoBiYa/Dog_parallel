#include "dog_controller/stateMonitor.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

namespace Dog8Dof {

    // ============================================================================
    // 构造
    // ============================================================================

    StateMonitor::StateMonitor(const std::array<std::string, 8> &motor_names)
        : motor_names_(motor_names), cpu_baseline_valid_(false), ema_frequency_(0.0f) {
        // 初始化电机名称
        for (size_t i = 0; i < 8; ++i) {
            state_.motors[i].name = motor_names_[i];
        }
        last_update_time_ = std::chrono::steady_clock::now();
    }

    // ============================================================================
    // update() — 主入口，每个控制周期调用
    // ============================================================================

    void StateMonitor::update(const std::array<int8_t, 8> &motor_temps,
                              const std::array<float, 8> &motor_positions,
                              const std::array<float, 8> &motor_velocities,
                              const std::array<float, 8> &motor_torques) {
        // ---- 1. 采集 CPU 使用率 ----
        CpuTimes curr_cpu;
        if (readCpuTimes(curr_cpu)) {
            if (cpu_baseline_valid_) {
                state_.cpu_usage = computeCpuUsage(prev_cpu_times_, curr_cpu);
            } else {
                state_.cpu_usage = 0.0f;
            }
            prev_cpu_times_ = curr_cpu;
            cpu_baseline_valid_ = true;
        }

        // ---- 2. 采集 CPU 温度 ----
        state_.cpu_temperature = readCpuTemperature();

        // ---- 3. 采集电机状态 ----
        for (size_t i = 0; i < 8; ++i) {
            state_.motors[i].position = motor_positions[i];
            state_.motors[i].velocity = motor_velocities[i];
            state_.motors[i].torque = motor_torques[i];
            state_.motors[i].temperature = motor_temps[i];
        }

        // ---- 4. 计算控制回路频率 ----
        updateFrequency();

        // ---- 5. 打时间戳 ----
        state_.timestamp = std::chrono::steady_clock::now();

        // ---- 6. 线程安全写入 ----
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // state_ 已在上面原地更新，此处仅保护读取侧
            // （update 假定单写入者，仅 getState 需要 mutex）
        }
    }

    // ============================================================================
    // getState() — 线程安全快照
    // ============================================================================

    SystemState StateMonitor::getState() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_;
    }

    // ============================================================================
    // CPU 使用率：读取 /proc/stat
    // ============================================================================

    bool StateMonitor::readCpuTimes(CpuTimes &out) {
        std::ifstream stat("/proc/stat");
        if (!stat.is_open()) {
            return false;
        }

        std::string line;
        if (!std::getline(stat, line)) {
            return false;
        }

        // 格式: cpu  user nice system idle iowait irq softirq steal ...
        std::istringstream iss(line);
        std::string cpu_label;
        iss >> cpu_label; // "cpu"
        if (cpu_label != "cpu") {
            return false;
        }

        iss >> out.user >> out.nice >> out.system >> out.idle >> out.iowait >> out.irq >>
            out.softirq >> out.steal;
        return !iss.fail();
    }

    float StateMonitor::computeCpuUsage(const CpuTimes &prev, const CpuTimes &curr) {
        auto total = [](const CpuTimes &t) -> unsigned long long {
            return t.user + t.nice + t.system + t.idle + t.iowait + t.irq + t.softirq +
                   t.steal;
        };

        unsigned long long prev_total = total(prev);
        unsigned long long curr_total = total(curr);
        unsigned long long prev_idle = prev.idle + prev.iowait;
        unsigned long long curr_idle = curr.idle + curr.iowait;

        unsigned long long total_delta = curr_total - prev_total;
        unsigned long long idle_delta = curr_idle - prev_idle;

        if (total_delta == 0) {
            return 0.0f;
        }

        float usage = 100.0f * static_cast<float>(total_delta - idle_delta) /
                      static_cast<float>(total_delta);
        return std::clamp(usage, 0.0f, 100.0f);
    }

    // ============================================================================
    // CPU 温度：读取 /sys/class/thermal/
    // ============================================================================

    float StateMonitor::readCpuTemperature() {
        // 尝试常见的 thermal zone 路径
        static const std::vector<const char *> kThermalPaths = {
            "/sys/class/thermal/thermal_zone0/temp",
            "/sys/class/thermal/thermal_zone1/temp",
            "/sys/class/hwmon/hwmon0/temp1_input",
        };

        for (const auto *path : kThermalPaths) {
            if (!std::filesystem::exists(path)) {
                continue;
            }

            std::ifstream file(path);
            if (!file.is_open()) {
                continue;
            }

            int raw_temp = 0;
            file >> raw_temp;
            if (file.fail()) {
                continue;
            }

            // thermal_zone 返回毫摄氏度，hwmon 也可能返回毫摄氏度
            // 统一按毫摄氏度处理（>200 视为毫摄氏度，否则视为摄氏度）
            if (raw_temp > 200) {
                return static_cast<float>(raw_temp) / 1000.0f;
            }
            return static_cast<float>(raw_temp);
        }

        return -1.0f;
    }

    // ============================================================================
    // 控制回路频率：EMA 滤波
    // ============================================================================

    void StateMonitor::updateFrequency() {
        auto now = std::chrono::steady_clock::now();
        auto delta = std::chrono::duration<float>(now - last_update_time_).count();

        if (delta > 0.0f && delta < 1.0f) {
            // delta 合理范围 (>0 且 <1s)，认为时间戳有效
            float instant_freq = 1.0f / delta;

            if (ema_frequency_ <= 0.0f) {
                // 首次有效测量，直接赋值
                ema_frequency_ = instant_freq;
            } else {
                // EMA: F_new = α · F_instant + (1-α) · F_old
                ema_frequency_ =
                    kEmaAlpha * instant_freq + (1.0f - kEmaAlpha) * ema_frequency_;
            }
        }

        state_.control_frequency = ema_frequency_;
        last_update_time_ = now;
    }

} // namespace Dog8Dof
