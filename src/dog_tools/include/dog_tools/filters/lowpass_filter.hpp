/**
 * @file    Filter_drv.hpp
 * @author  郝运来-hyl
 * @version 1.0
 * @date    2025-2-23
 * @brief   一阶低通滤波与平均滤波, 使用静态队列库实现
 * @details
 */
#pragma once

namespace Dog8Dof {
    class LowPassFilter {
        float alpha = 1.0f; // 默认滤波系数
        float last_output = 0.0f;
        float one_minus_alpha = 0.0f; // 新增成员变量

      public:
        // 参数配置（0.0~1.0）
        void set_Alpha(const float a) noexcept {
            alpha = a;
            one_minus_alpha = 1.0f - alpha; // 预计算
        }

        // 独立低通滤波
        float get_Lowpass(const float input) noexcept {
            const float output = alpha * input + one_minus_alpha * last_output; // 使用预存值
            last_output = output;
            return last_output;
        }

        // 清空滤波器状态
        void reset() noexcept { last_output = 0.0f; }
    };
} // namespace Dog8Dof
