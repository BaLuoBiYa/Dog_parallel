/**
 * @file gaitTwistConverter.cpp
 * @brief GaitTwist → per-leg GaitParameters 转换器实现
 *
 * 核心逻辑参考 twistToGait / commandManager：
 *   - 线速度转步长：stepLength = linerX * 1000 / stepDuration
 *   - 角速度转转弯偏置：正弦定理 + 机身对角线
 *   - roll/pitch 转四腿高度补偿
 */

#include "dog_controller/gaitTwistConverter.hpp"
#include <cmath>

namespace Dog8Dof {

    GaitTwistConverter::GaitTwistConverter(float stepDuration, float baseHeight)
        : stepDuration_(stepDuration), baseHeight_(baseHeight) {
    }

    bool GaitTwistConverter::convert(const dog_msg::msg::GaitTwist &msg,
                                     GaitParameters out[4]) {
        using GT = dog_msg::msg::GaitTwist;

        switch (msg.gait) {
        case GT::STAND: {
            const auto dz = getDeltaZ(msg.roll, msg.pitch);
            for (uint8_t i = 0; i < 4; ++i) {
                out[i].stepLength = 0.0f;
                out[i].stepHeight = 0.0f;
                out[i].baseHeight = msg.base_height + dz[i];
                out[i].stepDuration = 0.0f;
                out[i].phaseOffset = 0.0f;
            }
            return true;
        }

        case GT::WALK: {
            // 线速度 → 步长 (m/s → mm, /频率)
            const float linearVel_mm = msg.linear_x * 1000.0f;
            const float stepLength = linearVel_mm * msg.step_duration;

            const float bias = bodyWidth_ * 0.5f * msg.linear_y / msg.step_duration;

            // 四腿步长（含转弯偏置）
            const std::array<float, 4> stepLengths = {
                stepLength + bias, // LF
                stepLength + bias, // LR
                stepLength - bias, // RF
                stepLength - bias  // RR
            };

            const auto dz = getDeltaZ(msg.roll, msg.pitch);

            for (uint8_t i = 0; i < 4; ++i) {
                out[i].stepLength = stepLengths[i];
                out[i].stepDuration = msg.step_duration;
                out[i].baseHeight = msg.base_height + dz[i];
                out[i].stepHeight = msg.step_height;

                // 边界检查
                if (std::abs(out[i].stepLength) > Const::STEP_LENGTH_MAX) {
                    if (out[i].stepLength > 0) {
                        out[i].stepLength = Const::STEP_LENGTH_MAX;
                    } else {
                        out[i].stepLength = -Const::STEP_LENGTH_MAX;
                    }
                }
                if (out[i].stepHeight > out[i].baseHeight || out[i].stepHeight < Const::STEP_HEIGHT_MIN) {
                    if(out[i].stepHeight < Const::STEP_HEIGHT_MIN) {
                        out[i].stepHeight = Const::STEP_HEIGHT_MIN;
                    } else {
                        out[i].stepHeight = out[i].baseHeight;
                    }
                }
                if (out[i].baseHeight > Const::BODY_HEIGHT_MAX ||
                    out[i].baseHeight < Const::BODY_HEIGHT_MIN) {
                    if (out[i].baseHeight > Const::BODY_HEIGHT_MAX) {
                        out[i].baseHeight = Const::BODY_HEIGHT_MAX;
                    } else {
                        out[i].baseHeight = Const::BODY_HEIGHT_MIN;
                    }
                }
                if (out[i].stepDuration < 0.0f) {
                    return false;
                }
            }
            return true;
        }

        case GT::JUMP: {
            // 尚未实现跳跃步态，暂时直接映射为站立
            for (uint8_t i = 0; i < 4; ++i) {
                out[i].stepLength = 0.0f;
                out[i].stepHeight = 0.0f;
                out[i].baseHeight = baseHeight_;
                out[i].stepDuration = stepDuration_;
                out[i].phaseOffset = 0.0f;
            }
            return true;
        }

        default:
            // 未知步态 → 回退站立
            for (uint8_t i = 0; i < 4; ++i) {
                out[i].stepLength = 0.0f;
                out[i].stepHeight = 0.0f;
                out[i].baseHeight = baseHeight_;
                out[i].stepDuration = stepDuration_;
                out[i].phaseOffset = 0.0f;
            }
            return false;
        }
    }

    // =========================================================================
    //  getDeltaZ — roll/pitch 导致的四腿高度补偿
    // =========================================================================
    std::array<float, 4> GaitTwistConverter::getDeltaZ(float roll, float pitch) {
        const float L2 = bodyLength_ * 0.5f;
        const float W2 = bodyWidth_ * 0.5f;

        // pitch: 前倾 → 前腿降低、后腿升高
        const float dzPitchFront = -std::sin(pitch) * L2;
        const float dzPitchRear = +std::sin(pitch) * L2;

        // roll: 右倾 → 右腿降低、左腿升高
        const float dzRollLeft = +std::sin(roll) * W2;
        const float dzRollRight = -std::sin(roll) * W2;

        return {
            dzPitchFront + dzRollLeft,  // LF
            dzPitchRear + dzRollLeft,   // LR
            dzPitchFront + dzRollRight, // RF
            dzPitchRear + dzRollRight   // RR
        };
    }

} // namespace Dog8Dof
