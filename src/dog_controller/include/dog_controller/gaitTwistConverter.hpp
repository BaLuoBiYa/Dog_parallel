#pragma once

#include "dog_msg/msg/gait_twist.hpp"
#include "dog_tools/const.hpp"

namespace Dog8Dof {

    /**
     * @brief 每条腿的步态参数（从 GaitTwist 转译得到）
     *
     * 将 GaitTwist 中的高层指令（速度 + 姿态 + 步态模式）
     * 翻译为 GaitManager 可直接使用的每腿步态参数
     */
    struct GaitParameters {
        float stepLength;   ///< 步长 (mm)，正值为前进方向
        float stepHeight;   ///< 步高 (mm)
        float baseHeight;   ///< 基座高度 (mm)
        float stepDuration; ///< 步态周期 (s)
        float phaseOffset;  ///< 相位偏移 [0, 1)
    };

    /**
     * @brief GaitTwist → per-leg GaitParameters 转换器
     *
     *   - 线速度 → 步长 (stepLength = velocity / frequency)
     *   - 角速度 → 转弯步幅偏置 (正弦定理)
     *   - roll/pitch → 四腿高度补偿 (getDeltaZ)
     *
     * 用法：
     *   GaitTwistConverter converter(0.3f, 0.5f, 200.0f);
     *   GaitParameters params[4];
     *   converter.convert(gaitTwistMsg, params);
     *   // 然后直接传给 GaitManager::setWalkGait/setStandGait/setJumpGait
     */
    class GaitTwistConverter {
      public:
        /**
         * @param stepDuration     默认步态周期 (s)
         * @param baseHeight       默认基座高度 (mm)
         */
        GaitTwistConverter(float stepDuration = 0.5f, float baseHeight = 200.0f);

        /**
         * @brief 将 GaitTwist 消息转译为四腿步态参数
         * @param msg 输入的高层指令
         * @param out 输出的四腿参数数组，顺序：LF, LR, RF, RR
         * @return 转换成功返回 true，参数越界返回 false
         */
        bool convert(const dog_msg::msg::GaitTwist &msg, GaitParameters out[4]);

      private:
        float stepDuration_;
        float baseHeight_;

        static constexpr float bodyLength_ = Const::BODY_LENGTH_LONGITUDINAL;
        static constexpr float bodyWidth_ = Const::BODY_WIDTH_LATERAL;

        /** 根据 roll/pitch 计算四条腿的高度补偿量 (mm) */
        std::array<float, 4> getDeltaZ(float roll, float pitch);
    };

} // namespace Dog8Dof
