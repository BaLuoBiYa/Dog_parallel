#pragma once
namespace Dog8Dof {
    class Const {
        /** 坐标系映射
        机身前方
        X   ←---*（电机）
                |
                |
                ↓
                Z
        -------------------------地面
        */
        /**
            未有特殊说明，长度单位为毫米，时间单位为秒，力单位为牛，力矩为牛米，质量为千克
         */
      public:
        // 步长值映射
        static constexpr float STEP_LENGTH_MAX = 220.0f;
        static constexpr float STEP_LENGTH_MIN = -220.0f;

        // 转向值映射
        // static constexpr float BIAS_MAX = 100.0f;
        // static constexpr float BIAS_MIN = -100.0f;

        // 身体高度映射
        static constexpr float BODY_HEIGHT_MAX = 300.0f;
        static constexpr float BODY_HEIGHT_MIN = 80.0f;

        // 步高映射
        static constexpr float STEP_HEIGHT_MAX = 150.0f;
        static constexpr float STEP_HEIGHT_MIN = 0.0f;

        // // 伸展值映射
        // static constexpr float STRETCH_OFFSET_MAX = 75.0f;
        // static constexpr float STRETCH_OFFSET_MIN = -75.0f;

        // 持续时间范围
        static constexpr float DURATION_NUM_MAX = 42.0f;
        static constexpr float DURATION_NUM_MIN = 8.0f;

        // // 遥控器手感参数
        // static constexpr float DEAD_ZONE = 0.005f; // 死区, 0.0 ~ 1.0
        // static constexpr float EXPONENT = 1.35f;   // 指数, > 1.0

        // 腿长限制
        static constexpr float LEG_LENGTH_MIN = 85.0f;  // real min 85.43mm
        static constexpr float LEG_LENGTH_MAX = 330.0f; // real max 333mm

        // 机身尺寸
        static constexpr float BODY_LENGTH_LONGITUDINAL = 408.55f; // 前后关节距离 (mm)
        static constexpr float BODY_WIDTH_LATERAL = 370.9f; // 左右关节水平距离 (mm)

        // 机身质量
        static constexpr float BODY_MASS = 12.0f;
        static constexpr float G = 9.8f;

        // 轨迹生成参数
        static constexpr unsigned long MAX_RESOLUTION = 1000; // 轨迹生成分辨率
        static constexpr unsigned long MIN_RESOLUTION = 10;   // 轨迹生成分辨率

        // 机械腿参数
        static constexpr float UPPER_LENGTH = 160.0f;  // 机械腿大腿长度 (mm)
        static constexpr float LOWER_LENGTH = 194.64f; // 机械腿小腿长度 (mm)
        static constexpr float TIP_LENGTH = 97.41f;    // 机械腿足端长度 (mm)
    };

} // namespace Dog8Dof