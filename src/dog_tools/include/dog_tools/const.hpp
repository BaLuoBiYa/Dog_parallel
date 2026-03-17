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
      public:
        // 步长值映射
        static constexpr float STEP_LENGTH_MAX = 220.0f;
        static constexpr float STEP_LENGTH_MIN = -220.0f;

        // 转向值映射
        static constexpr float BIAS_MAX = 100.0f;
        static constexpr float BIAS_MIN = -100.0f;

        // 身体高度映射
        static constexpr float BODY_HEIGHT_MAX = 300.0f;
        static constexpr float BODY_HEIGHT_MIN = 80.0f;

        // // 平移值映射（待测试）
        //  static constexpr float TRANSLATION_HEIGHT_MAX = 70.0f;
        //  static constexpr float TRANSLATION_HEIGHT_MIN = -70.0f;

        // // 前后倾值映射（待测试）
        //  static constexpr float TILT_HEIGHT_MAX = 70.0f;
        //  static constexpr float TILT_HEIGHT_MIN = -70.0f;

        // 步高映射
        static constexpr float STEP_HEIGHT_MAX = 150.0f;
        static constexpr float STEP_HEIGHT_MIN = 0.0f;

        // 伸展值映射
        static constexpr float STRETCH_OFFSET_MAX = 75.0f;
        static constexpr float STRETCH_OFFSET_MIN = -75.0f;

        // 持续时间范围
        static constexpr float DURATION_NUM_MAX = 42.0f;
        static constexpr float DURATION_NUM_MIN = 8.0f;

        // 遥控器手感参数
        static constexpr float DEAD_ZONE = 0.005f; // 死区, 0.0 ~ 1.0
        static constexpr float EXPONENT = 1.35f;   // 指数, > 1.0

        // 腿长限制
        static constexpr float LEG_LENGTH_MIN = 70.0f;  // real min 67.5mm
        static constexpr float LEG_LENGTH_MAX = 305.0f; // real max 310mm

        // 机身尺寸
        static constexpr float BODY_LENGTH_LONGITUDINAL = 408.55f; // 前后关节距离 (mm)
        static constexpr float BODY_WIDTH_LATERAL = 370.9f;        // 左右关节水平距离 (mm)

        static constexpr float BODY_MASS = 0.0f;
        static constexpr float G = 9.8;

        static constexpr unsigned long RESOLUTION = 1000; // 轨迹生成分辨率
    };

} // namespace Dog8Dof