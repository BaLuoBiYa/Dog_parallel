#include "dog_tools/pid.hpp"
namespace Dog8Dof {
    /**
     * @brief PID初始化
     *
     * @param __K_P P值
     * @param __K_I I值
     * @param __K_D D值
     * @param __K_F 前馈
     * @param __I_Out_Max 积分限幅
     * @param __Out_Max 输出限幅
     * @param __D_T 时间片长度
     * @param __Dead_Zone 死区误差阈值
     */
    void Class_PID::Init(float __K_P,
                         float __K_I,
                         float __K_D,
                         float __K_F,
                         float __I_Out_Max,
                         float __Out_Max,
                         float __D_T,
                         float __Dead_Zone) {
        K_P = __K_P;
        K_I = __K_I;
        K_D = __K_D;
        K_F = __K_F;
        I_Out_Max = __I_Out_Max;
        Out_Max = __Out_Max;
        D_T = __D_T;
        Dead_Zone = __Dead_Zone;
        if_Deadband_ = false;
    }

    /**
     * @brief PID调整值, 计算周期与D_T相同
     *
     * @return float 输出值
     */
    void Class_PID::TIM_Calculate_PeriodElapsedCallback() {
        // P输出
        float p_out = 0.0f;
        // I输出
        float i_out = 0.0f;
        // D输出
        float d_out = 0.0f;
        // F输出
        float f_out = 0.0f;
        // 误差
        float error;
        // 绝对值误差
        float abs_error;

        error = Target - Now;
        abs_error = abs(error);
        if_Deadband_ = false;
        // 判断死区
        if (abs_error < Dead_Zone) {
            Target = Now;
            error = 0.0f;
            abs_error = 0.0f;
            // Integral_Error = 0.0f;
            if_Deadband_ = true;

            Out = 0.0f;
            // 善后工作
            Pre_Now = Now;
            Pre_Target = Target;
            Pre_Out = Out;
            Pre_Error = error;

            return;
        }

        else if (error > 0.0f && abs_error > Dead_Zone) {
            error -= Dead_Zone;
        } else if (error < 0.0f && abs_error > Dead_Zone) {
            error += Dead_Zone;
        }

        // 计算p项

        p_out = K_P * error;

        // 计算i项

        // 先累加积分
        Integral_Error += D_T * error;

        // 积分限幅 (增加 K_I != 0 的判断防止除以 0 崩溃)
        if (I_Out_Max != 0.0f && K_I != 0.0f) {
            Math_Constrain(&Integral_Error, -I_Out_Max / K_I, I_Out_Max / K_I);
        }

        i_out = K_I * Integral_Error;

        // 计算d项

        d_out = K_D * (error - Pre_Error) / D_T;
        // 计算前馈

        f_out = K_F;

        // 计算输出

        Out = p_out + i_out + d_out + f_out;

        // 输出限幅
        if (Out_Max != 0.0f) {
            Math_Constrain(&Out, -Out_Max, Out_Max);
        }

        // 善后工作
        Pre_Now = Now;
        Pre_Target = Target;
        Pre_Out = Out;
        Pre_Error = error;
    }
} // namespace Dog8Dof