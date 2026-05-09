#pragma once

#include <dog_hardware/CanManager.hpp>
#include <dog_tools/TypeConvert.hpp>
#include <dog_tools/pid.hpp>
#include <math.h>
#include <memory>

/* Exported macros -----------------------------------------------------------*/
namespace Dog8Dof {
// rpm换算到rad/s
#define RPM_TO_RADPS (2.0f * M_PI / 60.0f)
// deg换算到rad
#define DEG_TO_RAD (M_PI / 180.0f)
// 摄氏度换算到开氏度
#define CELSIUS_TO_KELVIN (273.15f)

    enum Enum_Motor_DJI_ID {
        Motor_DJI_ID_0x201 = 1,
        Motor_DJI_ID_0x202,
        Motor_DJI_ID_0x203,
        Motor_DJI_ID_0x204,
        Motor_DJI_ID_0x205,
        Motor_DJI_ID_0x206,
        Motor_DJI_ID_0x207,
        Motor_DJI_ID_0x208,
    };

    /**
     * @brief 大疆电机经过处理的数据
     *
     */
    struct Struct_Motor_DJI_Rx_Data {
        float Now_Angle;
        float Now_Omega;
        float Now_Current;
        float Now_Temperature;
        float Now_Power;
        uint32_t Pre_Encoder;
        int32_t Total_Encoder;
        int32_t Total_Round;
    };

    /**
     * @brief 大疆电机源数据
     *
     */
    struct Struct_Motor_DJI_CAN_Rx_Data {
        uint16_t Encoder_Reverse;
        int16_t Omega_Reverse;
        int16_t Current_Reverse;
        uint8_t Temperature;
        uint8_t Reserved;
    } __attribute__((packed));

    enum Enum_Motor_DJI_Status {
        Motor_DJI_Status_DISABLE = 0,
        Motor_DJI_Status_ENABLE,
    };
    /**
     * @brief 大疆电机控制方式
     *
     */
    enum Enum_Motor_DJI_Control_Method {
        Motor_DJI_Control_Method_VOLTAGE = 0,
        Motor_DJI_Control_Method_CURRENT,
        Motor_DJI_Control_Method_TORQUE,
        Motor_DJI_Control_Method_OMEGA,
        Motor_DJI_Control_Method_ANGLE,
    };

    class DJI_Motor_2006 {
      private:
        // 初始化相关常量
        std::unique_ptr<CanManager> CAN_Handler;

        // 收数据绑定的CAN ID, C6系列0x201~0x208, GM系列0x205~0x20b
        Enum_Motor_DJI_ID CAN_Rx_ID;
        // 发送ID
        canid_t CAN_Tx_ID;
        // 电机控制方式
        Enum_Motor_DJI_Control_Method Motor_DJI_Control_Method =
            Motor_DJI_Control_Method_ANGLE;
        // 发送缓存区
        uint8_t *Tx_Data;
        // 减速比, 默认带减速箱
        float Gearbox_Rate;
        // 最大电流
        float Current_Max;
        // 发送缓存
        uint8_t CAN1_0x1ff_Tx_Data[8];
        uint8_t CAN1_0x200_Tx_Data[8];

        // PID角度环控制
        Class_PID PID_Angle;
        // PID角速度环控制
        Class_PID PID_Omega;

        // 常量

        // 一圈编码器刻度
        uint16_t Encoder_Num_Per_Round = 8192;

        // 电流到输出的转化系数
        float Current_To_Out = 10000.0f / 10.0f;
        // 理论最大输出电流
        float Theoretical_Output_Current_Max = 10.0f;

        // 内部变量

        // 当前时刻的电机接收flag
        uint32_t Flag = 0;
        // 前一时刻的电机接收flag
        uint32_t Pre_Flag = 0;
        // 输出量
        float Out = 0.0f;

        // 读变量

        // 电机状态
        Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;
        // 电机对外接口信息
        Struct_Motor_DJI_Rx_Data Rx_Data;

        // 内部函数

        void Data_Process();

        void Output();

      public:
        DJI_Motor_2006(std::string &can_interface,
                       Enum_Motor_DJI_ID __CAN_Rx_ID,
                       Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method,
                       float __Gearbox_Rate,
                       float __Current_Max);
        ~DJI_Motor_2006();
        uint8_t *allocate_tx_data(Enum_Motor_DJI_ID __CAN_ID);
        void PID_Calculate();
        void TIM_Calculate_PeriodElapsedCallback();

        // 写变量

        // 读写变量

        // 目标的角度, rad
        float Target_Angle = 0.0f;
        // 目标的速度, rad/s
        float Target_Omega = 0.0f;
        // 目标的电流, A
        float Target_Current = 0.0f;
        // 前馈的速度, rad/s
        float Feedforward_Omega = 0.0f;
        // 前馈的电流, A
        float Feedforward_Current = 0.0f;
        // 清除标志位
        uint8_t Angle_Clear_Flag = 0;
    };
} // namespace Dog8Dof
