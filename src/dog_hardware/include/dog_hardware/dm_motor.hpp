#pragma once

#include <dog_hardware/CanManager.hpp>
#include <dog_tools/TypeConvert.hpp>
#include <memory>

/* Exported macros -----------------------------------------------------------*/

// rpm换算到rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)
// deg换算到rad
#define DEG_TO_RAD (PI / 180.0f)
// 摄氏度换算到开氏度
#define CELSIUS_TO_KELVIN (273.15f)

namespace Dog8Dof {
    /**
     * @brief 达妙电机控制状态, 传统模式有效
     *
     */
    enum Enum_Motor_DM_Control_Status_Normal {
        Motor_DM_Control_Status_DISABLE = 0x0,
        Motor_DM_Control_Status_ENABLE,
        Motor_DM_Control_Status_OVERVOLTAGE = 0x8,
        Motor_DM_Control_Status_UNDERVOLTAGE,
        Motor_DM_Control_Status_OVERCURRENT,
        Motor_DM_Control_Status_MOS_OVERTEMPERATURE,
        Motor_DM_Control_Status_ROTOR_OVERTEMPERATURE,
        Motor_DM_Control_Status_LOSE_CONNECTION,
        Motor_DM_Control_Status_MOS_OVERLOAD,
    };

    /**
     * @brief 达妙电机经过处理的数据, 传统模式有效
     *
     */
    struct Struct_Motor_DM_Rx_Data_Normal {
        Enum_Motor_DM_Control_Status_Normal Control_Status;
        float Now_Angle;
        float Now_Omega;
        float Now_Torque;
        float Now_MOS_Temperature;
        float Now_Rotor_Temperature;
        uint32_t Pre_Encoder;
        int32_t Total_Encoder;
        int32_t Total_Round;
    };

    /**
     * @brief 达妙电机常规源数据, MIT控制报文
     *
     */
    struct Struct_Motor_DM_CAN_Tx_Data_Normal_MIT {
        uint16_t Control_Angle_Reverse;
        uint8_t Control_Omega_11_4;
        uint8_t Control_Omega_3_0_K_P_11_8;
        uint8_t K_P_7_0;
        uint8_t K_D_11_4;
        uint8_t K_D_3_0_Control_Torque_11_8;
        uint8_t Control_Torque_7_0;
    } __attribute__((packed));

    /**
     * @brief 达妙电机传统模式源数据
     *
     */
    struct Struct_Motor_DM_CAN_Rx_Data_Normal {
        uint8_t CAN_ID : 4;
        uint8_t Control_Status_Enum : 4;
        uint16_t Angle_Reverse;
        uint8_t Omega_11_4;
        uint8_t Omega_3_0_Torque_11_8;
        uint8_t Torque_7_0;
        uint8_t MOS_Temperature;
        uint8_t Rotor_Temperature;
    } __attribute__((packed));

    class DM_Motor {
      private:
        // 清除电机错误信息, 传统模式有效
        uint8_t DM_Motor_CAN_Message_Clear_Error[8] = {
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};
        // 使能电机, 传统模式有效
        uint8_t DM_Motor_CAN_Message_Enter[8] = {
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
        // 失能电机, 传统模式有效
        uint8_t DM_Motor_CAN_Message_Exit[8] = {
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
        // 保存当前电机位置为零点, 传统模式有效
        uint8_t DM_Motor_CAN_Message_Save_Zero[8] = {
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

        // 初始化相关变量
        std::unique_ptr<CanManager> CAN_Handler;
        // 绑定的CAN
        can_frame *CAN_Manage_Object;
        // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
        uint16_t CAN_Rx_ID;
        // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量
        uint16_t CAN_Tx_ID;
        // 发送数据所的数据帧
        uint8_t Can_DLC = 8;
        // 最大位置, 与上位机控制幅值PMAX保持一致
        float Angle_Max;
        // 最大速度, 与上位机控制幅值VMAX保持一致
        float Omega_Max;
        // 最大扭矩, 与上位机控制幅值TMAX保持一致
        float Torque_Max;
        // 最大电流, 与上位机串口中上电打印电流保持一致
        float Current_Max;

        // 常量

        // 内部变量

        // 当前时刻的电机接收flag
        uint32_t Flag = 0;
        // 前一时刻的电机接收flag
        uint32_t Pre_Flag = 0;

        // 发送缓冲区
        uint8_t Tx_Data[8];

        // 读变量

        // // 电机状态
        // Enum_Motor_DM_Status Motor_DM_Status = Motor_DM_Status_DISABLE;
        // // 电机对外接口信息
        // Struct_Motor_DM_Rx_Data_Normal Rx_Data;

        // 写变量

        // // 角度, rad, 目标角度
        // float Control_Angle = 1.0f;
        // // 角速度, rad/s, MIT模式和速度模式是目标角速度, 其余模式是限幅
        // float Control_Omega = 1.0f;
        // // 扭矩, Nm, MIT模式是目标扭矩, EMIT模式无效, 其余模式是限幅
        // float Control_Torque = 0.0f;
        // // 电流, A, EMIT模式是限幅, 其余模式无效
        // float Control_Current = 0.0f;
        // // K_P, 0~500, MIT模式有效
        // float K_P = 0.5f;
        // // K_D, 0~5, MIT模式有效
        // float K_D = 1.0f;

      public:
        DM_Motor(std::string &can_interface,
                 uint8_t __CAN_Rx_ID,
                 uint8_t __CAN_Tx_ID,
                 float __Angle_Max,
                 float __Omega_Max,
                 float __Torque_Max,
                 float __Current_Max);
        void Output(float Control_Angle,
                    float Control_Omega,
                    float Control_Torque,
                    float K_P,
                    float K_D);
        void Data_Process();
        // 电机使能
        bool Motor_Enable();
        // 电机失能
        bool Motor_Disable();

        // 接收参数
        Struct_Motor_DM_Rx_Data_Normal Rx_Data;
    };
} // namespace Dog8Dof