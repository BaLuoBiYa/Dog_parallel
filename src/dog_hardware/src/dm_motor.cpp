#include <dog_hardware/dm_motor.hpp>

namespace Dog8Dof {
    /**
     * @brief 电机初始化
     *
     * @param can_interface 绑定的CAN总线
     * @param __CAN_Rx_ID 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致,
     * 传统模式有效
     * @param __CAN_Tx_ID 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量,
     * 传统模式有效
     * @param __Motor_DM_Control_Method 电机控制方式
     * @param __Angle_Max 最大位置, 与上位机控制幅值PMAX保持一致, 传统模式有效
     * @param __Omega_Max 最大速度, 与上位机控制幅值VMAX保持一致, 传统模式有效
     * @param __Torque_Max 最大扭矩, 与上位机控制幅值TMAX保持一致, 传统模式有效
     */
    DM_Motor::DM_Motor(std::string &can_interface,
                       uint8_t __CAN_Rx_ID,
                       uint8_t __CAN_Tx_ID,
                       float __Angle_Max,
                       float __Omega_Max,
                       float __Torque_Max,
                       float __Current_Max) {

        CAN_Handler = std::make_unique<CanManager>(can_interface);
        CAN_Rx_ID = __CAN_Rx_ID;
        CAN_Tx_ID = __CAN_Tx_ID;
        Angle_Max = __Angle_Max;
        Omega_Max = __Omega_Max;
        Torque_Max = __Torque_Max;
        Current_Max = __Current_Max;
    }

    /**
     * @brief 电机发送数据
     *
     */
    void DM_Motor::Output(float Control_Angle,
                          float Control_Omega,
                          float Control_Torque,
                          float K_P,
                          float K_D) {
        // 电机控制
        Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *tmp_buffer =
            (Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *)Tx_Data;

        uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;

        tmp_angle = Math_Float_To_Int(Control_Angle, 0, Angle_Max, 0x7fff, (1 << 16) - 1);
        tmp_omega = Math_Float_To_Int(Control_Omega, 0, Omega_Max, 0x7ff, (1 << 12) - 1);
        tmp_torque =
            Math_Float_To_Int(Control_Torque, 0, Torque_Max, 0x7ff, (1 << 12) - 1);
        tmp_k_p = Math_Float_To_Int(K_P, 0, 500.0f, 0, (1 << 12) - 1);
        tmp_k_d = Math_Float_To_Int(K_D, 0, 5.0f, 0, (1 << 12) - 1);

        tmp_buffer->Control_Angle_Reverse = Math_Endian_Reverse_16(&tmp_angle, nullptr);
        tmp_buffer->Control_Omega_11_4 = tmp_omega >> 4;
        tmp_buffer->Control_Omega_3_0_K_P_11_8 =
            ((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8);
        tmp_buffer->K_P_7_0 = tmp_k_p & 0xff;
        tmp_buffer->K_D_11_4 = tmp_k_d >> 4;
        tmp_buffer->K_D_3_0_Control_Torque_11_8 =
            ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        tmp_buffer->Control_Torque_7_0 = tmp_torque & 0xff;

        CAN_Handler->CanSend(CAN_Tx_ID, Can_DLC, Tx_Data);
    }

    /**
     * @brief 数据处理过程,CAN数据在这个函数中读到可优化
     *
     */
    void DM_Motor::Data_Process() {
        // 数据处理过程
        int32_t delta_encoder;
        uint16_t tmp_encoder, tmp_omega, tmp_torque;

        can_frame frame;
        CAN_Handler->CanRead(frame);

        Struct_Motor_DM_CAN_Rx_Data_Normal *tmp_buffer =
            (Struct_Motor_DM_CAN_Rx_Data_Normal *)frame.data;

        // 电机ID不匹配, 则不进行处理
        if (tmp_buffer->CAN_ID != (CAN_Tx_ID & 0x0f)) {
            return;
        }

        // 处理大小端
        Math_Endian_Reverse_16((void *)&tmp_buffer->Angle_Reverse, &tmp_encoder);
        tmp_omega =
            (tmp_buffer->Omega_11_4 << 4) | (tmp_buffer->Omega_3_0_Torque_11_8 >> 4);
        tmp_torque =
            ((tmp_buffer->Omega_3_0_Torque_11_8 & 0x0f) << 8) | (tmp_buffer->Torque_7_0);

        Rx_Data.Control_Status = static_cast<Enum_Motor_DM_Control_Status_Normal>(
            tmp_buffer->Control_Status_Enum);

        // 计算圈数与总角度值
        delta_encoder = tmp_encoder - Rx_Data.Pre_Encoder;
        if (delta_encoder < -(1 << 15)) {
            // 正方向转过了一圈
            Rx_Data.Total_Round++;
        } else if (delta_encoder > (1 << 15)) {
            // 反方向转过了一圈
            Rx_Data.Total_Round--;
        }
        Rx_Data.Total_Encoder =
            Rx_Data.Total_Round * (1 << 16) + tmp_encoder - ((1 << 15) - 1);

        // 计算电机本身信息
        Rx_Data.Now_Angle =
            (float)(Rx_Data.Total_Encoder) / (float)((1 << 16) - 1) * Angle_Max * 2.0f;
        Rx_Data.Now_Omega =
            Math_Int_To_Float(tmp_omega, 0x7ff, (1 << 12) - 1, 0, Omega_Max);
        Rx_Data.Now_Torque =
            Math_Int_To_Float(tmp_torque, 0x7ff, (1 << 12) - 1, 0, Torque_Max);
        Rx_Data.Now_MOS_Temperature = tmp_buffer->MOS_Temperature + CELSIUS_TO_KELVIN;
        Rx_Data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature + CELSIUS_TO_KELVIN;

        // 存储预备信息
        Rx_Data.Pre_Encoder = tmp_encoder;
    }

    bool DM_Motor::Motor_Enable() {
        CAN_Handler->CanSend(CAN_Tx_ID, Can_DLC, DM_Motor_CAN_Message_Enter);

        return true;
    }

    bool DM_Motor::Motor_Disable() {
        CAN_Handler->CanSend(CAN_Tx_ID, Can_DLC, DM_Motor_CAN_Message_Exit);

        return true;
    }

} // namespace Dog8Dof