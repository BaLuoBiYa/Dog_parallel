#include <dog_hardware/dji_motor.hpp>

namespace Dog8Dof {

    DJI_Motor_2006::DJI_Motor_2006(
        std::string &can_interface,
        Enum_Motor_DJI_ID __CAN_Rx_ID,
        Enum_Motor_DJI_Control_Method __Motor_DJI_Control_Method,
        float __Gearbox_Rate,
        float __Current_Max) {

        CAN_Handler = std::make_unique<CanManager>(can_interface);
        CAN_Rx_ID = __CAN_Rx_ID;
        Motor_DJI_Control_Method = __Motor_DJI_Control_Method;
        Gearbox_Rate = __Gearbox_Rate;
        Current_Max = __Current_Max;
        Tx_Data = allocate_tx_data(__CAN_Rx_ID);
        if (CAN_Rx_ID == Motor_DJI_ID_0x201 || CAN_Rx_ID == Motor_DJI_ID_0x202 ||
            CAN_Rx_ID == Motor_DJI_ID_0x203 || CAN_Rx_ID == Motor_DJI_ID_0x204) {
            CAN_Tx_ID = 0x200;
        } else if (CAN_Rx_ID == Motor_DJI_ID_0x205 || CAN_Rx_ID == Motor_DJI_ID_0x206 ||
                   CAN_Rx_ID == Motor_DJI_ID_0x207 || CAN_Rx_ID == Motor_DJI_ID_0x208) {
            CAN_Tx_ID = 0x1ff;
        }

        // PID初始化, 参数需要根据实际情况调整
        PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }

    DJI_Motor_2006::~DJI_Motor_2006() {
    }

    void DJI_Motor_2006::Data_Process() {
        // 数据处理过程
        int16_t delta_encoder; // 編碼器差值
        uint16_t tmp_encoder;
        uint16_t tmp_clear_encoder = 0; // 初始化清零编码器
        int16_t tmp_omega, tmp_current;

        can_frame frame;
        CAN_Handler->CanRead(frame);

        Struct_Motor_DJI_CAN_Rx_Data *tmp_buffer =
            (Struct_Motor_DJI_CAN_Rx_Data *)frame.data;

        // 处理大小端
        Math_Endian_Reverse_16((void *)&tmp_buffer->Encoder_Reverse,
                               (void *)&tmp_encoder);
        Math_Endian_Reverse_16((void *)&tmp_buffer->Omega_Reverse, (void *)&tmp_omega);
        Math_Endian_Reverse_16((void *)&tmp_buffer->Current_Reverse,
                               (void *)&tmp_current);
        // 清除标志位
        if (Angle_Clear_Flag == 1) {
            tmp_clear_encoder = tmp_encoder;
            Angle_Clear_Flag = 0;
        }
        // 计算圈数与总编码器值
        delta_encoder = tmp_encoder - Rx_Data.Pre_Encoder;
        if (delta_encoder < -Encoder_Num_Per_Round / 2) {
            // 正方向转过了一圈
            Rx_Data.Total_Round++;
        } else if (delta_encoder > Encoder_Num_Per_Round / 2) {
            // 反方向转过了一圈
            Rx_Data.Total_Round--;
        }
        Rx_Data.Total_Encoder =
            Rx_Data.Total_Round * Encoder_Num_Per_Round + tmp_encoder - tmp_clear_encoder;

        // 计算电机本身信息
        Rx_Data.Now_Angle = (float)Rx_Data.Total_Encoder / (float)Encoder_Num_Per_Round *
                            2.0f * M_PI / Gearbox_Rate;
        Rx_Data.Now_Omega = (float)tmp_omega * RPM_TO_RADPS / Gearbox_Rate;
        Rx_Data.Now_Current = tmp_current / Current_To_Out;
        Rx_Data.Now_Temperature = tmp_buffer->Temperature + CELSIUS_TO_KELVIN;

        // 存储预备信息
        Rx_Data.Pre_Encoder = tmp_encoder;
    }

    /**
     * @brief 分配CAN发送缓冲区
     *
     * @param hcan CAN编号
     * @param __CAN_ID CAN ID
     * @param __DJI_Motor_Driver_Version 大疆驱动版本, 当且仅当当前被分配电机为6020,
     * 且是电流驱动新版本时选2023, 否则都是default
     * @return uint8_t* 缓冲区指针
     */
    uint8_t *DJI_Motor_2006::allocate_tx_data(Enum_Motor_DJI_ID __CAN_ID) {
        uint8_t *tmp_tx_data_ptr;
        switch (__CAN_ID) {
        case (Motor_DJI_ID_0x201): {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);

            break;
        }
        case (Motor_DJI_ID_0x202): {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);

            break;
        }
        case (Motor_DJI_ID_0x203): {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);

            break;
        }
        case (Motor_DJI_ID_0x204): {
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);

            break;
        }
        case (Motor_DJI_ID_0x205): {

            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[0]);
            break;
        }
        case (Motor_DJI_ID_0x206): {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[2]);
            break;
        }
        case (Motor_DJI_ID_0x207): {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[4]);
            break;
        }
        case (Motor_DJI_ID_0x208): {
            tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[6]);
            break;
        }
        }
        return (tmp_tx_data_ptr);
    }

    /**
     * @brief 电机数据输出到CAN总线发送缓冲区
     *
     */
    void DJI_Motor_2006::Output() {
        Tx_Data[0] = (int16_t)Out >> 8;
        Tx_Data[1] = (int16_t)Out;
    }

    /**
     * @brief 计算PID，发送函数
     *  待优化，每次一个新电机都会新建一个变量类似CAN1_0x1ff_Tx_Data，会造成资源浪费
     */
    void DJI_Motor_2006::TIM_Calculate_PeriodElapsedCallback() {
        PID_Calculate();

        float tmp_value = Target_Current + Feedforward_Current;
        Math_Constrain(&tmp_value, -Current_Max, Current_Max);
        Out = tmp_value * Current_To_Out;

        Output();

        Feedforward_Current = 0.0f;
        Feedforward_Omega = 0.0f;

        if (CAN_Tx_ID == 0x200) {
            CAN_Handler->CanSend(CAN_Tx_ID, 8, CAN1_0x200_Tx_Data);
        }
        if (CAN_Tx_ID == 0x1ff) {
            CAN_Handler->CanSend(CAN_Tx_ID, 8, CAN1_0x1ff_Tx_Data);
        }
    }

    /**
     * @brief 计算PID
     *
     */
    void DJI_Motor_2006::PID_Calculate() {
        switch (Motor_DJI_Control_Method) {
        case (Motor_DJI_Control_Method_CURRENT): {
            break;
        }
        case (Motor_DJI_Control_Method_OMEGA): {
            PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
            PID_Omega.Set_Now(Rx_Data.Now_Omega);
            PID_Omega.TIM_Calculate_PeriodElapsedCallback();

            Target_Current = PID_Omega.Get_Out();

            break;
        }
        case (Motor_DJI_Control_Method_ANGLE): {
            PID_Angle.Set_Target(Target_Angle);
            PID_Angle.Set_Now(Rx_Data.Now_Angle);
            PID_Angle.TIM_Calculate_PeriodElapsedCallback();

            Target_Omega = PID_Angle.Get_Out();

            PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
            PID_Omega.Set_Now(Rx_Data.Now_Omega);
            PID_Omega.TIM_Calculate_PeriodElapsedCallback();

            Target_Current = PID_Omega.Get_Out();

            break;
        }
        default: {
            Target_Current = 0.0f;

            break;
        }
        }
    }
} // namespace Dog8Dof