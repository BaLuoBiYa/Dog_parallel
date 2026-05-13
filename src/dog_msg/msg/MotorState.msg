# 单个电机实时状态
string  name            # 电机名称 (LF_Front, RF_Rear, ...)
float32 position        # 当前位置 (rad)
float32 velocity        # 当前速度 (rad/s)
float32 torque          # 当前力矩 (N·m)
int8    temperature     # 电机温度 (°C)
