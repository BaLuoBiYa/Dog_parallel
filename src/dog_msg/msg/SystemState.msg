# 系统整体监控状态
float32         cpu_usage           # CPU 使用率百分比 (0.0 ~ 100.0)
float32         cpu_temperature     # CPU/SoC 温度 (°C)，-1 表示读取失败
MotorState[8]   motors              # 8 个电机状态
float32         control_frequency   # 主控制回路实时频率 (Hz)
builtin_interfaces/Time timestamp   # 状态采集时间戳
