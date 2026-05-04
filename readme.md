# Dog_Parallel —— 8自由度并联机器狗运动控制

## 架构概览

```
决策层（dog_remote）
  抽象控制指令 (stand/walk, 步长, 步高, ...)
      │
      ▼
高频控制回路 (ros2_control 管理)
    dog_controller                                     
      TrajectoryTracker::update()  ← 每周期核心入口     
        ├─ GaitManager::update()  ← 电机反馈 + 重规划   
        │    ├─ WalkTrajectory / StandTrajectory        
        │    │    └─ get(now, &currentTip) [在线重规划] 
        │    └─ Leg2Dof 逆运动学                        
        └─ setAllJointCommands()  → 关节位置/速度/力矩/Kp/Kd     
                                                               
    dog_hardware                                               
      └─ RS485下发                                             
```


## 核心设计原则

1. **每周期在线重规划**：不再预生成完整轨迹数组，每控制周期基于剩余时间 + 最新传感器反馈实时计算期望足端状态
2. **X/Z 轴完全解耦**：X 轴五次多项式精确到位置，Z 轴独立光滑弧线（quintic smoothstep + 拱形），零速起落 C² 连续
3. **闭环抗干扰**：`get(now, &currentTip)` 接收电机编码器反馈的足端真实位姿，每次重规划基于最新状态
4. **Walk 自循环**：步态管理器内部自动检测周期结束并续接下一周期，控制器层无需管理衔接
5. **Z=0 为机身坐标原点**，正 Z 向下（远离机身），抬腿 Z 减小


## 包说明

### dog_tools — 工具层
提供五次多项式求解和类型定义。

| 组件 | 说明 |
|---|---|
| `quinticPolynomial.hpp` | 五次多项式：`computeCoefficients` / `sampleTrajectory` / `evaluate*` / `solveMid*1D` |
| `type.hpp` | 统一类型：`TrajectoryPoint`, `Body`, `Motor`, `Gait` |

### dog_trajectory — 轨迹生成层
每周期在线重规划的足端轨迹生成。

| 类 | 说明 |
|---|---|
| `WalkTrajectory` | 步行步态：`SwingTrajectory`（摆动相）+ `StanceTrajectory`（支撑相），Raibert 落脚点 + 低通滤波 |
| `StandTrajectory` | 站立步态：TRANSITION（过渡）→ HOLD（保持），在线目标切换 |
| `BaseTrajectory` | 基类：`get(now, currentTip=nullptr)` 虚接口 |

**WalkTrajectory 摆动相 Z 轴公式**：
$$z(\varphi) = z_{\text{start}} + (z_{\text{target}} - z_{\text{start}}) \cdot s(\varphi) - H \cdot a(\varphi)$$
其中 $s(\varphi) = 10\varphi^3 - 15\varphi^4 + 6\varphi^5$（quintic smoothstep），$a(\varphi) = 16\varphi^2(1-\varphi)^2$（归一化拱形），$\varphi \in [0,1]$

**StanceTrajectory 支撑相机体坐标输出**：
$$x_{\text{des}} = x_{\text{land,body}} - \Sigma (v_{\text{body}} \cdot \Delta t)$$

### dog_controller — 控制框架层
ROS2 `ControllerInterface` 实现，管理四条腿的实时控制。

| 组件 | 说明 |
|---|---|
| `TrajectoryTracker` | 控制器主类：`update()` 每周期 ~25 行，处理命令 + 更新步态 + 下发关节 |
| `GaitManager` | 每腿一个实例：`init` → `setStandGait`/`setWalkGait` → `update` 循环 |
| `Leg2Dof` | 二自由度并联腿模型：逆运动学、PD 增益 |

**相位偏移配置**（对角小跑步态）：
```
LF: phaseOffset=0.5  (STANCE 起步，对侧 RF 在 SWING)
LR: phaseOffset=0.0  (SWING 起步，对侧 RR 在 STANCE)
RF: phaseOffset=0.0  (SWING 起步)
RR: phaseOffset=0.5  (STANCE 起步)
```

### dog_hardware — 硬件接口层
提供 ROS2 `HardwareInterface`，将关节命令转为 RS485/CAN 电机指令。


## 测试

| 测试 | 说明 |
|---|---|
| `walkTest` | 14 周期连续，6 段参数在线切换，300Hz |
| `standTest` | 3 段目标在线切换，TRANSITION → HOLD 验证 |
| `visual.py` | 通用可视化，自动识别 Walk/Stand CSV，输出静态图 + webp 动画 |
| `legTest` / `legCli` | 单腿运动学/逆运动学测试 |

**统一 CSV 列**：`t_s, pos_x, pos_z, vel_x, vel_z, cycle, stepLength_mm, totalTime_s`

> 详细重构说明见 `/doc/refactor_summary.md`


## 构建

```bash
cd Dog_parallel
colcon build --symlink-install \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
                 -DCMAKE_POSITION_INDEPENDENT_CODE=ON
source install/setup.sh
```
