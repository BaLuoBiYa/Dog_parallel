# Dog_Parallel —— 8自由度并联机器狗运动控制

## 架构概览

### dog_tools
提供帮助函数和统一类型定义 \
输入：无  \
输出：无

`以上为帮助函数，不加入控制回路`
---

### dog_trajectory
用于离线计算足端轨迹 \
输入：目标步态，当前足端位置 \
输出：足端轨迹

### dog_controller
用于跟踪并微调足端轨迹 \
输入：离线计算的足端轨迹，机体状态，（环境感知）（未完成） \
输出：电机指令

### dog_hardware
用于提供硬件接口 \
输入：电机指令 \
输出：电机控制指令（RS485/CAN）

`以上为高频控制回路，由ros_control管理，运行于高频`
---

## dog_remote
用于和决策层交互 \
输入：抽象控制指令 \
输出：目标步态

<!-- ## dog_voice(进行中)
基于Qwen3 TTS的文字转语音模块 \
输入：字符串 \
输出：音频流
> **Note:**该部分的核心推理框架参照\
> https://github.com/predict-woo/qwen3-tts.cpp -->

<!-- ## dog_handler(未完成)
用于针对不同的环境设置不同的处理策略,需要被`dog_interface`调用 \
输入：无 \
输出：目标步态 -->

# 已知问题：
## 1.Ubuntu 22.04版本的yaml-cpp库为0.7.0,无法正常安装，需要手动安装0.8.0以上版本
```bash
cd Dog_paraller
git clone https://github.com/jbeder/yaml-cpp.git
理论上进行到此步即可，clocon会自动将新库加入依赖

cd yaml-cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
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
LF: phaseOffset=0.5
LR: phaseOffset=0.0
RF: phaseOffset=0.0
RR: phaseOffset=0.5
```

### dog_hardware — 硬件接口层
提供 ROS2 `HardwareInterface`，将关节命令转为 RS485 电机指令。


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
<<<<<<< HEAD
sudo apt install ros-<distro>-pinocchio

=======
cd Dog_parallel
colcon build --symlink-install \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
                 -DCMAKE_POSITION_INDEPENDENT_CODE=ON
source install/setup.sh
>>>>>>> main
```

