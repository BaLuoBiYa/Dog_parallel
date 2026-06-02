# 机器狗运动控制重构说明

> 对比基线：`a403c26` (2026-04-27)  
> 共计 **29 个文件，+3493 / -3816 行**

---

## 核心思路

从**预生成轨迹 → 时间插值播放**架构，演进为**每控制周期基于最新状态在线重规划**的闭环架构。

- 所有轨迹不再提前一次性生成，而是在每个控制周期内基于剩余时间和最新传感器反馈实时计算
- Walk 步态支持周期自循环，无需上层控制器管理周期间衔接
- Stand 步态支持在线目标切换，到达后自动 HOLD

---

## 一、`dog_tools` — 五次多项式工具层

### 1.1 `QuinticPolynomial` (`quinticPolynomial.hpp`)

| 接口 | 说明 |
|---|---|
| `computeCoefficients(startState[3], finishState[3], T)` | 计算五次多项式系数 `c0~c5` |
| `sampleTrajectory(coeff, tStart, tEnd, numPoints, positions, velocities)` | 从系数采样指定时间段的点 |
| `solveMidVel1D(startState[2], finishState[2], midPos, T1, T2)` | 一维中间速度求解 |
| `solveMidAcc1D(startState[3], finishState[3], midState[2], T1, T2)` | 一维中间加速度求解 |
| `evaluatePos(coeff, t)` / `evaluateVel` / `evaluateAcc` | 解析求位置/速度/加速度 |

多项式形式：$p(t) = c_0 + c_1 t + c_2 t^2 + c_3 t^3 + c_4 t^4 + c_5 t^5$

---

## 二、`dog_trajectory` — 轨迹生成核心层

### 2.1 `WalkTrajectory` (`walkTrajectory.hpp` / `.cpp`)

**X/Z 轴完全解耦，每周期在线重规划。**

#### 内部类：`SwingTrajectory`

摆动相轨迹生成器，管理离地到落地的全过程。

```
update() 每周期流程：
  1. t_elapsed += dt; T_rem = T_swing_nominal - t_elapsed
  2. 若 T_rem ≤ eps → 落地，返回
  3. 若 T_rem < T_min(20ms) → 锁定落脚点
  4. 滑动更新 X 目标：
     p_raw = v_des * T_stance/2 + k*(v_des - v_act)    ← Raibert 落脚点
     p_target += α * (p_raw - p_target)                  ← 一阶低通滤波
  5. 获取起点状态（优先传感器反馈，其次从上一多项式反算）
  6. X 轴：起点+剩余时间 → 五次多项式重规划 → 取 dt 时刻值
  7. Z 轴：光滑弧线
     z(φ) = z₀ + Δz·s(φ) - H·a(φ)
     s(φ) = 10φ³-15φ⁴+6φ⁵  (quintic smoothstep)
     a(φ) = 16φ²(1-φ)²      (归一化拱形)
```

- `smoothArcZ(φ)` 保证零速起落、C² 连续
- Z=0 为机身，抬腿方向 Z 减小（靠近机身）

#### 内部类：`StanceTrajectory`

- 落地瞬间记录机体坐标 `(x_land_body, z_land_body)`
- 支撑期间累加机体位移 `body_disp_x_ += bodyVelX * dt`
- 输出：`xDes = x_land_body - body_disp_x_`（世界静止 → 机体系后退）
- 离地判定：名义时间 / 最大时间 / 跟踪误差过大

#### 生命周期

```
setTarget(stepLength, stepHeight, baseHeight, totalTime, phaseOffset)
  → 更新 vDes = stepLength / totalTime

generate(currentTip, now, SWING)
  → 初始化 SwingTrajectory，以当前足端为起点

get(now, &currentTip)  ← 每控制周期循环
  → swing_.update() 或 stance_.update()
  → 自动检测 landed → stance 切换
  → 自动检测 lifted → swing 切换（walk 自循环）
```

### 2.2 `StandTrajectory` (`standTrajectory.hpp` / `.cpp`)

两阶段：**TRANSITION**（过渡）→ **HOLD**（保持）。

#### 生命周期

```
setTarget(tipX, tipZ)

generate(currentTip, now)
  → 若已在目标邻域 → 直接 HOLD
  → 否则初始化 TRANSITION：计算步高、初始五次多项式系数

get(now, &currentTip)  ← 每控制周期循环
  TRANSITION 阶段：
    - t_elapsed += dt; T_rem = T_transition - t_elapsed
    - 传感器反馈 / 轨迹反算获取当前状态
    - X 轴：以当前状态 + 剩余时间重规划五次多项式
    - Z 轴：光滑弧线（smoothArcZ，复用 SwingTrajectory 公式）
    - 到达判定 → 切换 HOLD

  HOLD 阶段：
    - 恒输出目标位置 (targetTipX, targetTipZ)，vel = (0, 0)
    - finished_ = true
```

### 2.3 `BaseTrajectory` (`base.hpp`)

```cpp
class BaseTrajectory {
public:
    BaseTrajectory(const std::string &configPath, rclcpp::Duration period);
    virtual ~BaseTrajectory() = default;

    // 每周期获取轨迹点（传感器反馈可选）
    virtual TrajectoryPoint get(const rclcpp::Time &currentTime,
                                const Body *currentTip = nullptr) = 0;

    void start(rclcpp::Time startTime);      // 非虚
    virtual bool reloadParameters() = 0;
    bool finished();

protected:
    const std::string configPath_;
    YAML::Node configFile_;
    long startTimeNanoSec_;
    const long controlDurationNanoSec_;
    bool finished_ = false;
};
```

> `generate()` 不再是虚函数，由各派生类自行定义签名（不需要 `override`）

---

## 三、`dog_controller` — ROS2 控制框架层

### 3.1 `GaitManager` (`gaitManager.hpp` / `.cpp`)

每腿一个实例，统一步态管理。

```
init(pos, vel, tau)                    → 从电机编码器建立腿模型
setStandGait(tipX, tipZ)               → 切换到站立（调用 standTrajectory_->generate）
setWalkGait(stepLen, stepH, baseH, T, phaseOffset) → 切换到步行（调用 walkTrajectory_->generate）

update(pos, vel, tau)  ← 每控制周期：
  1. leg_.updateMotor(pos, vel, tau)            → 编码器 → 足端位姿
  2. currentTrajectory_->get(now, &currentTip_) → 在线重规划（闭环）
  3. leg_.setTip(pos, vel, tau)                 → 逆运动学
  4. leg_.updateKp/Kd(kp, kd)                   → PD 增益
  5. 若 walk 且 finished → generate 续接下一周期（自循环）

isFinished()
  - walk: 永远返回 false（内部自循环）
  - stand: 返回 isHolding()
```

### 3.2 `TrajectoryTracker` (`trajectoryTracker.hpp` / `.cpp`)

ROS2 ControllerInterface 实现，简化对比：

| 项目 | 旧版 | 新版 |
|---|---|---|
| `update()` 行数 | ~250 行 | ~25 行 |
| `ControlMode` | STAND / WALK / JUMP | STAND / WALK |
| 命令处理 | 缓存( `pendingCmd_` ) + 完成后应用 | **立即 `applyCommand()`** |
| 去重逻辑 | `isSameWalkCommand()` + `lastWalkParams_` | 删除（GaitManager 自循环） |
| 首帧延迟 | `initialStandPending_` + `start(syncStart)` | 删除（`generate` 已初始化） |
| `start()` 调用 | ~30 处 | 全部删除 |

**update() 主循环**：
```
1. 若 newCmd → applyCommand(msg)  ← 立即切换步态
2. updateAllGaitManagers()         ← 四条腿同步闭环重规划
3. setAllJointCommands()           ← 下发到硬件接口
```

---

## 四、测试层

### 4.1 `walkTest.cpp`

- 控制频率 300Hz
- 6 段参数在线切换：前进→加速→小步幅→原地踏步→后退→停止
- 14 个完整周期，无传感器反馈（纯开环轨迹验证）

### 4.2 `standTest.cpp`

- 控制频率 300Hz
- 3 段目标切换：`(50,240) → (20,270) → (0,250)`
- 每段 5s transition + HOLD 观察

### 4.3 `visual.py`

- 统一 CSV 列格式：`t_s, pos_x, pos_z, vel_x, vel_z, cycle, stepLength_mm, totalTime_s`
- 自动检测 Walk/Stand 数据格式

---

## 五、统一 CSV 输出列

```csv
t_s, pos_x, pos_z, vel_x, vel_z, cycle, stepLength_mm, totalTime_s
```

| 列 | Walk 含义 | Stand 含义 |
|---|---|---|
| `cycle` | 周期编号 | 目标段索引 |
| `stepLength_mm` | 当前步长 (mm) | 当前目标 Z (mm) |
| `totalTime_s` | 单周期时长 (s) | 过渡时长，HOLD=0 |

---

## 六、关键架构变化图

```
旧版（预生成播放）：
  ┌──────────────────────────────────────────────────────────────┐
  │ setTarget → generate(预生成全部帧数组) → start(记录起始时间) │
  │            → get(now) [按时间插值播放]                        │
  │ 周期结束 → 外部检测 finished → 重新 setWalkGait → generate  │
  │            → start → 间隙(1~2s 停顿)                         │
  └──────────────────────────────────────────────────────────────┘

新版（每周期在线重规划）：
  ┌────────────────────────────────────────────────────────────┐
  │ setTarget → generate(仅初始化起点，不生成数组)               │
  │                                                              │
  │ ┌─── 每控制周期循环 ────────────────────────────────────┐   │
  │ │ get(now, &currentTip)  ← 传入传感器真实反馈             │   │
  │ │   → 获取当前状态 (传感器优先 / 轨迹反算)                │   │
  │ │   → 以(T_rem)重规划五次多项式 + 光滑弧线               │   │
  │ │   → 返回本周期的 pos, vel                             │   │
  │ │   → 逆运动学 → 关节角度命令                             │   │
  │ │   → Walk: 自动检测 finished → generate 续接 (无缝)     │   │
  │ │   → Stand: 到达目标 → HOLD                             │   │
  │ └───────────────────────────────────────────────────────┘   │
  └────────────────────────────────────────────────────────────┘
```

---

## 七、文件清单

### 修改的文件（29个）

| 层 | 文件 | 主要变更 |
|---|---|---|
| tools | `quinticPolynomial.hpp` | 新增 `computeCoefficients` / `sampleTrajectory` / `solveMid*1D` / `evaluate*` |
| tools | `type.hpp` | 清理未使用的 `Trajectory` 字段 |
| trajectory | `base.hpp` | 接口精简，`get()` 接收 `const Body*` |
| trajectory | `walkTrajectory.hpp` | 全新实现，新增 `SwingTrajectory` / `StanceTrajectory` |
| trajectory | `walkTrajectory.cpp` | 全新实现，每周期在线重规划 |
| trajectory | `standTrajectory.hpp` | 全新实现，TRANSITION / HOLD 两阶段 |
| trajectory | `standTrajectory.cpp` | 全新实现，在线重规划 |
| trajectory | `jumpTrajectory.hpp/.cpp` | 注释（未适配新接口） |
| trajectory | `jumpTrajectoryLegacy.hpp/.cpp` | 对接口变更的兼容调整 |
| controller | `gaitManager.hpp` | 移除 Jump，接口与新 API 对齐 |
| controller | `gaitManager.cpp` | 实现简化，Walk 自循环 |
| controller | `trajectoryTracker.hpp` | 移除旧状态机，简化成员 |
| controller | `trajectoryTracker.cpp` | 大幅简化（-481 行），update 核心 ~25 行 |
| controller | `trajectoryTrackerSingle.hpp/.cpp` | 同步更新 |
| test | `walkTest.cpp` | 多周期在线重规划测试 |
| test | `standTest.cpp` | 多目标在线切换测试 |
| test | `visual.py` | 通用可视化，自动检测 Walk/Stand |
| build | `CMakeLists.txt` | 取消注释 standTest，安装目标更新 |
