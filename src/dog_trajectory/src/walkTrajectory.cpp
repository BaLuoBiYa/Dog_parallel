#include "dog_trajectory/walkTrajectory.hpp"
#include "dog_tools/quinticPolynomial.hpp"
#include <cmath>

namespace Dog8Dof {

    // ===================================================================
    //  WalkTrajectory 构造 & 参数
    // ===================================================================
    WalkTrajectory::WalkTrajectory(const std::string &configPath, rclcpp::Duration period)
        : BaseTrajectory(configPath, period) {
        if (!reloadParameters()) {
            throw std::invalid_argument("Could not load WalkTrajectory Config YAML!");
        }
        stepHeight_ = 0.0f;
        stepLength_ = 0.0f;
        baseHeight_ = 0.0f;
        swingTime_ = 0.5f;
        stanceTime_ = 0.5f;
    }

    bool WalkTrajectory::reloadParameters() {
        configFile_ = YAML::LoadFile(configPath_);
        try {
            parameters_ = configFile_.as<parameters_t>();
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(logger_, "YAML Error: %s", e.what());
            return false;
        }

        if (parameters_.swingDutyCycle >= 1.0f) {
            RCLCPP_ERROR(logger_, "Swing duty cycle must be less than 1!");
            return false;
        }
        if (parameters_.swingDutyCycle <= 0.0f) {
            RCLCPP_ERROR(logger_, "Swing duty cycle must be greater than 0!");
            return false;
        }
        return true;
    }

    bool WalkTrajectory::setTarget(float stepLength,
                                   float stepHeight,
                                   float baseHeight,
                                   float totalTime,
                                   float phaseOffset) {
        if (stepHeight <= 0.0f) {
            RCLCPP_ERROR(logger_, "Step height must be greater than 0!");
            return false;
        }
        if (baseHeight > Const::BODY_HEIGHT_MAX || baseHeight < Const::BODY_HEIGHT_MIN) {
            RCLCPP_ERROR(logger_,
                         "Base height must be between %f and %f",
                         static_cast<double>(Const::BODY_HEIGHT_MAX),
                         static_cast<double>(Const::BODY_HEIGHT_MIN));
            return false;
        }

        stepLength_ = stepLength;
        stepHeight_ = stepHeight;
        baseHeight_ = baseHeight;
        swingTime_ = totalTime * parameters_.swingDutyCycle;
        stanceTime_ = totalTime - swingTime_;
        phaseOffset_ = phaseOffset;

        // 计算期望前进速度（腿坐标系）
        vDes_ = (totalTime > 0.0f) ? stepLength_ / totalTime : 0.0f;

        return true;
    }

    TrajectoryPoint WalkTrajectory::generate(const Body &currentTip,
                                             const rclcpp::Time now) {
        finished_ = false;
        lastUpdateTime_ = now;
        gaitEpoch_ = now; // 锁定全局相位纪元
        lastTipX_ = currentTip.pos[0];

        // 以当前足端位置作为摆动相起点，初始化摆动相
        const float zStart = currentTip.pos[2];
        const float zTarget = baseHeight_;
        const float xStart = currentTip.pos[0];
        // 假设从站立开始，速度/加速度均为 0
        const float vxStart = 0.0f;
        const float axStart = 0.0f;

        // ---- 相位偏移处理 ----
        // phaseOffset_ ∈ [0, 1)：表示本腿在全局周期中的起始相位点
        //
        // 当 phaseOffset_ = 0.0 时：从 SWING 开始，先摆后撑，完整周期
        // 当 phaseOffset_ = 0.5 时：从 STANCE 开始，先撑后摆（对侧腿正在摆动）
        //
        // Stance 的初始时长按 phaseOffset_ 缩放：
        //   若 phaseOffset_ < swingDuty，直接按比例进入 SWING 的中间时刻
        //   若 phaseOffset_ >= swingDuty，从 STANCE 起步，剩余支撑时间 =
        //       stanceTime * (1 - (phaseOffset - swingDuty) / (1 - swingDuty))

        const float swingDuty = parameters_.swingDutyCycle;
        const float phase = std::fmod(phaseOffset_, 1.0f);

        // 根据 phaseOffset 自动推导初始步态，保证相位一致
        //   phase <  swingDuty → 从 SWING 开始（先摆后撑）
        //   phase >= swingDuty → 从 STANCE 开始（先撑后摆）
        const GaitPhase effectiveInitGait = (phase < swingDuty) ? SWING : STANCE;

        if (effectiveInitGait == SWING) {
            // SWING 起步：按 phase 比例跳过已过的摆动时间
            //   skipFraction = phase / swingDuty, clamp to [0,1]
            const float skipFraction =
                (swingDuty > 0.0f) ? std::max(0.0f, std::min(phase / swingDuty, 1.0f))
                                   : 0.0f;
            const float effectiveSwingT =
                std::max(swingTime_ * (1.0f - skipFraction), parameters_.minReplanTime);

            swing_.init(zStart,
                        zTarget,
                        stepHeight_,
                        effectiveSwingT,
                        parameters_.lowPassAlpha,
                        parameters_.minReplanTime,
                        xStart,
                        vxStart,
                        axStart,
                        stepLength_);
        } else {
            // STANCE 起步：按 (phase - swingDuty) / stanceDuty 比例跳过已过的支撑时间
            const float stanceDuty = 1.0f - swingDuty;
            const float stancePhase =
                (stanceDuty > 0.0f)
                    ? std::max(0.0f, std::min((phase - swingDuty) / stanceDuty, 1.0f))
                    : 0.0f;
            const float effectiveStanceT = stanceTime_ * (1.0f - stancePhase);

            stance_.init(xStart,
                         zStart,
                         effectiveStanceT,
                         parameters_.stanceTimeMin,
                         parameters_.stanceTimeMax);
        }
        gaitPhase_ = effectiveInitGait;

        // 返回初始点
        target_.pos[0] = xStart;
        target_.pos[1] = 0.0f;
        target_.pos[2] = zStart;
        target_.vel.setZero();
        target_.tau.setZero();
        target_.kp[0] = parameters_.walkKp;
        target_.kp[1] = parameters_.walkKp;
        target_.kd[0] = parameters_.walkKd;
        target_.kd[1] = parameters_.walkKd;

        return target_;
    }

    // ===================================================================
    //  get() — 每控制周期的主入口
    // ===================================================================
    TrajectoryPoint WalkTrajectory::get(const rclcpp::Time &currentTime,
                                        const Body *currentTip) {
        const float dt = (currentTime - lastUpdateTime_).seconds();
        if (dt <= 0.0f) {
            // 同一时刻多次调用，直接返回上次结果
            return target_;
        }
        lastUpdateTime_ = currentTime;

        // 速度估计：用足端 X 位置差分
        float vAct = vDes_; // 默认用期望速度
        if (currentTip) {
            vAct = (currentTip->pos[0] - lastTipX_) / dt;
            lastTipX_ = currentTip->pos[0];
        }

        float xDes = target_.pos[0];
        float zDes = target_.pos[2];
        float vxDes = 0.0f;
        float vzDes = 0.0f;

        // ---- 全局相位锁：基于绝对时间的确定性相位 ----
        // 所有腿接收相同的 currentTime，确保相位永不漂移
        const float totalPeriod = swingTime_ + stanceTime_;
        if (totalPeriod > 0.0f) {
            const double elapsed = (currentTime - gaitEpoch_).seconds();
            const double globalPhase =
                std::fmod(std::max(0.0, elapsed) / static_cast<double>(totalPeriod) +
                              static_cast<double>(phaseOffset_),
                          1.0);
            const float swingDuty = parameters_.swingDutyCycle;
            const GaitPhase clockPhase =
                (globalPhase < static_cast<double>(swingDuty)) ? SWING : STANCE;

            // 全局时钟认为应切换相位时，强制切换
            if (clockPhase != gaitPhase_) {
                if (clockPhase == STANCE) {
                    // 摆动→支撑：立即锁定落脚点并初始化支撑相
                    stance_.init(swing_.getTargetX(),
                                 swing_.getZTarget(),
                                 stanceTime_,
                                 parameters_.stanceTimeMin,
                                 parameters_.stanceTimeMax);
                } else {
                    // 支撑→摆动：从当前足端状态出发
                    const float zs = currentTip ? currentTip->pos[2] : zDes;
                    const float xs = currentTip ? currentTip->pos[0] : xDes;
                    const float vxs = currentTip ? currentTip->linerVel[0] : 0.0f;
                    swing_.init(zs,
                                baseHeight_,
                                stepHeight_,
                                swingTime_,
                                parameters_.lowPassAlpha,
                                parameters_.minReplanTime,
                                xs,
                                vxs,
                                0.0f,
                                stepLength_);
                }
                gaitPhase_ = clockPhase;
            }
        }

        switch (gaitPhase_) {
        // ---- 摆动相 ----
        case SWING: {
            swing_.update(dt,
                          vDes_,
                          vAct,
                          parameters_.horizontalPushGain,
                          currentTip,
                          xDes,
                          zDes,
                          vxDes,
                          vzDes,
                          stepLength_);

            if (swing_.isLanded()) {
                // 摆动→支撑 切换
                stance_.init(swing_.getTargetX(),
                             swing_.getZTarget(),
                             stanceTime_,
                             parameters_.stanceTimeMin,
                             parameters_.stanceTimeMax);
                gaitPhase_ = STANCE;
            }
        } break;

        // ---- 支撑相 ----
        case STANCE: {
            // 使用支撑相专用机体速度，保证支撑期间机体移动完整步长，
            // 使足端在机体系下从 +stepLength/2 移动到 -stepLength/2
            const float stanceBodyVel =
                (stanceTime_ > 0.0f) ? (stepLength_ / stanceTime_) : vDes_;
            stance_.update(dt, stanceBodyVel, currentTip, xDes, zDes);
            vxDes = 0.0f;
            vzDes = 0.0f;

            if (stance_.isLifted()) {
                // 支撑→摆动 切换
                const float zStart = currentTip ? currentTip->pos[2] : zDes;
                const float xStart = currentTip ? currentTip->pos[0] : xDes;
                const float vxStart = currentTip ? currentTip->linerVel[0] : 0.0f;
                const float axStart = 0.0f;

                swing_.init(zStart,
                            baseHeight_,
                            stepHeight_,
                            swingTime_,
                            parameters_.lowPassAlpha,
                            parameters_.minReplanTime,
                            xStart,
                            vxStart,
                            axStart,
                            stepLength_);

                gaitPhase_ = SWING;

                // 立即为第一帧摆动计算输出
                swing_.update(0.0f,
                              vDes_,
                              vAct,
                              parameters_.horizontalPushGain,
                              currentTip,
                              xDes,
                              zDes,
                              vxDes,
                              vzDes,
                              stepLength_);
            }
        } break;
        }

        // 组装输出
        target_.pos[0] = xDes;
        target_.pos[1] = 0.0f;
        target_.pos[2] = zDes;
        target_.vel[0] = vxDes;
        target_.vel[1] = 0.0f;
        target_.vel[2] = vzDes;
        target_.tau.setZero();
        target_.kp[0] = parameters_.walkKp;
        target_.kp[1] = parameters_.walkKp;
        target_.kd[0] = parameters_.walkKd;
        target_.kd[1] = parameters_.walkKd;

        return target_;
    }

    // ===================================================================
    //  SwingTrajectory 实现
    // ===================================================================
    void WalkTrajectory::SwingTrajectory::init(float zStart,
                                               float zTarget,
                                               float stepH,
                                               float swingT,
                                               float alpha,
                                               float minReplan,
                                               float xStart,
                                               float vxStart,
                                               float axStart,
                                               float stepLength) {
        H_ = stepH;
        T_swing_nominal_ = swingT;
        T_min_ = minReplan;
        alpha_ = alpha;
        t_elapsed_ = 0.0f;
        step_length_ = stepLength;

        z_start_ = zStart;
        z_target_ = zTarget;
        landed_ = false;
        foothold_locked_ = false;

        // 初始 X 系数：从起点到初始目标（Raibert 在 update 中第一次计算前，
        // 先用期望步长的一半作为初始落脚点）
        p_target_x_ = xStart; // 首次滤波由 update 填充

        // 初始多项式：匀速延伸至当前 p_target_x_（或保持原速）
        // 为安全起见，构建一个初始的常速/零加速多项式
        std::array<float, 3> start = {xStart, vxStart, axStart};
        std::array<float, 3> finish = {xStart, vxStart, axStart};
        // T=0 无法求解，使用一个极小的 T 构建初始安全多项式
        const float epsT = 1e-3f;
        last_coeffs_x_ = QuinticPolynomial::computeCoefficients(start, finish, epsT);
    }

    void WalkTrajectory::SwingTrajectory::update(float dt,
                                                 float vDes,
                                                 float vAct,
                                                 float pushGain,
                                                 const Body *currentTip,
                                                 float &xDes,
                                                 float &zDes,
                                                 float &vxDes,
                                                 float &vzDes,
                                                 float stepLength) {
        constexpr float eps = 1e-6f;

        // 1. 更新时间
        t_elapsed_ += dt;
        const float T_rem = T_swing_nominal_ - t_elapsed_;

        // 2. 落地判定
        if (T_rem <= eps || t_elapsed_ >= T_swing_nominal_) {
            xDes = p_target_x_;
            zDes = z_target_;
            vxDes = 0.0f;
            vzDes = 0.0f;
            landed_ = true;
            return;
        }

        // 3. 滑动更新 X 目标（Raibert 落脚点 + 低通滤波）
        if (!foothold_locked_) {
            if (T_rem < T_min_) {
                foothold_locked_ = true; // 接近落地，锁定落脚点
            } else {
                // Raibert 公式：p_raw = stepLength/2 + k*(v_des - v_act)
                const float p_raw_x = stepLength * 0.5f + pushGain * (vDes - vAct);

                // 一阶低通滤波
                p_target_x_ += alpha_ * (p_raw_x - p_target_x_);
            }
        }

        // 4. 获取起点状态（优先传感器，其次轨迹反算）
        float x0, vx0, ax0;
        if (currentTip) {
            x0 = currentTip->pos[0];
            vx0 = currentTip->linerVel[0];
            // 加速度从上一多项式反算以保证连续性
            ax0 = QuinticPolynomial::evaluateAcc(last_coeffs_x_, dt);
        } else {
            // 从上一轨迹反算在时刻 dt 的期望状态
            x0 = QuinticPolynomial::evaluatePos(last_coeffs_x_, dt);
            vx0 = QuinticPolynomial::evaluateVel(last_coeffs_x_, dt);
            ax0 = QuinticPolynomial::evaluateAcc(last_coeffs_x_, dt);
        }

        // 5. X 轴五次多项式重规划
        //    起点：(x0, vx0, ax0)  终点：(p_target_x_, 0, 0)  时长：T_rem
        if (T_rem > eps) {
            const std::array<float, 3> startState = {x0, vx0, ax0};
            const std::array<float, 3> finishState = {p_target_x_, 0.0f, 0.0f};
            last_coeffs_x_ =
                QuinticPolynomial::computeCoefficients(startState, finishState, T_rem);
        }

        // 在新多项式中取时刻 dt 的值
        xDes = QuinticPolynomial::evaluatePos(last_coeffs_x_, dt);
        vxDes = QuinticPolynomial::evaluateVel(last_coeffs_x_, dt);

        // 6. Z 轴独立光滑弧线
        const float phase = std::max(0.0f, std::min(1.0f, t_elapsed_ / T_swing_nominal_));
        zDes = smoothArcZ(phase);

        // Z 速度：解析求导，匹配 -H 符号（抬腿=Z减小）
        //   z(φ) = z₀ + Δz·s(φ) - H·a(φ)
        //   s'(φ) = 30φ²(1-φ)²
        //   a'(φ) = 32φ(1-φ)(1-2φ)
        const float dz = z_target_ - z_start_;
        const float ds = 30.0f * phase * phase * (1.0f - phase) * (1.0f - phase);
        const float da = 32.0f * phase * (1.0f - phase) * (1.0f - 2.0f * phase);
        const float dphase_dt = (T_swing_nominal_ > eps) ? 1.0f / T_swing_nominal_ : 0.0f;
        vzDes = (dz * ds - H_ * da) * dphase_dt;

        // 7. 二次落地判定：是否已在落点邻域
        if (T_rem < T_min_) {
            const float dx = p_target_x_ - xDes;
            const float dz2 = z_target_ - zDes;
            if (dx * dx + dz2 * dz2 < 10.0f) { // 10mm² 邻域
                xDes = p_target_x_;
                zDes = z_target_;
                vxDes = 0.0f;
                vzDes = 0.0f;
                landed_ = true;
            }
        }
    }

    float WalkTrajectory::SwingTrajectory::smoothArcZ(float phase) const {
        // Z=0 为机身，正 Z 向下（远离机身）。
        // 抬腿时 Z 减小（靠近机身），因此拱形项为负：
        // z(φ) = z_start + (z_target - z_start) * s(φ) - H * a(φ)
        //
        // s(φ) = 10φ³ - 15φ⁴ + 6φ⁵   —— quintic smoothstep (C², s(0)=0, s(1)=1,
        //                                                s'(0)=s'(1)=s''(0)=s''(1)=0)
        // a(φ) = 16φ²(1-φ)²           —— 归一化拱形 (max=1 at φ=0.5, a(0)=a(1)=0,
        //                                                a'(0)=a'(1)=0)
        const float p2 = phase * phase;
        const float p3 = p2 * phase;
        const float p4 = p3 * phase;
        const float p5 = p4 * phase;

        const float s = 10.0f * p3 - 15.0f * p4 + 6.0f * p5;
        const float oneMinusPhase = 1.0f - phase;
        const float a = 16.0f * p2 * oneMinusPhase * oneMinusPhase;

        // 抬腿方向：Z 减小（靠近机身） → -H*a
        return z_start_ + (z_target_ - z_start_) * s - H_ * a;
    }

    // ===================================================================
    //  StanceTrajectory 实现
    // ===================================================================
    void WalkTrajectory::StanceTrajectory::init(
        float xLand, float zLand, float stanceT, float tMin, float tMax) {
        foot_pos_land_ = Eigen::Vector2f(xLand, zLand);
        body_disp_x_ = 0.0f;
        T_stance_nominal_ = stanceT;
        T_stance_min_ = tMin;
        T_stance_max_ = tMax;
        t_stance_ = 0.0f;
        lifted_ = false;
    }

    void WalkTrajectory::StanceTrajectory::update(
        float dt, float bodyVelX, const Body *currentTip, float &xDes, float &zDes) {
        t_stance_ += dt;

        // 累积机体位移：机体前进 → 足端在机体系中后退
        body_disp_x_ += bodyVelX * dt;

        // 足端在世界系固定 → 机体系坐标 = 落地机体坐标 - 机体位移
        xDes = foot_pos_land_[0] - body_disp_x_;
        zDes = foot_pos_land_[1];

        // ---- 离地判定 ----
        // 条件 1：达到名义支撑时间
        if (t_stance_ >= T_stance_nominal_) {
            lifted_ = true;
            return;
        }

        // 条件 2：达到最大支撑时间，强制离地
        if (t_stance_ >= T_stance_max_) {
            lifted_ = true;
            return;
        }

        // 条件 3：超过最小支撑+静默期，且跟踪误差过大时提前离地
        //  静默期(30ms)让 PD 控制器在落地瞬间有足够时间收敛，避免误触离地
        if (t_stance_ > T_stance_min_ + early_lift_grace_s_ && currentTip) {
            const float dx = currentTip->pos[0] - xDes;
            const float dz = currentTip->pos[2] - zDes;
            constexpr float earlyLiftErrorSQ = 400.0f; // 20mm 跟踪误差触发提前离地
            if (dx * dx + dz * dz > earlyLiftErrorSQ) {
                lifted_ = true;
                return;
            }
        }
    }

} // namespace Dog8Dof

// ===================================================================
//  YAML 转换
// ===================================================================
namespace YAML {
    template <> struct convert<Dog8Dof::WalkTrajectory::parameters_t> {
        static Node encode(const Dog8Dof::WalkTrajectory::parameters_t &rhs) {
            Node node;
            node["walkKp"] = rhs.walkKp;
            node["walkKd"] = rhs.walkKd;
            node["swingDutyCycle"] = rhs.swingDutyCycle;
            node["minReplanTime"] = rhs.minReplanTime;
            node["lowPassAlpha"] = rhs.lowPassAlpha;
            node["pushGain"]["horizontal"] = rhs.horizontalPushGain;
            node["pushGain"]["vertical"] = rhs.verticalPushGain;
            node["stanceTimeMin"] = rhs.stanceTimeMin;
            node["stanceTimeMax"] = rhs.stanceTimeMax;
            return node;
        }

        static bool decode(const Node &node, Dog8Dof::WalkTrajectory::parameters_t &rhs) {
            rhs.walkKp = node["walkKp"].as<float>(3.0f);
            rhs.walkKd = node["walkKd"].as<float>(0.05f);

            rhs.swingDutyCycle = node["swingDutyCycle"].as<float>(0.5f);
            rhs.minReplanTime = node["minReplanTime"].as<float>(0.02f);
            rhs.lowPassAlpha = node["lowPassAlpha"].as<float>(0.3f);

            rhs.horizontalPushGain = node["pushGain"]["horizontal"].as<float>(0.02f);
            rhs.verticalPushGain = node["pushGain"]["vertical"].as<float>(0.02f);

            rhs.stanceTimeMin = node["stanceTimeMin"].as<float>(0.05f);
            rhs.stanceTimeMax = node["stanceTimeMax"].as<float>(1.0f);

            return true;
        }
    };
} // namespace YAML