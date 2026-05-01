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
                                             const rclcpp::Time now,
                                             const GaitPhase initGait) {
        finished_ = false;
        lastUpdateTime_ = now;
        lastTipX_ = currentTip.pos[0];

        // 以当前足端位置作为摆动相起点，初始化摆动相
        const float zStart = currentTip.pos[2];
        const float zTarget = baseHeight_;
        const float xStart = currentTip.pos[0];
        // 假设从站立开始，速度/加速度均为 0
        const float vxStart = 0.0f;
        const float axStart = 0.0f;

        swing_.init(zStart,
                    zTarget,
                    stepHeight_,
                    swingTime_,
                    parameters_.lowPassAlpha,
                    parameters_.minReplanTime,
                    xStart,
                    vxStart,
                    axStart);

        gaitPhase_ = initGait;

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
                          vzDes);

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
            stance_.update(dt, currentTip, xDes, zDes);
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
                            axStart);

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
                              vzDes);
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
                                               float axStart) {
        H_ = stepH;
        T_swing_nominal_ = swingT;
        T_min_ = minReplan;
        alpha_ = alpha;
        t_elapsed_ = 0.0f;

        z_start_ = zStart;
        z_target_ = zTarget;
        landed_ = false;
        foothold_locked_ = false;

        // 初始 X 系数：从起点到初始目标（Raibert 在 update 中第一次计算前，
        // 先用期望步长的一半作为初始落脚点）
        p_target_x_ = xStart + 0.0f; // 首次滤波由 update 填充

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
                                                 float &vzDes) {
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
                // Raibert 公式：p_raw = v_des * T_stance/2 + k*(v_des - v_act)
                //
                // 注意：此处 T_stance 是下一支撑相时间，简化为当前名义支撑时间。
                // 在支撑相中可通过步频调整更新此值。
                const float T_stance =
                    T_swing_nominal_ * (1.0f / 0.5f - 1.0f); // duty=0.5 → T_st=T_sw
                const float p_raw_x = vDes * T_stance * 0.5f + pushGain * (vDes - vAct);

                // 一阶低通滤波
                if (alpha_ > eps) {
                    p_target_x_ += alpha_ * (p_raw_x - p_target_x_);
                }
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
            if (dx * dx + dz2 * dz2 < 1.0f) { // 1mm² 邻域
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
        world_pos_land_ = Eigen::Vector2f(xLand, zLand);
        T_stance_nominal_ = stanceT;
        T_stance_min_ = tMin;
        T_stance_max_ = tMax;
        t_stance_ = 0.0f;
        lifted_ = false;
    }

    void WalkTrajectory::StanceTrajectory::update(float dt,
                                                  const Body *currentTip,
                                                  float &xDes,
                                                  float &zDes) {
        t_stance_ += dt;

        // 核心原则：足端在世界坐标系中绝对静止
        // 期望位置恒等于落地瞬间记录的位置
        xDes = world_pos_land_[0];
        zDes = world_pos_land_[1];

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

        // 条件 3：超过最小支撑时间且满足提前离地条件
        //         （有传感器反馈时，检测跟踪误差是否过大）
        if (t_stance_ > T_stance_min_ && currentTip) {
            const float dx = currentTip->pos[0] - world_pos_land_[0];
            const float dz = currentTip->pos[2] - world_pos_land_[1];
            constexpr float earlyLiftErrorSQ = 100.0f; // 10mm 跟踪误差触发提前离地
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
            node["reachThreshold"] = std::sqrt(rhs.reachThresholdSQ);
            node["swingDutyCycle"] = rhs.swingDutyCycle;
            node["minReplanTime"] = rhs.minReplanTime;
            node["lowPassAlpha"] = rhs.lowPassAlpha;
            node["pushGain"]["horizontal"] = rhs.horizontalPushGain;
            node["pushGain"]["vertical"] = rhs.verticalPushGain;
            node["deltaSpeed"]["x"] = rhs.deltaSpeed[0];
            node["deltaSpeed"]["z"] = rhs.deltaSpeed[1];
            node["stanceTimeMin"] = rhs.stanceTimeMin;
            node["stanceTimeMax"] = rhs.stanceTimeMax;
            return node;
        }

        static bool decode(const Node &node, Dog8Dof::WalkTrajectory::parameters_t &rhs) {
            rhs.walkKp = node["walkKp"].as<float>(3.0f);
            rhs.walkKd = node["walkKd"].as<float>(0.05f);

            const float reachTh = node["reachThreshold"].as<float>(5.0f);
            rhs.reachThresholdSQ = reachTh * reachTh;

            rhs.swingDutyCycle = node["swingDutyCycle"].as<float>(0.5f);
            rhs.minReplanTime = node["minReplanTime"].as<float>(0.02f);
            rhs.lowPassAlpha = node["lowPassAlpha"].as<float>(0.3f);

            rhs.horizontalPushGain = node["pushGain"]["horizontal"].as<float>(0.02f);
            rhs.verticalPushGain = node["pushGain"]["vertical"].as<float>(0.02f);

            rhs.deltaSpeed = Eigen::Vector2f(node["deltaSpeed"]["x"].as<float>(0.0f),
                                             node["deltaSpeed"]["z"].as<float>(1.0f));

            rhs.stanceTimeMin = node["stanceTimeMin"].as<float>(0.05f);
            rhs.stanceTimeMax = node["stanceTimeMax"].as<float>(1.0f);

            return true;
        }
    };
} // namespace YAML