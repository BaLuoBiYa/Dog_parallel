/**
 * @file standTrajectory.cpp
 * @brief StandTrajectory — 站立步态在线重规划实现
 *
 * TRANSITION 阶段每周期基于剩余时间重规划五次多项式（X/Z 轴），
 * Z 轴叠加光滑拱形抬脚，保证位置-速度-加速度 C² 连续。
 * 到达目标后进入 HOLD 阶段，恒输出目标位置。
 */

#include "dog_trajectory/standTrajectory.hpp"
#include "dog_tools/quinticPolynomial.hpp"
#include <cmath>

namespace Dog8Dof {

    // =========================================================================
    //  构造 & 参数加载
    // =========================================================================
    StandTrajectory::StandTrajectory(const std::string &configPath,
                                     rclcpp::Duration period)
        : BaseTrajectory(configPath, period) {
        if (!reloadParameters()) {
            throw std::invalid_argument("Could not load StandTrajectory Config YAML!");
        }
        targetTipPos_.setZero();
        coeff_x_.setZero();
        coeff_z_.setZero();
    }

    bool StandTrajectory::reloadParameters() {
        configFile_ = YAML::LoadFile(configPath_);
        try {
            parameters_ = configFile_.as<parameters_t>();
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(logger_, "YAML Error: %s", e.what());
            return false;
        }
        return true;
    }

    bool StandTrajectory::setTarget(float tipX, float tipZ) {
        targetTipPos_[0] = tipX;
        targetTipPos_[1] = tipZ;
        return true;
    }

    // =========================================================================
    //  generate() — 初始化过渡
    // =========================================================================
    TrajectoryPoint StandTrajectory::generate(const Body &currentTip,
                                              const rclcpp::Time now) {
        finished_ = false;
        lastUpdateTime_ = now;
        lastTipX_ = currentTip.pos[0];

        const float xStart = currentTip.pos[0];
        const float zStart = currentTip.pos[2];
        const float vxStart = currentTip.linerVel[0];
        const float vzStart = currentTip.linerVel[2];

        z_start_ = zStart;
        t_elapsed_ = 0.0f;
        T_transition_ = parameters_.transitionTime;

        // 计算步高 = 水平位移 × 因子，确保抬脚不拖地
        const float dx = targetTipPos_[0] - xStart;
        const float dz = targetTipPos_[1] - zStart;
        stepHeight_ = std::abs(dx) * parameters_.stepHeightFactor;
        // 最小步高 5mm，避免零位移时拱形消失
        if (stepHeight_ < 5.0f) {
            stepHeight_ = 5.0f;
        }

        // 判断是否已在目标邻域，无需过渡
        const float errSQ = dx * dx + dz * dz;
        if (errSQ <= parameters_.reachThresholdSQ) {
            phase_ = HOLD;
            target_.pos[0] = targetTipPos_[0];
            target_.pos[1] = 0.0f;
            target_.pos[2] = targetTipPos_[1];
            target_.vel.setZero();
            target_.tau.setZero();
            target_.kp[0] = parameters_.standKp;
            target_.kp[1] = parameters_.standKp;
            target_.kd[0] = parameters_.standKd;
            target_.kd[1] = parameters_.standKd;
            finished_ = true;
            return target_;
        }

        // 构建初始五次多项式系数（X 单轴，Z 用光滑弧线）
        const std::array<float, 3> startX = {xStart, vxStart, 0.0f};
        const std::array<float, 3> finishX = {targetTipPos_[0], 0.0f, 0.0f};
        const std::array<float, 3> startZ = {zStart, vzStart, 0.0f};
        const std::array<float, 3> finishZ = {targetTipPos_[1], 0.0f, 0.0f};

        coeff_x_ = QuinticPolynomial::computeCoefficients(startX, finishX, T_transition_);
        coeff_z_ = QuinticPolynomial::computeCoefficients(startZ, finishZ, T_transition_);

        phase_ = TRANSITION;

        target_.pos[0] = xStart;
        target_.pos[1] = 0.0f;
        target_.pos[2] = zStart;
        target_.vel.setZero();
        target_.tau.setZero();
        target_.kp[0] = parameters_.transitionKp;
        target_.kp[1] = parameters_.transitionKp;
        target_.kd[0] = parameters_.transitionKd;
        target_.kd[1] = parameters_.transitionKd;

        return target_;
    }

    // =========================================================================
    //  get() — 每控制周期主入口
    // =========================================================================
    TrajectoryPoint StandTrajectory::get(const rclcpp::Time &currentTime,
                                         const Body *currentTip) {
        const float dt = (currentTime - lastUpdateTime_).seconds();
        if (dt <= 0.0f) {
            return target_;
        }
        lastUpdateTime_ = currentTime;

        // ---- HOLD 阶段：恒输出目标位置 ----
        if (phase_ == HOLD) {
            target_.pos[0] = targetTipPos_[0];
            target_.pos[1] = 0.0f;
            target_.pos[2] = targetTipPos_[1];
            target_.vel.setZero();
            target_.tau.setZero();
            target_.kp[0] = parameters_.standKp;
            target_.kp[1] = parameters_.standKp;
            target_.kd[0] = parameters_.standKd;
            target_.kd[1] = parameters_.standKd;
            finished_ = true;
            return target_;
        }

        // ---- TRANSITION 阶段：在线重规划 ----
        t_elapsed_ += dt;
        const float T_rem = T_transition_ - t_elapsed_;
        constexpr float eps = 1e-6f;

        // 时间耗尽 → 直接切到目标
        if (T_rem <= eps || t_elapsed_ >= T_transition_) {
            phase_ = HOLD;
            target_.pos[0] = targetTipPos_[0];
            target_.pos[1] = 0.0f;
            target_.pos[2] = targetTipPos_[1];
            target_.vel.setZero();
            target_.tau.setZero();
            target_.kp[0] = parameters_.standKp;
            target_.kp[1] = parameters_.standKp;
            target_.kd[0] = parameters_.standKd;
            target_.kd[1] = parameters_.standKd;
            finished_ = true;
            return target_;
        }

        // 获取当前 X 状态：位置用传感器修正漂移，速度/加速度用轨迹内推保持一致性
        //    直接使用传感器速度可能导致与目标状态矛盾，使五次多项式产生大幅度偏移，
        //    超出腿工作空间 → IK 报不可达。
        float x0, vx0, ax0;
        // 始终从轨迹反算速度和加速度，保证初始条件与多项式连续
        vx0 = QuinticPolynomial::evaluateVel(coeff_x_, dt);
        ax0 = QuinticPolynomial::evaluateAcc(coeff_x_, dt);
        if (currentTip) {
            // 仅位置使用传感器反馈，修正累积漂移
            x0 = currentTip->pos[0];
        } else {
            x0 = QuinticPolynomial::evaluatePos(coeff_x_, dt);
        }

        float xDes, vxDes, zDes, vzDes;

        // X 轴：五次多项式重规划
        if (T_rem > eps) {
            const std::array<float, 3> startX = {x0, vx0, ax0};
            const std::array<float, 3> finishX = {targetTipPos_[0], 0.0f, 0.0f};
            coeff_x_ = QuinticPolynomial::computeCoefficients(startX, finishX, T_rem);
        }
        xDes = QuinticPolynomial::evaluatePos(coeff_x_, dt);
        vxDes = QuinticPolynomial::evaluateVel(coeff_x_, dt);

        // Z 轴：smoothArc（光滑弧线）+ 五次多项式基线的混合
        //       基线负责从 z_start_ 到 targetTipPos_[1] 的净位移
        //       拱形负责抬脚高度
        const float phase = std::max(0.0f, std::min(1.0f, t_elapsed_ / T_transition_));
        zDes = smoothArcZ(phase, z_start_, targetTipPos_[1], stepHeight_);

        // Z 速度解析求导
        // z(φ) = z₀ + Δz·s(φ) - H·a(φ)  (抬腿 Z 减小)
        // s'(φ) = 30φ²(1-φ)²
        // a'(φ) = 32φ(1-φ)(1-2φ)
        const float dz2 = targetTipPos_[1] - z_start_;
        const float ds = 30.0f * phase * phase * (1.0f - phase) * (1.0f - phase);
        const float da = 32.0f * phase * (1.0f - phase) * (1.0f - 2.0f * phase);
        const float dphidt = (T_transition_ > eps) ? 1.0f / T_transition_ : 0.0f;
        vzDes = (dz2 * ds - stepHeight_ * da) * dphidt;

        // 到达判定
        const float dxRem = targetTipPos_[0] - xDes;
        const float dzRem = targetTipPos_[1] - zDes;
        if (T_rem < parameters_.minReplanTime ||
            (dxRem * dxRem + dzRem * dzRem) <= parameters_.reachThresholdSQ) {
            phase_ = HOLD;
            xDes = targetTipPos_[0];
            zDes = targetTipPos_[1];
            vxDes = 0.0f;
            vzDes = 0.0f;
        }

        // 组装输出
        target_.pos[0] = xDes;
        target_.pos[1] = 0.0f;
        target_.pos[2] = zDes;
        target_.vel[0] = vxDes;
        target_.vel[1] = 0.0f;
        target_.vel[2] = vzDes;
        target_.tau.setZero();
        target_.kp[0] = parameters_.transitionKp;
        target_.kp[1] = parameters_.transitionKp;
        target_.kd[0] = parameters_.transitionKd;
        target_.kd[1] = parameters_.transitionKd;

        if (currentTip) {
            lastTipX_ = currentTip->pos[0];
        }

        return target_;
    }

    // =========================================================================
    //  smoothArcZ — Z 轴光滑弧线
    // =========================================================================
    float StandTrajectory::smoothArcZ(float phase, float z0, float zT, float H) const {
        // Z=0 为机身，正 Z 向下。抬腿 Z 减小（靠近机身） → -H*a
        // z(φ) = z0 + (zT - z0)*s(φ) - H*a(φ)
        const float p2 = phase * phase;
        const float p3 = p2 * phase;
        const float p4 = p3 * phase;
        const float p5 = p4 * phase;

        const float s = 10.0f * p3 - 15.0f * p4 + 6.0f * p5;
        const float om = 1.0f - phase;
        const float a = 16.0f * p2 * om * om;

        return z0 + (zT - z0) * s - H * a;
    }

} // namespace Dog8Dof

// =========================================================================
//  YAML 转换
// =========================================================================
namespace YAML {
    template <> struct convert<Dog8Dof::StandTrajectory::parameters_t> {
        static Node encode(const Dog8Dof::StandTrajectory::parameters_t &rhs) {
            Node node;
            node["standKp"] = rhs.standKp;
            node["standKd"] = rhs.standKd;
            node["transitionKp"] = rhs.transitionKp;
            node["transitionKd"] = rhs.transitionKd;
            node["transitionVelMax"] = rhs.transitionVelMax;
            node["reachThreshold"] = std::sqrt(rhs.reachThresholdSQ);
            node["stepHeightFactor"] = rhs.stepHeightFactor;
            node["transitionTime"] = rhs.transitionTime;
            node["minReplanTime"] = rhs.minReplanTime;
            return node;
        }

        static bool decode(const Node &node,
                           Dog8Dof::StandTrajectory::parameters_t &rhs) {
            rhs.standKp = node["standKp"].as<float>(2.0f);
            rhs.standKd = node["standKd"].as<float>(0.01f);
            rhs.transitionKp = node["transitionKp"].as<float>(2.0f);
            rhs.transitionKd = node["transitionKd"].as<float>(0.01f);
            rhs.transitionVelMax = node["transitionVelMax"].as<float>(500.0f);

            const float reachTh = node["reachThreshold"].as<float>(0.01f);
            rhs.reachThresholdSQ = reachTh * reachTh;

            rhs.stepHeightFactor = node["stepHeightFactor"].as<float>(0.1f);
            rhs.transitionTime = node["transitionTime"].as<float>(5.0f);
            rhs.minReplanTime = node["minReplanTime"].as<float>(0.02f);
            return true;
        }
    };
} // namespace YAML
