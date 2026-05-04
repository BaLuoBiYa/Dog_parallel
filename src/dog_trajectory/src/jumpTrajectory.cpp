/**
 * @file jumpTrajectory.cpp
 * @brief JumpTrajectory — 三段式跳跃步态，全髋坐标系输出
 *
 * 坐标系：Z=0 为髋部，正 Z 向下。
 *
 * 公式：
 *   起跳垂直速度： v_z_to = sqrt(2*g*H_max)
 *   飞行时间：T_flight = 2*v_z_to/g
 *   水平蹬地距离：L_push = min(jumpLength * 0.8, 100)
 *   水平蹬地速度：v_x = -L_push / T_push
 *
 * 各阶段（髋系坐标）：
 *   PUSH_OFF  X: 五次多项式，x0 → x0-L_push，v=0→-v_x
 *             Z: 下蹲段(z0→z0+Δz,零速) + 伸展段(→z0-ε, v=-v_z_to)
 *   FLIGHT    X: 五次多项式，从push_end_x → x0（对称回落点），v=-v_x→0
 *             Z: 基线五次 + 拱形叠加
 *   ABSORPTION 世界锁定 → 髋系补偿 + 腿长压缩恢复曲线
 */

#include "dog_trajectory/jumpTrajectory.hpp"
#include "dog_tools/quinticPolynomial.hpp"
#include <cmath>

namespace Dog8Dof {

    // =========================================================================
    //  quinticFromTo6 — 便捷五次多项式系数求解
    // =========================================================================
    Eigen::Matrix<float, 6, 1> JumpTrajectory::quinticFromTo6(
        float p0, float v0, float a0, float pT, float vT, float aT, float T) {
        return QuinticPolynomial::computeCoefficients({p0, v0, a0}, {pT, vT, aT}, T);
    }

    // =========================================================================
    //  构造 & 参数
    // =========================================================================
    JumpTrajectory::JumpTrajectory(const std::string &configPath, rclcpp::Duration period)
        : BaseTrajectory(configPath, period) {
        if (!reloadParameters())
            throw std::invalid_argument("JumpTrajectory: YAML load failed");
    }

    bool JumpTrajectory::reloadParameters() {
        configFile_ = YAML::LoadFile(configPath_);
        try {
            parameters_ = configFile_.as<parameters_t>();
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(logger_, "YAML: %s", e.what());
            return false;
        }
        return true;
    }

    bool JumpTrajectory::setTarget(float jumpLength, float jumpHeight) {
        jumpLength_ = jumpLength;
        jumpHeight_ = jumpHeight;
        return true;
    }

    // =========================================================================
    //  generate() — 初始化跳跃
    // =========================================================================
    TrajectoryPoint JumpTrajectory::generate(const Body &currentTip,
                                             const rclcpp::Time now) {
        finished_ = false;
        lastUpdateTime_ = now;
        phase_ = PUSH_OFF;
        t_push_ = 0.0f;
        t_flight_ = 0.0f;
        t_absorb_ = 0.0f;

        constexpr float g = 9.81f * 1000.0f; // mm/s²
        const float pushDur = parameters_.pushDuration;
        const float crouchFrac = parameters_.crouchFraction;

        // ---- 时间参数 ----
        const float vZTakeoff = std::sqrt(2.0f * g * jumpHeight_);
        const float TFlight = 2.0f * vZTakeoff / g;

        // ---- 水平蹬地参数 ----
        const float lPush = std::min(jumpLength_ * 0.8f, 100.0f);
        const float vX = lPush / pushDur; // 大小（正）
        const float xEnd = currentTip.pos[0] - lPush;

        // ---- 蹬地 Z 参数 ----
        // Z=0 为髋部，正 Z 向下。下蹲=腿折叠=Z减小（靠近髋）
        const float xStart = currentTip.pos[0];
        const float zStart = currentTip.pos[2];
        const float zBottom = zStart - parameters_.compressDepth; // 下蹲压缩 (Z↓)
        const float zPushEnd = zStart; // 伸展回到站立腿长

        T_push_ = pushDur;
        T_crouch_ = pushDur * crouchFrac;
        T_extend_ = pushDur - T_crouch_;
        x_rel_0_ = currentTip.pos[0];
        z_rel_0_ = zStart;
        push_end_x_ = xEnd;
        push_end_vel_x_ = -vX;
        push_end_vel_z_ = vZTakeoff; // 蹬地向下伸展 = 正 Z 速度

        // ---- 蹬地多项式系数 ----
        // X: 静止 → 向后加速
        coeff_push_x_ = quinticFromTo6(xStart, 0, 0, xEnd, -vX, 0, T_push_);
        // Z 下蹲: 零速 → 零速
        coeff_push_z1_ = quinticFromTo6(zStart, 0, 0, zBottom, 0, 0, T_crouch_);
        // Z 伸展: 零速 → 向下蹬地速度
        coeff_push_z2_ = quinticFromTo6(zBottom, 0, 0, zPushEnd, vZTakeoff, 0, T_extend_);

        // ---- 空中阶段参数 ----
        T_flight_ = TFlight;
        x_flight_target_ = xStart;   // 对称回落，髋系 X 回到初始位置
        z_flight_launch_ = zPushEnd; // 蹬地结束 Z
        z_flight_land_ = parameters_.legLengthStand;
        L_stand_ = parameters_.legLengthStand;

        // 空中 X 固定多项式（回放用）：push_end → target
        coeff_flight_x_ = quinticFromTo6(xEnd, -vX, 0, x_flight_target_, 0, 0, T_flight_);

        // ---- 返回初始点 ----
        target_.pos.setZero();
        target_.pos[0] = xStart;
        target_.pos[2] = zStart;
        target_.vel.setZero();
        target_.tau.setZero();
        target_.kp[0] = parameters_.pushKp;
        target_.kp[1] = parameters_.pushKp;
        target_.kd[0] = parameters_.pushKd;
        target_.kd[1] = parameters_.pushKd;

        RCLCPP_INFO(logger_,
                    "Jump: L=%.0f H=%.0f T_push=%.3f T_flight=%.3f vZ=%.0f lPush=%.0f",
                    static_cast<double>(jumpLength_),
                    static_cast<double>(jumpHeight_),
                    static_cast<double>(T_push_),
                    static_cast<double>(T_flight_),
                    static_cast<double>(vZTakeoff),
                    static_cast<double>(lPush));
        return target_;
    }

    // =========================================================================
    //  get() — 每控制周期入口
    // =========================================================================
    TrajectoryPoint JumpTrajectory::get(const rclcpp::Time &currentTime,
                                        const Body * /*currentTip*/) {
        const float dt = (currentTime - lastUpdateTime_).seconds();
        if (dt <= 0.0f)
            return target_;
        lastUpdateTime_ = currentTime;
        constexpr float eps = 1e-6f;

        float xDes = target_.pos[0], zDes = target_.pos[2];
        float vxDes = 0.0f, vzDes = 0.0f;

        switch (phase_) {
        // ================================================================
        case PUSH_OFF: {
            t_push_ += dt;
            if (t_push_ >= T_push_ || T_push_ <= eps) {
                // → FLIGHT
                xDes = push_end_x_;
                zDes = z_flight_launch_;
                vxDes = push_end_vel_x_;
                vzDes = push_end_vel_z_;
                t_flight_ = 0.0f;
                phase_ = FLIGHT;
            } else if (t_push_ <= T_crouch_) {
                xDes = QuinticPolynomial::evaluatePos(coeff_push_x_, t_push_);
                vxDes = QuinticPolynomial::evaluateVel(coeff_push_x_, t_push_);
                zDes = QuinticPolynomial::evaluatePos(coeff_push_z1_, t_push_);
                vzDes = QuinticPolynomial::evaluateVel(coeff_push_z1_, t_push_);
            } else {
                const float tL = t_push_ - T_crouch_;
                xDes = QuinticPolynomial::evaluatePos(coeff_push_x_, t_push_);
                vxDes = QuinticPolynomial::evaluateVel(coeff_push_x_, t_push_);
                zDes = QuinticPolynomial::evaluatePos(coeff_push_z2_, tL);
                vzDes = QuinticPolynomial::evaluateVel(coeff_push_z2_, tL);
            }
            target_.kp[0] = parameters_.pushKp;
            target_.kp[1] = parameters_.pushKp;
            target_.kd[0] = parameters_.pushKd;
            target_.kd[1] = parameters_.pushKd;
        } break;

        // ================================================================
        case FLIGHT: {
            t_flight_ += dt;
            if (t_flight_ >= T_flight_ || T_flight_ <= eps) {
                // → ABSORPTION
                xDes = x_flight_target_;
                zDes = z_flight_land_;
                vxDes = 0.0f;
                vzDes = 0.0f;
                L_land_ = z_flight_land_;
                t_absorb_ = 0.0f;
                T_absorb_ = parameters_.absorbDuration;
                phase_ = ABSORPTION;
            } else {
                xDes = QuinticPolynomial::evaluatePos(coeff_flight_x_, t_flight_);
                vxDes = QuinticPolynomial::evaluateVel(coeff_flight_x_, t_flight_);
                zDes = flightZ(t_flight_,
                               T_flight_,
                               z_flight_launch_,
                               z_flight_land_,
                               parameters_.flightClearance);
                // Z 速度数值微分
                const float zp = flightZ(std::max(0.0f, t_flight_ - dt),
                                         T_flight_,
                                         z_flight_launch_,
                                         z_flight_land_,
                                         parameters_.flightClearance);
                vzDes = (zDes - zp) / std::max(dt, eps);
            }
            target_.kp[0] = parameters_.flightKp;
            target_.kp[1] = parameters_.flightKp;
            target_.kd[0] = parameters_.flightKd;
            target_.kd[1] = parameters_.flightKd;
        } break;

        // ================================================================
        case ABSORPTION: {
            t_absorb_ += dt;
            if (t_absorb_ >= T_absorb_ || T_absorb_ <= eps) {
                // 完成
                xDes = x_flight_target_;
                zDes = parameters_.legLengthStand;
                vxDes = 0.0f;
                vzDes = 0.0f;
                phase_ = FINISHED;
                finished_ = true;
            } else {
                // 髋系 X 恒定 = 落地时髋系坐标（世界锁定 → 髋系不变）
                xDes = x_flight_target_;
                vxDes = 0.0f;

                // Z: 腿长压缩恢复曲线
                zDes = absorbLegLength(t_absorb_,
                                       T_absorb_,
                                       L_land_,
                                       parameters_.legLengthStand,
                                       parameters_.absorbCompress);
                const float zp = absorbLegLength(std::max(0.0f, t_absorb_ - dt),
                                                 T_absorb_,
                                                 L_land_,
                                                 parameters_.legLengthStand,
                                                 parameters_.absorbCompress);
                vzDes = (zDes - zp) / std::max(dt, eps);
            }
            target_.kp[0] = parameters_.absorbKp;
            target_.kp[1] = parameters_.absorbKp;
            target_.kd[0] = parameters_.absorbKd;
            target_.kd[1] = parameters_.absorbKd;
        } break;

        case FINISHED:
            break;
        }

        target_.pos[0] = xDes;
        target_.pos[1] = 0.0f;
        target_.pos[2] = zDes;
        target_.vel[0] = vxDes;
        target_.vel[1] = 0.0f;
        target_.vel[2] = vzDes;
        target_.tau.setZero();
        return target_;
    }

    // =========================================================================
    //  flightZ — 空中 Z 轨迹：五次基线(保留起跳速度) + 拱形叠加
    //
    //  基线: 起点(起点z, 起跳速度vzLauch, 0)→终点(zLand, 0, 0)
    //  拱形: arch(t) = H * 16*(t/T)²*(1-t/T)², peak=H at φ=0.5
    //  抬腿 = Z 减小（靠近髋部）→ z = base(t) - arch(t)
    // =========================================================================
    float JumpTrajectory::flightZ(
        float t, float T, float zStart, float zLand, float H_air) const {
        if (T < 1e-6f)
            return zLand;

        // 基线：起跳速度 → 零速落地
        const Eigen::Matrix<float, 6, 1> baseCoeff =
            quinticFromTo6(zStart, push_end_vel_z_, 0, zLand, 0, 0, T);
        const float baseZ = QuinticPolynomial::evaluatePos(baseCoeff, t);

        const float frac = t / T;
        const float arch = H_air * 16.0f * frac * frac * (1.0f - frac) * (1.0f - frac);

        return baseZ - arch;
    }

    // =========================================================================
    //  absorbLegLength — 缓冲阶段腿长曲线
    //  前半段 L0→(L_stand-deltaL)，后半段 →L_stand，零点速边界，C² 拼接
    // =========================================================================
    float JumpTrajectory::absorbLegLength(
        float t, float T, float L0, float L_stand, float deltaL) const {
        const float halfT = T * 0.5f;
        const float L_mid = L_stand - deltaL;
        if (t <= halfT) {
            return QuinticPolynomial::evaluatePos(
                quinticFromTo6(L0, 0, 0, L_mid, 0, 0, halfT), t);
        }
        return QuinticPolynomial::evaluatePos(
            quinticFromTo6(L_mid, 0, 0, L_stand, 0, 0, halfT), t - halfT);
    }

} // namespace Dog8Dof

// =========================================================================
//  YAML 转换
// =========================================================================
namespace YAML {
    template <> struct convert<Dog8Dof::JumpTrajectory::parameters_t> {
        static Node encode(const Dog8Dof::JumpTrajectory::parameters_t &rhs) {
            Node node;
            node["pushDuration"] = rhs.pushDuration;
            node["crouchFraction"] = rhs.crouchFraction;
            node["pushKp"] = rhs.pushKp;
            node["pushKd"] = rhs.pushKd;
            node["compressDepth"] = rhs.compressDepth;
            node["toeOffLift"] = rhs.toeOffLift;
            node["flightKp"] = rhs.flightKp;
            node["flightKd"] = rhs.flightKd;
            node["flightClearance"] = rhs.flightClearance;
            node["absorbDuration"] = rhs.absorbDuration;
            node["absorbKp"] = rhs.absorbKp;
            node["absorbKd"] = rhs.absorbKd;
            node["absorbCompress"] = rhs.absorbCompress;
            node["legLengthStand"] = rhs.legLengthStand;
            return node;
        }
        static bool decode(const Node &node, Dog8Dof::JumpTrajectory::parameters_t &rhs) {
            rhs.pushDuration = node["pushDuration"].as<float>(0.15f);
            rhs.crouchFraction = node["crouchFraction"].as<float>(0.30f);
            rhs.pushKp = node["pushKp"].as<float>(3.0f);
            rhs.pushKd = node["pushKd"].as<float>(0.05f);
            rhs.compressDepth = node["compressDepth"].as<float>(30.0f);
            rhs.toeOffLift = node["toeOffLift"].as<float>(3.0f);
            rhs.flightKp = node["flightKp"].as<float>(1.5f);
            rhs.flightKd = node["flightKd"].as<float>(0.03f);
            rhs.flightClearance = node["flightClearance"].as<float>(20.0f);
            rhs.absorbDuration = node["absorbDuration"].as<float>(0.10f);
            rhs.absorbKp = node["absorbKp"].as<float>(4.0f);
            rhs.absorbKd = node["absorbKd"].as<float>(0.08f);
            rhs.absorbCompress = node["absorbCompress"].as<float>(25.0f);
            rhs.legLengthStand = node["legLengthStand"].as<float>(270.0f);
            return true;
        }
    };
} // namespace YAML
