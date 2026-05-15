#include "dog_controller/leg.hpp"
#include <cmath>
#include <iostream>

namespace Dog8Dof {
    namespace {
        constexpr float kMotorSign = -1.0f;

        inline float motorToModel(float v) {
            return kMotorSign * v;
        }

        inline float modelToMotor(float v) {
            return kMotorSign * v;
        }
    } // namespace

    Leg2Dof::Leg2Dof(float upperLength,
                     float lowerLength,
                     float tipLength,
                     float frontInitPos,
                     float rearInitPos)
        : upperLength_(upperLength), lowerLength_(lowerLength), tipLength_(tipLength),
          upperSQ_(upperLength * upperLength), lowerSQ_(lowerLength * lowerLength) {
        currentMotors_[FRONT] = Motor{motorToModel(frontInitPos), 0.0, 0.0, 0.0, 0.0};
        currentMotors_[REAR] = Motor{motorToModel(rearInitPos), 0.0, 0.0, 0.0, 0.0};

        targetMotors_[FRONT] = Motor{motorToModel(frontInitPos), 0.0, 0.0, 0.0, 0.0};
        targetMotors_[REAR] = Motor{motorToModel(rearInitPos), 0.0, 0.0, 0.0, 0.0};

        currentTip_ = Body("Mount");
        targetTip_ = Body("Mount");

        // updateMotor(frontInitPos, rearInitPos);
    }

    bool Leg2Dof::setTip(const Eigen::Vector3<float> &pos,
                         const Eigen::Vector3<float> &vel,
                         const Eigen::Vector3<float> &tau) {
        targetTip_.pos = pos;
        targetTip_.linerVel = vel;
        targetTip_.tau = tau;

        if (!tipPosToMotor(pos[0], pos[2])) {
            std::cerr << "Failed set  tip pos" << std::endl;
            return false;
        }
        // tipPosToMotor 已填充 tgt_kin_，后续函数直接复用
        if (!tipVelToMotor(vel[0], vel[2], tgt_kin_)) {
            std::cerr << "Failed set  tip vel" << std::endl;
            return false;
        }
        if (!tipForceToMotor(tau[0], tau[2], tgt_kin_)) {
            std::cerr << "Failed set  tip tau" << std::endl;
            return false;
        }
        return true;
    }

    bool Leg2Dof::updateMotor(const Eigen::Vector2f &pos,
                              const Eigen::Vector2f &vel,
                              const Eigen::Vector2f &tau) {
        const float modelFrontPos = motorToModel(pos[0]);
        const float modelRearPos = motorToModel(pos[1]);
        const float modelFrontVel = motorToModel(vel[0]);
        const float modelRearVel = motorToModel(vel[1]);
        const float modelFrontTau = motorToModel(tau[0]);
        const float modelRearTau = motorToModel(tau[1]);

        currentMotors_[FRONT].pos = modelFrontPos;
        currentMotors_[REAR].pos = modelRearPos;

        currentMotors_[FRONT].vel = modelFrontVel;
        currentMotors_[REAR].vel = modelRearVel;

        currentMotors_[FRONT].tau = modelFrontTau;
        currentMotors_[REAR].tau = modelRearTau;

        if (!motorPosToTip(modelFrontPos, modelRearPos)) {
            return false;
        }
        // motorPosToTip 已填充 cur_kin_，后续函数直接复用
        if (!motorVelToTip(modelFrontVel, modelRearVel, cur_kin_)) {
            return false;
        }
        if (!motorTauToTip(modelFrontTau, modelRearTau, cur_kin_)) {
            return false;
        }

        return true;
    }

    bool Leg2Dof::updateKp(MotorPlacement target, float kp) {
        if (kp > 100.0f || kp < 0.0f) {
            return false;
        }
        targetMotors_[target].kp = kp;
        return true;
    }

    bool Leg2Dof::updateKd(MotorPlacement target, float kd) {
        if (kd > 100.0f || kd < 0.0f) {
            return false;
        }
        targetMotors_[target].kd = kd;
        return true;
    }

    Motor Leg2Dof::getMotorTarget(MotorPlacement target) const {
        Motor motorTarget = targetMotors_[target];
        motorTarget.pos = modelToMotor(motorTarget.pos);
        motorTarget.vel = modelToMotor(motorTarget.vel);
        motorTarget.tau = modelToMotor(motorTarget.tau);
        return motorTarget;
    }

    Body Leg2Dof::getTipCurrent() const {
        return currentTip_;
    }

    Body Leg2Dof::getTipTarget() const {
        return targetTip_;
    }

    /******************************************************************************************/

    bool Leg2Dof::tipPosToMotor(float posX, float posZ) {
        const float L_OC_SQ = posX * posX + posZ * posZ;
        const float L_OC = std::sqrt(L_OC_SQ);

        if (L_OC < std::numeric_limits<float>::epsilon()) {
            return false;
        }

        if (L_OC_SQ < (upperLength_ - lowerLength_) * (upperLength_ - lowerLength_) ||
            L_OC > upperLength_ + lowerLength_) {
            return false;
        }

        const float numerator = L_OC_SQ + upperSQ_ - lowerSQ_;
        const float denominator = 2.0f * L_OC * upperLength_;
        float ratio = numerator / denominator;
        ratio = std::clamp(ratio, -1.0f, 1.0f);

        const float angle_gamma = std::atan2(posX, posZ);
        const float same_part = std::acos(ratio);

        const float q_front = same_part + angle_gamma;
        const float q_rear = same_part - angle_gamma;

        targetMotors_[FRONT].pos = M_PI_2 - q_front;
        targetMotors_[REAR].pos = M_PI_2 - q_rear;

        // 预计算 sin/cos 缓存，供 tipVelToMotor / tipForceToMotor 复用
        fillKinCache(tgt_kin_, q_front, q_rear);

        return true;
    }

    bool Leg2Dof::tipVelToMotor(float velX, float velZ, const KinCache &kc) {
        // 使用缓存的 sin/cos，避免重复计算
        const float sin_theta = kc.sin_theta;
        const float cos_theta = kc.cos_theta;
        const float sin_gamma = kc.sin_gamma;
        const float cos_gamma = kc.cos_gamma;

        const float l1_sin_theta = upperLength_ * sin_theta;
        const float l2_sq = lowerSQ_;
        const float l2_proj_sq = l2_sq - l1_sin_theta * l1_sin_theta;

        // 容忍浮点误差在边界处产生的微小负数，clamp 到 0
        const float l2_proj_sq_clamped = std::max(0.0f, l2_proj_sq);
        const float l2_proj = std::sqrt(l2_proj_sq_clamped);
        const float R = upperLength_ * cos_theta + l2_proj;

        // Jacobian components
        float dR_dtheta;
        if (l2_proj < 1e-6f) {
            dR_dtheta = -upperLength_ * sin_theta;
        } else {
            dR_dtheta = -l1_sin_theta * (R / l2_proj);
        }

        const float J11 = dR_dtheta * sin_gamma;
        const float J12 = R * cos_gamma;
        const float J21 = dR_dtheta * cos_gamma;
        const float J22 = -R * sin_gamma;

        const float detJ = J11 * J22 - J12 * J21;
        if (std::abs(detJ) < 1e-6f) {
            targetMotors_[FRONT].vel = 0.0f;
            targetMotors_[REAR].vel = 0.0f;
            return false;
        }

        const float inv_detJ = 1.0f / detJ;
        const float theta_dot = inv_detJ * (J22 * velX - J12 * velZ);
        const float gamma_dot = inv_detJ * (-J21 * velX + J11 * velZ);

        targetMotors_[FRONT].vel = -(theta_dot + gamma_dot);
        targetMotors_[REAR].vel = -(theta_dot - gamma_dot);

        return true;
    }

    bool Leg2Dof::tipForceToMotor(float fX, float fZ, const KinCache &kc) {
        // 使用缓存的 sin/cos；L1/L2 单位转换为 m (N→Nm)
        const float L1 = upperLength_ / 1000.0f;
        const float L2 = lowerLength_ / 1000.0f;

        const float sin_theta = kc.sin_theta;
        const float cos_theta = kc.cos_theta;
        const float sin_gamma = kc.sin_gamma;
        const float cos_gamma = kc.cos_gamma;

        const float l1_sin_theta = L1 * sin_theta;
        const float l2_sq = L2 * L2;
        const float l2_proj_sq = l2_sq - l1_sin_theta * l1_sin_theta;

        // 容忍浮点误差在边界处产生的微小负数，clamp 到 0
        const float l2_proj_sq_clamped = std::max(0.0f, l2_proj_sq);
        const float l2_proj = std::sqrt(l2_proj_sq_clamped);
        const float R = L1 * cos_theta + l2_proj;

        float dR_dtheta;
        if (l2_proj < 1e-6f) {
            dR_dtheta = -L1 * sin_theta;
        } else {
            dR_dtheta = -l1_sin_theta * (R / l2_proj);
        }

        const float dx_dtheta = dR_dtheta * sin_gamma;
        const float dx_dgamma = R * cos_gamma;
        const float dz_dtheta = dR_dtheta * cos_gamma;
        const float dz_dgamma = -R * sin_gamma;

        // ∂/∂front_angle = -(∂/∂q_f), where q_f = ½(θ+γ)
        const float dx_dfront = -(dx_dtheta * 0.5f + dx_dgamma * 0.5f);
        const float dz_dfront = -(dz_dtheta * 0.5f + dz_dgamma * 0.5f);
        const float dx_drear = -(dx_dtheta * 0.5f - dx_dgamma * 0.5f);
        const float dz_drear = -(dz_dtheta * 0.5f - dz_dgamma * 0.5f);

        targetMotors_[FRONT].tau = dx_dfront * fX + dz_dfront * fZ;
        targetMotors_[REAR].tau = dx_drear * fX + dz_drear * fZ;
        return true;
    }

    /******************************************************************************************/

    bool Leg2Dof::motorPosToTip(float posFront, float posRear) {
        const float q_f = M_PI_2 - posFront;
        const float q_r = M_PI_2 - posRear;

        // 填充运动学缓存，供 motorVelToTip / motorTauToTip 复用
        fillKinCache(cur_kin_, q_f, q_r);

        const float l1_sin = upperLength_ * cur_kin_.sin_theta;
        const float l2_proj_sq = lowerSQ_ - l1_sin * l1_sin;

        // 容忍浮点误差在边界处产生的微小负数，clamp 到 0
        const float l2_proj_sq_clamped = std::max(0.0f, l2_proj_sq);
        const float l2_proj = std::sqrt(l2_proj_sq_clamped);
        const float now_R = upperLength_ * cur_kin_.cos_theta + l2_proj;

        currentTip_.pos[0] = now_R * cur_kin_.sin_gamma;
        currentTip_.pos[2] = now_R * cur_kin_.cos_gamma;
        return true;
    }

    bool Leg2Dof::motorVelToTip(float velFront, float velRear, const KinCache &kc) {
        const float sin_theta = kc.sin_theta;
        const float cos_theta = kc.cos_theta;
        const float sin_gamma = kc.sin_gamma;
        const float cos_gamma = kc.cos_gamma;

        const float l1_sin_theta = upperLength_ * sin_theta;
        const float l2_proj_sq = lowerSQ_ - l1_sin_theta * l1_sin_theta;

        // 容忍浮点误差在边界处产生的微小负数，clamp 到 0
        const float l2_proj_sq_clamped = std::max(0.0f, l2_proj_sq);
        const float l2_proj = std::sqrt(l2_proj_sq_clamped);
        const float R = upperLength_ * cos_theta + l2_proj;

        float dR_dtheta;
        if (l2_proj < 1e-6f) {
            dR_dtheta = -upperLength_ * sin_theta;
        } else {
            dR_dtheta = -l1_sin_theta * (R / l2_proj);
        }

        const float J11 = dR_dtheta * sin_gamma;
        const float J12 = R * cos_gamma;
        const float J21 = dR_dtheta * cos_gamma;
        const float J22 = -R * sin_gamma;

        const float q_f_dot = -velFront;
        const float q_r_dot = -velRear;
        const float theta_dot = (q_f_dot + q_r_dot) * 0.5f;
        const float gamma_dot = (q_f_dot - q_r_dot) * 0.5f;

        currentTip_.linerVel[0] = J11 * theta_dot + J12 * gamma_dot;
        currentTip_.linerVel[2] = J21 * theta_dot + J22 * gamma_dot;
        return true;
    }

    bool Leg2Dof::motorTauToTip(float tau_front, float tau_rear, const KinCache &kc) {
        const float L1 = upperLength_ / 1000.0f;
        const float L2 = lowerLength_ / 1000.0f;

        const float sin_theta = kc.sin_theta;
        const float cos_theta = kc.cos_theta;
        const float sin_gamma = kc.sin_gamma;
        const float cos_gamma = kc.cos_gamma;

        const float l1_sin_theta = L1 * sin_theta;
        const float l2_sq = L2 * L2;
        const float l2_proj_sq = l2_sq - l1_sin_theta * l1_sin_theta;

        // 容忍浮点误差在边界处产生的微小负数，clamp 到 0
        const float l2_proj_sq_clamped = std::max(0.0f, l2_proj_sq);
        const float l2_proj = std::sqrt(l2_proj_sq_clamped);
        const float R = L1 * cos_theta + l2_proj;

        float dR_dtheta;
        if (l2_proj < 1e-6f) {
            dR_dtheta = -L1 * sin_theta;
        } else {
            dR_dtheta = -l1_sin_theta * (R / l2_proj);
        }

        const float dx_dtheta = dR_dtheta * sin_gamma;
        const float dx_dgamma = R * cos_gamma;
        const float dz_dtheta = dR_dtheta * cos_gamma;
        const float dz_dgamma = -R * sin_gamma;

        const float Jt_11 = -(dx_dtheta * 0.5f + dx_dgamma * 0.5f);
        const float Jt_12 = -(dz_dtheta * 0.5f + dz_dgamma * 0.5f);
        const float Jt_21 = -(dx_dtheta * 0.5f - dx_dgamma * 0.5f);
        const float Jt_22 = -(dz_dtheta * 0.5f - dz_dgamma * 0.5f);

        const float detJt = Jt_11 * Jt_22 - Jt_12 * Jt_21;
        if (std::abs(detJt) < 1e-6f) {
            currentTip_.tau[0] = 0.0f;
            currentTip_.tau[2] = 0.0f;
            return false;
        }

        const float inv_detJt = 1.0f / detJt;
        currentTip_.tau[0] = inv_detJt * (Jt_22 * tau_front - Jt_12 * tau_rear);
        currentTip_.tau[2] = inv_detJt * (-Jt_21 * tau_front + Jt_11 * tau_rear);
        return true;
    }

} // namespace Dog8Dof