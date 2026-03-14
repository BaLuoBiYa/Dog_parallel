#include "dog_controller/model.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>

namespace Dog8Dof {
    Leg2Dof::Leg2Dof(float upperLength, float lowerLength, float tipLength, float frontInitPos, float rearInitPos)
        : upperLength_(upperLength), lowerLength_(lowerLength), tipLength_(tipLength),
          upperSQ_(upperLength * upperLength), lowerSQ_(lowerLength * lowerLength) {
        currentMotors_[FRONT] = Motor{frontInitPos, 0.0, 0.0, 0.0, 0.0};
        currentMotors_[REAR] = Motor{rearInitPos, 0.0, 0.0, 0.0, 0.0};

        targetMotors_[FRONT] = Motor{frontInitPos, 0.0, 0.0, 0.0, 0.0};
        targetMotors_[REAR] = Motor{rearInitPos, 0.0, 0.0, 0.0, 0.0};

        currentTip_ = Body{"Mount",
                           Eigen::Vector3<float>::Zero(),
                           Eigen::Matrix3<float>::Zero(),
                           Eigen::Vector3<float>::Zero(),
                           Eigen::Vector3<float>::Zero(),
                           Eigen::Vector3<float>::Zero()};
        targetTip_ = Body{"Mount",
                          Eigen::Vector3<float>::Zero(),
                          Eigen::Matrix3<float>::Zero(),
                          Eigen::Vector3<float>::Zero(),
                          Eigen::Vector3<float>::Zero(),
                          Eigen::Vector3<float>::Zero()};

        // updateMotor(frontInitPos, rearInitPos);
    }

    bool Leg2Dof::setTip(Eigen::Vector3<float> pos, Eigen::Vector3<float> vel, Eigen::Vector3<float> tau) {
        targetTip_.pos = pos;
        targetTip_.linerVel = vel;
        targetTip_.tau = tau;

        if (!tipPosToMotor(pos[0], pos[2])) {
            return false;
        }
        if (!tipVelToMotor(vel[0], vel[2])) {
            return false;
        }
        if (!tipTauToMotor(tau[0], tau[2])) {
            return false;
        }
        return true;
    }

    bool Leg2Dof::updateMotor(Eigen::Vector2<float> pos, Eigen::Vector2<float> vel, Eigen::Vector2<float> tau) {
        currentMotors_[FRONT].pos = pos[0];
        currentMotors_[REAR].pos = pos[1];

        currentMotors_[FRONT].vel = vel[0];
        currentMotors_[REAR].vel = vel[1];

        currentMotors_[FRONT].tau = tau[0];
        currentMotors_[REAR].pos = tau[1];

        if (!motorPosToTip(pos[0], pos[1])) {
            return false;
        }
        if (!motorVelToTip(vel[0], vel[1])) {
            return false;
        }
        if (!motorTauToTip(tau[0], tau[1])) {
            return false;
        }

        return true;
    }

    void Leg2Dof::updateKp(MotorPlacement target, float kp) {
        targetMotors_[target].kp = kp;
    }

    void Leg2Dof::updateKd(MotorPlacement target, float kd) {
        targetMotors_[target].kd = kd;
    }

    Motor Leg2Dof::getMotorTarget(MotorPlacement target) const {
        return targetMotors_[target];
    }

    Body Leg2Dof::getTipCurrent() const {
        return currentTip_;
    }

    /******************************************************************************************/

    bool Leg2Dof::tipPosToMotor(float posX, float posZ) {
        const float L_OC_SQ = posX * posX + posZ * posZ;
        const float L_OC = std::sqrt(L_OC_SQ); // 计算足端到髋关节的距离

        // 避免除以零
        if (L_OC < std::numeric_limits<float>::epsilon()) {
            return false;
        }

        // 保证可以形成三角形
        if (L_OC_SQ < (upperLength_ - lowerLength_) * (upperLength_ - lowerLength_) ||
            L_OC > upperLength_ + lowerLength_) {
            return false;
        }

        // 根据余弦定理计算角度
        const float numerator = L_OC_SQ + upperSQ_ - lowerSQ_;
        const float denominator = 2.0f * L_OC * upperLength_;

        const float ratio = numerator / denominator;

        const float angle_gamma = std::atan2(posX, posZ); // 足端位置向量与Z轴的夹角
        const float same_part = std::acos(ratio);         // 余弦定理计算出的角度

        targetMotors_[FRONT].pos = same_part + angle_gamma;
        targetMotors_[REAR].pos = same_part - angle_gamma;

        return true;
    }

    bool Leg2Dof::tipVelToMotor(float velX, float velZ) {
        // This function calculates the joint velocities required to produce a given linear
        // velocity at the foot. It uses the inverse of the Jacobian matrix: q_dot = J^-1 * V
        // where q_dot = [vel_front, vel_rear]^T, V = [vx, vz]^T.

        // 1. Convert joint angles to intermediate coordinates (theta, gamma)
        const float q_f = M_PI_2 - currentMotors_[FRONT].pos;
        const float q_r = M_PI_2 - currentMotors_[REAR].pos;

        const float theta = (q_f + q_r) / 2.0f;
        const float gamma = (q_f - q_r) / 2.0f;

        const float sin_theta = std::sin(theta);
        const float cos_theta = std::cos(theta);
        const float sin_gamma = std::sin(gamma);
        const float cos_gamma = std::cos(gamma);

        // 2. Calculate kinematic quantities
        const float l1_sin_theta = upperLength_ * sin_theta;
        const float l2_sq = lowerSQ_;
        const float l2_proj_sq = l2_sq - l1_sin_theta * l1_sin_theta;

        if (l2_proj_sq < 0) {
            targetMotors_[FRONT].vel = 0.0f;
            targetMotors_[REAR].vel = 0.0f;
            return false;
        }
        const float l2_proj = std::sqrt(l2_proj_sq);
        const float R = upperLength_ * cos_theta + l2_proj;

        // 3. Calculate Jacobian matrix components
        float dR_dtheta;
        if (l2_proj < 1e-6f) {
            dR_dtheta = -upperLength_ * sin_theta;
        } else {
            dR_dtheta = -l1_sin_theta * (R / l2_proj);
        }

        const float J11 = dR_dtheta * sin_gamma; // ∂x/∂theta
        const float J12 = R * cos_gamma;         // ∂x/∂gamma
        const float J21 = dR_dtheta * cos_gamma; // ∂z/∂theta
        const float J22 = -R * sin_gamma;        // ∂z/∂gamma

        // 4. Calculate the determinant of the Jacobian
        const float detJ = J11 * J22 - J12 * J21;

        // Avoid division by zero
        if (std::abs(detJ) < 1e-6f) {
            targetMotors_[FRONT].vel = 0.0f;
            targetMotors_[REAR].vel = 0.0f;
            return false;
        }

        // 5. Calculate the inverse Jacobian and multiply by the velocity vector
        const float inv_detJ = 1.0f / detJ;
        const float theta_dot = inv_detJ * (J22 * velX - J12 * velZ);
        const float gamma_dot = inv_detJ * (-J21 * velX + J11 * velZ);

        // 6. Convert back to joint velocities
        // Note the negative sign because q_f = M_PI_2 - front_angle => front_angle_dot =
        // -q_f_dot
        targetMotors_[FRONT].vel = -(theta_dot + gamma_dot);
        targetMotors_[REAR].vel = -(theta_dot - gamma_dot);

        return true;
    }

    bool Leg2Dof::tipTauToMotor(float tauX, float tauZ) {
        // This function calculates the motor torques required to produce a given force vector
        // (Fx, Fz) at the foot. It uses the transpose of the Jacobian matrix: τ = J^T * F where
        // τ = [tau_front, tau_rear]^T, F = [leg_tau_x, leg_tau_z]^T. The formula is: tau_front
        // = (∂x/∂front_angle) * leg_tau_x + (∂z/∂front_angle) * leg_tau_z tau_rear  =
        // (∂x/∂rear_angle) * leg_tau_x + (∂z/∂rear_angle) * leg_tau_z

        // Convert lengths from mm to m for correct unit conversion (N -> Nm)
        const float L1 = upperLength_ / 1000.0f;
        const float L2 = lowerLength_ / 1000.0f;

        // 1. Convert joint angles to intermediate coordinates (theta, gamma)
        const float q_f = M_PI_2 - currentMotors_[FRONT].pos;
        const float q_r = M_PI_2 - currentMotors_[REAR].pos;

        const float theta = (q_f + q_r) / 2.0f;
        const float gamma = (q_f - q_r) / 2.0f;

        const float sin_theta = std::sin(theta);
        const float cos_theta = std::cos(theta);
        const float sin_gamma = std::sin(gamma);
        const float cos_gamma = std::cos(gamma);

        // 2. Calculate kinematic quantities
        const float l1_sin_theta = L1 * sin_theta;
        const float l2_sq = L2 * L2;
        const float l2_proj_sq = l2_sq - l1_sin_theta * l1_sin_theta;

        if (l2_proj_sq < 0) {
            targetMotors_[FRONT].tau = 0.0f;
            targetMotors_[REAR].tau = 0.0f;
            return false;
        }
        const float l2_proj = std::sqrt(l2_proj_sq);
        const float R = L1 * cos_theta + l2_proj;

        // 3. Calculate the partial derivative ∂R/∂theta
        float dR_dtheta;
        if (l2_proj < 1e-6f) {
            dR_dtheta = -L1 * sin_theta;
        } else {
            dR_dtheta = -l1_sin_theta * (R / l2_proj);
        }

        // 4. Calculate partial derivatives of x and z with respect to theta and gamma
        // x = R*sin(gamma), z = R*cos(gamma)
        const float dx_dtheta = dR_dtheta * sin_gamma;
        const float dx_dgamma = R * cos_gamma;
        const float dz_dtheta = dR_dtheta * cos_gamma;
        const float dz_dgamma = -R * sin_gamma;

        // 5. Apply the chain rule to get partial derivatives with respect to joint angles
        // Note: ∂theta/∂q_f = 0.5, ∂gamma/∂q_f = 0.5
        //       ∂theta/∂q_r = 0.5, ∂gamma/∂q_r = -0.5
        // And since q_f = M_PI_2 - front_angle, ∂q_f/∂front_angle = -1.
        // So, ∂/∂front_angle = -∂/∂q_f
        const float d_dfront = -1.0f;
        const float d_drear = -1.0f;

        const float dx_dfront = (dx_dtheta * 0.5f + dx_dgamma * 0.5f) * d_dfront;
        const float dz_dfront = (dz_dtheta * 0.5f + dz_dgamma * 0.5f) * d_dfront;

        const float dx_drear = (dx_dtheta * 0.5f + dx_dgamma * -0.5f) * d_drear;
        const float dz_drear = (dz_dtheta * 0.5f + dz_dgamma * -0.5f) * d_drear;

        // 6. Calculate the final torques using J^T * F
        targetMotors_[FRONT].tau = dx_dfront * tauX + dz_dfront * tauZ;
        targetMotors_[REAR].tau = dx_drear * tauX + dz_drear * tauZ;
        return true;
    }

    /******************************************************************************************/

    bool Leg2Dof::motorPosToTip(float posFront, float posRear) {
        // 将关节角度转换为与Z轴的夹角（假设M_PI_2是90度）
        const float adjusted_front_angle = M_PI_2 - posFront;
        const float adjusted_rear_angle = M_PI_2 - posRear;

        const float angle_diff = (adjusted_front_angle - adjusted_rear_angle) * 0.5f; // 角度差的一半 (γ)
        const float angle_sum = (adjusted_front_angle + adjusted_rear_angle) * 0.5f;  // 角度和的一半 (θ)

        const float sin_theta = std::sin(angle_sum);
        const float cos_theta = std::cos(angle_sum);

        const float l1_sin = upperLength_ * sin_theta;
        const float l2_proj_sq = lowerSQ_ - l1_sin * l1_sin; // 计算L2在某个方向上的投影平方

        // 防止负数开方
        if (l2_proj_sq < 0) {
            return false;
        }

        const float l2_proj = std::sqrt(l2_proj_sq);            // L2的投影
        const float now_R = upperLength_ * cos_theta + l2_proj; // 计算足端到髋关节的距离

        currentTip_.pos[0] = now_R * std::sin(angle_diff); // 计算足端X坐标
        currentTip_.pos[2] = now_R * std::cos(angle_diff); // 计算足端Z坐标
        return true;
    }

    bool Leg2Dof::motorVelToTip(float velFront, float velRear) {
        // This function calculates the linear velocity of the foot given the joint velocities.
        // It uses the Jacobian matrix: V = J * q_dot
        // where V = [vx, vz]^T, q_dot = [vel_front, vel_rear]^T.

        // 1. Convert joint angles to intermediate coordinates (theta, gamma)
        const float q_f = M_PI_2 - currentMotors_[FRONT].pos;
        const float q_r = M_PI_2 - currentMotors_[REAR].pos;

        const float theta = (q_f + q_r) / 2.0f;
        const float gamma = (q_f - q_r) / 2.0f;

        const float sin_theta = std::sin(theta);
        const float cos_theta = std::cos(theta);
        const float sin_gamma = std::sin(gamma);
        const float cos_gamma = std::cos(gamma);

        // 2. Calculate kinematic quantities
        const float l1_sin_theta = upperLength_ * sin_theta;
        const float l2_proj_sq = lowerSQ_ - l1_sin_theta * l1_sin_theta;

        if (l2_proj_sq < 0) {
            targetTip_.linerVel[0] = 0.0f;
            targetTip_.linerVel[2] = 0.0f;
            return false;
        }
        const float l2_proj = std::sqrt(l2_proj_sq);
        const float R = upperLength_ * cos_theta + l2_proj;

        // 3. Calculate Jacobian matrix components (from leg_vel_to_motor_vel)
        float dR_dtheta;
        if (l2_proj < 1e-6f) {
            dR_dtheta = -upperLength_ * sin_theta;
        } else {
            dR_dtheta = -l1_sin_theta * (R / l2_proj);
        }

        const float J11_theta_gamma = dR_dtheta * sin_gamma; // ∂x/∂theta
        const float J12_theta_gamma = R * cos_gamma;         // ∂x/∂gamma
        const float J21_theta_gamma = dR_dtheta * cos_gamma; // ∂z/∂theta
        const float J22_theta_gamma = -R * sin_gamma;        // ∂z/dgamma

        // 4. Convert motor velocities to theta_dot and gamma_dot
        // Note the negative sign because front_angle_dot = -q_f_dot
        const float q_f_dot = -velFront;
        const float q_r_dot = -velRear;

        const float theta_dot = (q_f_dot + q_r_dot) / 2.0f;
        const float gamma_dot = (q_f_dot - q_r_dot) / 2.0f;

        // 5. Calculate the final linear velocities using V = J_theta_gamma * [theta_dot, gamma_dot]^T
        currentTip_.linerVel[0] = J11_theta_gamma * theta_dot + J12_theta_gamma * gamma_dot;
        currentTip_.linerVel[2] = J21_theta_gamma * theta_dot + J22_theta_gamma * gamma_dot;
        return true;
    }

    bool Leg2Dof::motorTauToTip(float tau_front, float tau_rear) {
        // This function calculates the foot force vector (Fx, Fz) produced by given motor torques.
        // It uses the relationship F = (J^T)^-1 * τ, where τ = J^T * F.
        // τ = [tau_front, tau_rear]^T, F = [leg_tau_x, leg_tau_z]^T.

        // Convert lengths from mm to m for correct unit conversion (Nm -> N)
        const float L1 = upperLength_ / 1000.0f;
        const float L2 = lowerLength_ / 1000.0f;

        // 1. Convert joint angles to intermediate coordinates (theta, gamma)
        const float q_f = M_PI_2 - currentMotors_[FRONT].pos;
        const float q_r = M_PI_2 - currentMotors_[REAR].pos;

        const float theta = (q_f + q_r) / 2.0f;
        const float gamma = (q_f - q_r) / 2.0f;

        const float sin_theta = std::sin(theta);
        const float cos_theta = std::cos(theta);
        const float sin_gamma = std::sin(gamma);
        const float cos_gamma = std::cos(gamma);

        // 2. Calculate kinematic quantities
        const float l1_sin_theta = L1 * sin_theta;
        const float l2_sq = L2 * L2;
        const float l2_proj_sq = l2_sq - l1_sin_theta * l1_sin_theta;

        if (l2_proj_sq < 0) {
            targetTip_.tau[0] = 0.0f;
            targetTip_.tau[2] = 0.0f;
            return false;
        }
        const float l2_proj = std::sqrt(l2_proj_sq);
        const float R = L1 * cos_theta + l2_proj;

        // 3. Calculate the partial derivative ∂R/∂theta
        float dR_dtheta;
        if (l2_proj < 1e-6f) {
            dR_dtheta = -L1 * sin_theta;
        } else {
            dR_dtheta = -l1_sin_theta * (R / l2_proj);
        }

        // 4. Calculate partial derivatives of x and z with respect to theta and gamma
        const float dx_dtheta = dR_dtheta * sin_gamma;
        const float dx_dgamma = R * cos_gamma;
        const float dz_dtheta = dR_dtheta * cos_gamma;
        const float dz_dgamma = -R * sin_gamma;

        // 5. Calculate partial derivatives with respect to joint angles (components of Jacobian J)
        // Note: ∂/∂front_angle = -∂/∂q_f, ∂/∂rear_angle = -∂/∂q_r
        const float d_dfront = -1.0f;
        const float d_drear = -1.0f;

        // J = [[dx_dfront, dx_drear], [dz_dfront, dz_drear]]
        const float dx_dfront = (dx_dtheta * 0.5f + dx_dgamma * 0.5f) * d_dfront;
        const float dz_dfront = (dz_dtheta * 0.5f + dz_dgamma * 0.5f) * d_dfront;
        const float dx_drear = (dx_dtheta * 0.5f + dx_dgamma * -0.5f) * d_drear;
        const float dz_drear = (dz_dtheta * 0.5f + dz_dgamma * -0.5f) * d_drear;

        // 6. Form the Jacobian Transpose matrix J^T
        // J^T = [[dx_dfront, dz_dfront], [dx_drear, dz_drear]]
        const float Jt_11 = dx_dfront;
        const float Jt_12 = dz_dfront;
        const float Jt_21 = dx_drear;
        const float Jt_22 = dz_drear;

        // 7. Calculate the determinant of J^T (which is equal to det(J))
        const float detJt = Jt_11 * Jt_22 - Jt_12 * Jt_21;

        if (std::abs(detJt) < 1e-6f) {
            targetTip_.tau[0] = 0.0f;
            targetTip_.tau[2] = 0.0f;
            return false;
        }

        // 8. Calculate F = (J^T)^-1 * τ
        // (J^T)^-1 = (1/detJt) * [[Jt_22, -Jt_12], [-Jt_21, Jt_11]]
        const float inv_detJt = 1.0f / detJt;
        targetTip_.tau[0] = inv_detJt * (Jt_22 * tau_front - Jt_12 * tau_rear);
        targetTip_.tau[2] = inv_detJt * (-Jt_21 * tau_front + Jt_11 * tau_rear);
        return true;
    }

} // namespace Dog8Dof