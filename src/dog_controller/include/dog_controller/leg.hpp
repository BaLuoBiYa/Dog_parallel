#pragma once
#include "dog_tools/type.hpp"

namespace Dog8Dof {

    class Leg2Dof {
      public:
        enum MotorPlacement {
            FRONT = 0,
            REAR = 1,
        };

        /**
         * @brief 二自由度并联腿
         * @param upperLength 上连杆长度
         * @param lowerLength 下连杆长度
         * @param tipLength 足端长度
         * @note
         * 一般使用方法：先调用updateMotor更新状态，再调用setTip设置足端目标，最后使用getMotorTargrt读取期望电机状态
         */
        Leg2Dof(float upperLength,
                float lowerLength,
                float tipLength,
                float frontInitPos = 0.0,
                float rearInitPos = 0.0);
        /**
         * @brief 设置期望足尖状态
         * @param pos 足端在腿坐标系下的X,Z坐标。
         * @param vel 足端在腿坐标系下的X,Z速度。
         * @param tau 足端在腿坐标系下的X,Z力矩。
         * @return 位置是否可达
         */
        bool setTip(const Eigen::Vector3<float> &pos,
                    const Eigen::Vector3<float> &vel,
                    const Eigen::Vector3<float> &tau);

        /**
         * @brief 逆运动学：根据电机当前状态逆向计算足端。
         * @param pos 前后电机位置。
         * @param vel 前后电机速度。
         * @param tau 前后电机力矩。
         * @return 是否成功更新模型状态
         */
        bool updateMotor(const Eigen::Vector2f &pos,
                         const Eigen::Vector2f &vel,
                         const Eigen::Vector2f &tau);

        /**
         * @brief 获得电机的期望状态
         * @param target 目标电机
         * @return 前电机期望状态
         */
        Motor getMotorTarget(MotorPlacement target) const;

        /**
         * @brief 获得足尖的当前状态
         * @return 足尖当前状态
         */
        Body getTipCurrent() const;

        /**
         * @brief 设置电机kp
         * @param target 目标电机
         */
        bool updateKp(MotorPlacement target, float kp);

        /**
         * @brief 设置电机kd
         * @param target 目标电机
         */
        bool updateKd(MotorPlacement target, float kd);

      private:
        const float upperLength_, lowerLength_, tipLength_;
        const float upperSQ_, lowerSQ_;
        Motor currentMotors_[2];
        Body currentTip_;
        Motor targetMotors_[2];
        Body targetTip_;

        /**
         * @brief 计算不带足端的机械腿关节角度。
         * 根据足端在腿坐标系下的X和Z坐标，计算前关节和后关节的目标角度。
         * @param x 足端在腿坐标系下的X坐标。
         * @param z 足端在腿坐标系下的Z坐标。
         */
        bool tipPosToMotor(float posX, float posZ);

        /**
         * @brief 雅可比矩阵转换函数 (速度)。
         * 将腿部末端的线速度转换为前后电机的角速度。
         * @param vx 腿部在X方向的速度。
         * @param vz 腿部在Z方向的速度。
         */
        bool tipVelToMotor(float velX, float velZ);

        /**
         * @brief 雅可比矩阵转换函数。
         * 将腿部在X和Z方向的力矢量转换为前后电机所需的力矩。
         * @param fX 腿部在X方向的力。
         * @param fZ 腿部在Z方向的力。
         */
        bool tipForceToMotor(float fX, float fZ);

        /**
         * @brief 逆运动学：根据关节角度逆向计算不带足端的足端坐标。
         * @param front_angle 前关节的当前角度。
         * @param rear_angle 后关节的当前角度。
         */
        bool motorPosToTip(float posFront, float posRear);

        /**
         * @brief 雅可比矩阵转换函数 (速度)。
         * 将前后电机的角速度转换为腿部末端的线速度。
         * @param velFront 前电机的角速度。
         * @param velRear 后电机的角速度。
         */
        bool motorVelToTip(float velFront, float velRear);

        /**
         * @brief 雅可比矩阵转换函数 (力)。
         * 将前后电机的力矩转换为腿部在X和Z方向的力矢量。
         * @param tau_front 前电机的力矩。
         * @param tau_rear 后电机的力矩。
         */
        bool motorTauToTip(float tau_front, float tau_rear);
    };

} // namespace Dog8Dof