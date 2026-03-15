#pragma once
#include <Eigen/Dense>

namespace Dog8Dof {
    struct TrajectoryPoint {
        Eigen::Vector3<float> pos;
        Eigen::Vector3<float> vel;
        Eigen::Vector3<float> tau;
        Eigen::Vector2<float> kp;
        Eigen::Vector2<float> kd;
    }; // xyz方式排列,kp/kd顺序为Front->Rear

    // template <uint16_t length> struct Trajectory {
    //     TrajectoryPoint points[length];
    //     int16_t progress;
    // };

    struct Motor {
        float pos;
        float vel;
        float tau;
        float kp;
        float kd;
    };

    struct Body {
        std::string parent;               // 父坐标系
        Eigen::Vector3<float> pos;        // xyz格式机体位置
        Eigen::Matrix3<float> rot;        // 旋转矩阵
        Eigen::Vector3<float> linerVel;   // 线速度
        Eigen::Vector3<float> angularVel; // 角速度
        Eigen::Vector3<float> tau;        // 力
    };

    // 统一状态枚举定义
    enum class DogState {
        PARALYSE = 0,
        STAND = 1,
        WALK = 2,
        JUMP = 3,   // 添加跳跃模式
        STRAFE = 4, // 添加平移模式
        VISION = 5,
        DEBUG = 6,
        AUTO_RUN = 7
    };

} // namespace Dog8Dof
