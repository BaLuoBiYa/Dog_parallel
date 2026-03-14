#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

namespace Dog8Dof {
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

} // namespace Dog8Dof
