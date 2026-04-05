#pragma once
#include <Eigen/Dense>

namespace Dog8Dof {
    struct TrajectoryPoint {
        Eigen::Vector3<float> pos;
        Eigen::Vector3<float> vel;
        Eigen::Vector3<float> tau;
        Eigen::Vector2<float> kp;
        Eigen::Vector2<float> kd;

        TrajectoryPoint()
            : pos(Eigen::Vector3<float>::Zero()), vel(Eigen::Vector3<float>::Zero()),
              tau(Eigen::Vector3<float>::Zero()), kp(Eigen::Vector2<float>::Zero()),
              kd(Eigen::Vector2<float>::Zero()){};
    }; // xyz方式排列,kp/kd顺序为Front->Rear

    struct Trajectory {
        std::vector<TrajectoryPoint> points;
        std::vector<size_t> time;
        size_t progress;

        Trajectory() {
        }
        Trajectory(size_t steps) {
            points.resize(steps);
            time.resize(steps);
            progress = 0;
        }

        Trajectory operator+(const Trajectory &other) const {
            Trajectory result = *this;
            result.points.insert(
                result.points.end(), other.points.begin(), other.points.end());
            result.time.insert(result.time.end(), other.time.begin(), other.time.end());
            result.progress = 0;
            return result;
        }
    };

    struct Motor {
        float pos;
        float vel;
        float tau;
        float kp;
        float kd;
    };

    struct Body {
        std::string parent;         // 父坐标系
        Eigen::Vector3f pos;        // xyz格式机体位置
        Eigen::Matrix3f rot;        // 旋转矩阵
        Eigen::Vector3f linerVel;   // 线速度
        Eigen::Vector3f angularVel; // 角速度
        Eigen::Vector3f tau;        // 力

        Body(std::string parent = "base")
            : parent(parent), pos(Eigen::Vector3f::Zero()), rot(Eigen::Matrix3f::Zero()),
              linerVel(Eigen::Vector3f::Zero()), angularVel(Eigen::Vector3f::Zero()),
              tau(Eigen::Vector3f::Zero()) {
        }
    };

    enum Gait {
        Stand,
        Walk,
        Jump,
    };

    struct TrajectoryCommand {
        std::string legName;
        Gait gait;
        float par1;
        float par2;
    };

    // 统一状态枚举定义
    // enum class DogState {
    //     PARALYSE = 0,
    //     STAND = 1,
    //     WALK = 2,
    //     JUMP = 3,   // 添加跳跃模式
    //     STRAFE = 4, // 添加平移模式
    //     VISION = 5,
    //     DEBUG = 6,
    //     AUTO_RUN = 7
    // };

} // namespace Dog8Dof
