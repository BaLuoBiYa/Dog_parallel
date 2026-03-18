#include "dog_tools/cubicBezier.hpp"
#include "dog_tools/quinticPolynomial.hpp"
#include <fstream>
#include <iomanip> // for std::setprecision
#include <iostream>

using namespace Dog8Dof;

// Trajectory start(const Body &currentTip) {
//     Trajectory res;

//     const Eigen::Vector2<float> startPos(currentTip.pos[0], currentTip.pos[2]);
//     const Eigen::Vector2<float> startVel(currentTip.linerVel[0], currentTip.linerVel[2]);

//     const Eigen::Vector2<float> finishPos(0, 200);
//     const Eigen::Vector2<float> finishVel(0, 0);

//     const Eigen::Vector2<float> tangentVec = finishPos - startPos;
//     const float stepHeight = abs(tangentVec[0] * 0.3);
//     // 根据步长计算步高

//     const Eigen::Vector2<float> normVec0(-tangentVec[1], tangentVec[0]);
//     const Eigen::Vector2<float> normVec1(tangentVec[1], -tangentVec[0]);
//     // 计算起始向量的两个法向量

//     Eigen::Vector2<float> directionVec;
//     if (normVec0[1] <= 0) {
//         directionVec = normVec0.normalized();
//     } else {
//         directionVec = normVec1.normalized();
//     }
//     // 朝向Z负方向（朝向天空的向量作为运动方向）

//     Eigen::Vector2<float> midPos = (finishPos + startPos) / 2 + directionVec * stepHeight;
//     const float T1 = 2.0f * 0.5;
//     const float T2 = T1;

//     const Eigen::Vector2f d1 = midPos - startPos;
//     const Eigen::Vector2f d2 = finishPos - midPos;

//     Eigen::Vector2f midVel =
//         (6.0f * (d1 / (T1 * T1) + d2 / (T2 * T2)) - 2.0f * (startVel / T1 + finishVel / T2)) /
//         (4.0f * (1.0f / T1 + 1.0f / T2));

//     Trajectory first = CubicBezier::generate(startPos, midPos, startVel, midVel, 1.0f, 500);
//     Trajectory second = CubicBezier::generate(midPos, finishPos, midVel, finishVel, 1.0f, 500);

//     res = first + second;
//     return res;
// }

Trajectory generate(const Body &currentTip) {

    Trajectory trajectory_ = Trajectory(1000);
    float stepLength_ = 200;
    float stepHeight_ = 20;
    float durationNanoSec_ = 2e9;

    const float durationSec = durationNanoSec_ * 1e-9;
    const Eigen::Vector2<float> swingStartPos(currentTip.pos[0], currentTip.pos[2]);
    const Eigen::Vector2<float> swingStartVel(currentTip.linerVel[0], currentTip.linerVel[2]);

    const Eigen::Vector2<float> swingFinishPos(stepLength_ * 0.5f, 0.0f);
    // 摆动相结束时，足端应该以机体速度向后运动

    const Eigen::Vector2<float> tangentVec = swingFinishPos - swingStartPos;
    const Eigen::Vector2<float> normVec0(-tangentVec[1], tangentVec[0]);
    const Eigen::Vector2<float> normVec1(tangentVec[1], -tangentVec[0]);
    // 计算起始向量的两个法向量

    Eigen::Vector2<float> directionVec;
    if (normVec0[1] <= 0) {
        directionVec = normVec0.normalized();
    } else {
        directionVec = normVec1.normalized();
    }
    // 朝向Z负方向（朝向天空的向量作为运动方向）

    const Eigen::Vector2<float> swingMidPos =
        (swingFinishPos + swingStartPos) * 0.5 + directionVec * stepHeight_;

    const float swingRatio = 0.5f; // 摆动相占整个周期比例
    const float T1 = durationSec * swingRatio * 0.5f;
    const float T2 = T1;
    const float T3 = durationSec - T1 - T2;

    const size_t steps1 = static_cast<size_t>(1000 * swingRatio * 0.5f);
    const size_t steps2 = steps1;
    const size_t steps3 = 1000 - steps1 - steps2;

    const Eigen::Vector2<float> stanceFinishPos(-stepLength_ * 0.5f, 0.0f);

    // 恒速度支撑相：由位移/时长决定
    const Eigen::Vector2<float> stanceVel = (stanceFinishPos - swingFinishPos) / T3;

    // 为了摆动->支撑拼接速度连续，把摆动结束速度设成支撑相速度
    const Eigen::Vector2<float> swingFinishVel = stanceVel;

    const Eigen::Vector2<float> swingMidVel = QuinticPolynomial::solveMidVel2D(
        swingStartPos, swingMidPos, swingFinishPos, swingStartVel, swingFinishVel, T1, T2);

    const Eigen::Vector2<float> zeroAcc(0.0f, 0.0f);
    const Eigen::Vector2f swingMidAcc = QuinticPolynomial::solveMidAcc2D(swingStartPos,
                                                                         swingStartVel,
                                                                         zeroAcc,
                                                                         swingMidPos,
                                                                         swingMidVel,
                                                                         swingFinishPos,
                                                                         swingFinishVel,
                                                                         zeroAcc,
                                                                         T1,
                                                                         T2);

    Trajectory first = QuinticPolynomial::generate(
        swingStartPos, swingMidPos, swingStartVel, swingMidVel, zeroAcc, swingMidAcc, T1, steps1);

    // 落足前最后一段：五次，约束末端加速度为 0
    Trajectory second = QuinticPolynomial::generate(
        swingMidPos, swingFinishPos, swingMidVel, swingFinishVel, swingMidAcc, zeroAcc, T2, steps2);

    // 支撑段：恒速 + 五次（起终速度相同，起终加速度为 0）
    Trajectory third = QuinticPolynomial::generate(
        swingFinishPos, stanceFinishPos, stanceVel, stanceVel, zeroAcc, zeroAcc, T3, steps3);

    trajectory_ = first + second + third;
    return trajectory_;
}

int main() {
    Body testpos;
    testpos.pos = Eigen::Vector3<float>(-100, 0, 0);
    testpos.linerVel = Eigen::Vector3<float>(-200, 0, 0);
    Trajectory tra = generate(testpos);

    // 写入 CSV 文件
    std::string csvPath;
    std::cout << "Type CSV Path:";
    std::cin >> csvPath;
    std::ofstream csvFile(csvPath);
    if (!csvFile.is_open()) {
        std::cerr << "Error: Cannot open CSV file for writing." << std::endl;
        return 1;
    }

    // CSV 头部
    csvFile << "time,x,y,z,vx,vy,vz\n";

    // 总时间 2.0 秒，steps=1000
    float totalTime = 2.0f;
    float timeStep = totalTime / (1000 - 1);

    for (size_t i = 0; i < 1000; ++i) {
        float t = i * timeStep;
        const auto &point = tra.points[i];
        csvFile << std::fixed << std::setprecision(6) << t << "," << point.pos[0] << "," // x
                << 0.0f << ","           // y (硬编码 0)
                << point.pos[2] << ","   // z
                << point.vel[0] << ","   // vx
                << 0.0f << ","           // vy (0)
                << point.vel[2] << "\n"; // vz
    }

    csvFile.close();
    std::cout << "Trajectory written to" << csvPath << std::endl;
    return 0;
}