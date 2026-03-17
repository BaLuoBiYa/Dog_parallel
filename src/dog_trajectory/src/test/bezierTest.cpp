#include "dog_tools/cubicBezier.hpp"
#include <fstream>
#include <iomanip> // for std::setprecision
#include <iostream>

using namespace Dog8Dof;

Trajectory start(const Body &currentTip) {
    Trajectory res;

    const Eigen::Vector2<float> startPos(currentTip.pos[0], currentTip.pos[2]);
    const Eigen::Vector2<float> startVel(currentTip.linerVel[0], currentTip.linerVel[2]);

    const Eigen::Vector2<float> finishPos(0, 200);
    const Eigen::Vector2<float> finishVel(0, 0);

    const Eigen::Vector2<float> tangentVec = finishPos - startPos;
    const float stepHeight = abs(tangentVec[0] * 0.3);
    // 根据步长计算步高

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

    Eigen::Vector2<float> midPos = (finishPos + startPos) / 2 + directionVec * stepHeight;
    const float T1 = 2.0f * 0.5;
    const float T2 = T1;

    const Eigen::Vector2f d1 = midPos - startPos;
    const Eigen::Vector2f d2 = finishPos - midPos;

    Eigen::Vector2f midVel =
        (6.0f * (d1 / (T1 * T1) + d2 / (T2 * T2)) - 2.0f * (startVel / T1 + finishVel / T2)) /
        (4.0f * (1.0f / T1 + 1.0f / T2));

    Trajectory first = CubicBezier::generate(startPos, midPos, startVel, midVel, 1.0f, 500);
    Trajectory second = CubicBezier::generate(midPos, finishPos, midVel, finishVel, 1.0f, 500);

    res = first + second;
    return res;
}

int main() {
    // Trajectory first, second;
    // Eigen::Vector2<float> startPos = {0.0f, 100.0f}; // (x, z)
    // Eigen::Vector2<float> startVel = {0.0f, 0.0f};   // (vx, vz)

    // Eigen::Vector2<float> midPos = {50.0f, 20.0f}; // (x, z)
    // Eigen::Vector2<float> midVel = {150.0f, 0.0f}; // (vx, vz)

    // Eigen::Vector2<float> finishPos = {200.0f, 100.0f}; // (x, z)
    // Eigen::Vector2<float> finishVel = {0.0f, 0.0f};     // (vx, vz)

    // first = CubicBezier::generate(startPos, midPos, startVel, midVel, 1.0f, 500);
    // second = CubicBezier::generate(midPos, finishPos, midVel, finishVel, 1.0f, 500);
    Body testpos;
    testpos.pos = Eigen::Vector3<float>(-100, 0, -100);
    testpos.linerVel = Eigen::Vector3<float>(0, 0, 0);
    Trajectory tra = start(testpos);

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
                << 0.0f << ","                                                           // y (硬编码 0)
                << point.pos[2] << ","                                                   // z
                << point.vel[0] << ","                                                   // vx
                << 0.0f << ","                                                           // vy (0)
                << point.vel[2] << "\n";                                                 // vz
    }

    csvFile.close();
    std::cout << "Trajectory written to" << csvPath << std::endl;
    return 0;
}