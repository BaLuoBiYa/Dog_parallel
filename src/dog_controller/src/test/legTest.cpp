#include "dog_controller/leg.hpp"
#include "dog_tools/type.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace Dog8Dof;

static bool approxEqual(float a, float b, float tol = 1e-3f) {
    return std::fabs(a - b) <= tol;
}

static void printResult(const std::string &name, bool pass) {
    std::cout << "  " << name << ": "
              << (pass ? "\033[32mPASS\033[0m" : "\033[31mFAIL\033[0m") << "\n";
}

struct Case {
    Eigen::Vector3f pos;
    Eigen::Vector3f vel;
    Eigen::Vector3f force;
};

static bool isReachable(float x, float z, float upper, float lower) {
    const float l2 = x * x + z * z;
    const float minL = std::fabs(upper - lower) + 1e-3f;
    const float maxL = (upper + lower) - 1e-3f;
    return (l2 >= minL * minL) && (l2 <= maxL * maxL);
}

static Case sampleReachableCase(std::mt19937 &rng, float upper, float lower) {
    std::uniform_real_distribution<float> dist_x(-80.0f, 80.0f);
    std::uniform_real_distribution<float> dist_z(80.0f, 190.0f);
    std::uniform_real_distribution<float> dist_v(-8.0f, 8.0f);
    std::uniform_real_distribution<float> dist_f(-12.0f, 12.0f);

    Case c{};
    for (int k = 0; k < 500; ++k) {
        float x = dist_x(rng);
        float z = dist_z(rng);
        if (!isReachable(x, z, upper, lower)) {
            continue;
        }
        c.pos = Eigen::Vector3f(x, 0.0f, z);
        c.vel = Eigen::Vector3f(dist_v(rng), 0.0f, dist_v(rng));
        c.force = Eigen::Vector3f(dist_f(rng), 0.0f, dist_f(rng));
        return c;
    }

    // 兜底：给一个稳定可达点
    c.pos = Eigen::Vector3f(0.0f, 0.0f, 180.0f);
    c.vel = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    c.force = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    return c;
}

int main() {
    const float upper = 100.0f;
    const float lower = 100.0f;
    const float tiplen = 10.0f;

    // 随机样本数量与随机种子
    const int case_count = 50;
    const uint32_t seed = 20260405u;
    std::mt19937 rng(seed);

    std::cout << "Random test seed: " << seed << ", case_count: " << case_count << "\n";

    Leg2Dof leg(upper, lower, tiplen, 0.0f, 0.0f);

    std::vector<Case> cases;
    cases.reserve(case_count);
    for (int i = 0; i < case_count; ++i) {
        cases.push_back(sampleReachableCase(rng, upper, lower));
    }

    bool all_pass = true;
    int fail_pos = 0;
    int fail_vel = 0;
    int fail_force = 0;

    for (size_t i = 0; i < cases.size(); ++i) {
        const auto &c = cases[i];
        std::cout << "Case " << i << " pos(" << c.pos[0] << ", " << c.pos[2] << ")\n";

        Eigen::Vector3f zero = Eigen::Vector3f::Zero();

        bool ok = leg.setTip(c.pos, zero, zero);
        if (!ok) {
            std::cout << "  setTip(position) failed\n";
            all_pass = false;
            ++fail_pos;
            ++fail_vel;
            ++fail_force;
            continue;
        }

        Motor target_front = leg.getMotorTarget(Leg2Dof::FRONT);
        Motor target_rear = leg.getMotorTarget(Leg2Dof::REAR);

        Eigen::Vector2f motorPos(target_front.pos, target_rear.pos);
        Eigen::Vector2f motorVel(0.0f, 0.0f);
        Eigen::Vector2f motorTau(0.0f, 0.0f);

        bool upd_ok = leg.updateMotor(motorPos, motorVel, motorTau);
        if (!upd_ok) {
            std::cout << "  updateMotor(position) failed\n";
            all_pass = false;
            ++fail_pos;
            ++fail_vel;
            ++fail_force;
            continue;
        }

        Body tip = leg.getTipCurrent();

        bool pass_pos = approxEqual(tip.pos[0], c.pos[0], 1e-2f) &&
                        approxEqual(tip.pos[2], c.pos[2], 1e-2f);

        std::cout << "  delta tip: x=" << tip.pos[0] - c.pos[0]
                  << " z=" << tip.pos[2] - c.pos[2] << "\n";
        printResult("POSITION", pass_pos);
        all_pass = all_pass && pass_pos;
        if (!pass_pos) {
            ++fail_pos;
        }

        ok = leg.setTip(c.pos, c.vel, zero);
        if (!ok) {
            std::cout << "  setTip(velocity) failed\n";
            all_pass = false;
            ++fail_vel;
            ++fail_force;
            continue;
        }

        target_front = leg.getMotorTarget(Leg2Dof::FRONT);
        target_rear = leg.getMotorTarget(Leg2Dof::REAR);

        motorPos = Eigen::Vector2f(target_front.pos, target_rear.pos);
        motorVel = Eigen::Vector2f(target_front.vel, target_rear.vel);
        motorTau = Eigen::Vector2f(0.0f, 0.0f);

        upd_ok = leg.updateMotor(motorPos, motorVel, motorTau);
        if (!upd_ok) {
            std::cout << "  updateMotor(velocity) failed\n";
            all_pass = false;
            ++fail_vel;
            ++fail_force;
            continue;
        }

        tip = leg.getTipCurrent();
        bool pass_vel = approxEqual(tip.linerVel[0], c.vel[0], 1e-2f) &&
                        approxEqual(tip.linerVel[2], c.vel[2], 1e-2f);

        std::cout << "  delta vel: vx=" << tip.linerVel[0] - c.vel[0]
                  << " vz=" << tip.linerVel[2] - c.vel[2] << "\n";
        printResult("VELOCITY", pass_vel);
        all_pass = all_pass && pass_vel;
        if (!pass_vel) {
            ++fail_vel;
        }

        ok = leg.setTip(c.pos, zero, c.force);
        if (!ok) {
            std::cout << "  setTip(force) failed\n";
            all_pass = false;
            ++fail_force;
            continue;
        }

        target_front = leg.getMotorTarget(Leg2Dof::FRONT);
        target_rear = leg.getMotorTarget(Leg2Dof::REAR);

        motorPos = Eigen::Vector2f(target_front.pos, target_rear.pos);
        motorVel = Eigen::Vector2f(0.0f, 0.0f);
        motorTau = Eigen::Vector2f(target_front.tau, target_rear.tau);

        upd_ok = leg.updateMotor(motorPos, motorVel, motorTau);
        if (!upd_ok) {
            std::cout << "  updateMotor(force) failed\n";
            all_pass = false;
            ++fail_force;
            continue;
        }

        tip = leg.getTipCurrent();
        bool pass_force = approxEqual(tip.tau[0], c.force[0], 1e-2f) &&
                          approxEqual(tip.tau[2], c.force[2], 1e-2f);

        std::cout << "  delta force: fx=" << tip.tau[0] - c.force[0]
                  << " fz=" << tip.tau[2] - c.force[2] << "\n";
        printResult("FORCE", pass_force);
        all_pass = all_pass && pass_force;
        if (!pass_force) {
            ++fail_force;
        }

        std::cout << "----------------------------------------\n";
    }

    std::cout << "\nSummary:\n";
    std::cout << "  POSITION fails: " << fail_pos << "/" << case_count << "\n";
    std::cout << "  VELOCITY fails: " << fail_vel << "/" << case_count << "\n";
    std::cout << "  FORCE fails   : " << fail_force << "/" << case_count << "\n";

    return all_pass ? 0 : 1;
}