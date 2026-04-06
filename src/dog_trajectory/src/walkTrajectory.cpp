#include "dog_trajectory/walkTrajectory.hpp"
#include "dog_tools/quinticPolynomial.hpp"
#include <limits>

namespace Dog8Dof {
    WalkTrajectory::WalkTrajectory(const std::string &configPath, rclcpp::Duration period)
        : BaseTrajectory(configPath, period) {
        if (!reloadParameters()) {
            throw std::invalid_argument("Could not load Config YAML!");
        }
        // 加载参数
    }

    bool WalkTrajectory::reloadParameters() {
        configFile_ = YAML::LoadFile(configPath_);
        // 重新加载参数文件
        try {
            parameters_ = configFile_.as<parameters_t>();
        } catch (const YAML::Exception &e) {
            std::cout << "YAML Error" << e.what() << std::endl;
            return false;
        }

        if (parameters_.swingDutyCycle >= 1.0f) {
            std::cout << "Swing duty cycle can't larger than 1!" << std::endl;
            return false;
        }

        if (parameters_.transitionVelMax <= 0.0f) {
            std::cout << "Transition velocity limit must greater than 0!" << std::endl;
            return false;
        }
        if (parameters_.deltaSpeed[0] < 0.0f) {
            std::cout << "Delta speed x must greater than 0!" << std::endl;
            return false;
        }
        if (parameters_.deltaSpeed[1] < 0.0f) {
            std::cout << "Delta speed z must greater than 0!" << std::endl;
            return false;
        }
        return true;
    }

    bool WalkTrajectory::setTarget(float stepLength,
                                   float stepHeight,
                                   float baseHeight,
                                   float phaseOffset,
                                   bool useNearestStart) {
        if (stepHeight <= 0.0f) {
            std::cout << "Step height must greater than 0!" << std::endl;
            return false;
        }
        if (baseHeight > Const::BODY_HEIGHT_MAX || baseHeight < Const::BODY_HEIGHT_MIN) {
            std::cout << "Base height must between " << Const::BODY_HEIGHT_MAX << " and "
                      << Const::BODY_HEIGHT_MIN << std::endl;
            return false;
        }

        stepHeight_ = stepHeight;
        stepLength_ = stepLength;
        baseHeight_ = baseHeight;
        useNearestStart_ = useNearestStart;

        // 归一化到 [0, 1)
        phaseOffset_ = phaseOffset - std::floor(phaseOffset);
        if (phaseOffset_ < 0.0f) {
            phaseOffset_ += 1.0f;
        }
        return true;
    }

    bool WalkTrajectory::generate(const Body &currentTip) {
        finished_ = false;
        trajectory_.progress = 0;
        target_ = TrajectoryPoint();
        trajectory_ = Trajectory(0);

        const size_t totalSteps = stepsFromDurationNs(parameters_.totalTime * 1e9);
        const float dt = static_cast<float>(durationNanoSec_) * 1e-9f;
        const float TCycle = dt * static_cast<float>(totalSteps);

        Trajectory closedCycle;
        if (std::abs(stepLength_) < 1e-3f) {
            closedCycle = generateZeroLength();
        } else {
            closedCycle = generateClosedCycle(TCycle, totalSteps);
        }

        if (closedCycle.points.empty()) {
            return false;
        }

        const size_t n = closedCycle.points.size();
        const size_t startIndex =
            useNearestStart_
                ? findNearestPointIndex(closedCycle, currentTip)
                : static_cast<size_t>(std::lround(phaseOffset_ * static_cast<float>(n))) %
                      n;

        Trajectory phaseCycle = rotateCycleToStart(closedCycle, startIndex);
        if (phaseCycle.points.empty()) {
            return false;
        }

        // 运行中重规划：仍按最近点直接续接
        if (useNearestStart_) {
            trajectory_ = std::move(phaseCycle);
            return true;
        }

        // 相位启动：先从当前位置过渡到相位起点
        const TrajectoryPoint &phaseStart = phaseCycle.points.front();

        const Eigen::Vector2f currentPos(currentTip.pos[0], currentTip.pos[2]);
        const Eigen::Vector2f currentVel(currentTip.linerVel[0], currentTip.linerVel[2]);
        const Eigen::Vector2f phaseStartPos(phaseStart.pos[0], phaseStart.pos[2]);
        const Eigen::Vector2f phaseStartVel(phaseStart.vel[0], phaseStart.vel[2]);
        const Eigen::Vector2f zeroAcc = Eigen::Vector2f::Zero();

        // 步向符号：>0 前进，<0 后退
        const float stepDir = (stepLength_ >= 0.0f) ? 1.0f : -1.0f;
        const bool startInForwardDir = (phaseStartPos[0] * stepDir) >= 0.0f;

        // 过渡时长与步数
        const size_t transitionSteps =
            std::clamp(stepsFromDurationNs(parameters_.startblendTimeNs, 4, 40),
                       static_cast<size_t>(4),
                       static_cast<size_t>(40));
        const float transitionTime =
            std::max(0.08f, static_cast<float>(transitionSteps) * dt);

        Trajectory transition(0);

        if (startInForwardDir) {
            // 前向相位起点：抬腿移动到起点
            const Eigen::Vector2f skyDir = getHeightVec(Eigen::Vector2f(stepDir, 0.0f));
            const float liftMag = std::max(5.0f, std::abs(stepHeight_) * 0.6f);

            const size_t steps1 = std::max<size_t>(2, transitionSteps / 2);
            const size_t steps2 = std::max<size_t>(2, transitionSteps - steps1);
            const float t1 = transitionTime * 0.5f;
            const float t2 = transitionTime - t1;

            const Eigen::Vector2f midPos =
                0.5f * (currentPos + phaseStartPos) + skyDir * liftMag;

            const Eigen::Vector2f midVel = QuinticPolynomial::solveMidVel2D(
                currentPos, midPos, phaseStartPos, currentVel, phaseStartVel, t1, t2);

            const Eigen::Vector2f midAcc = QuinticPolynomial::solveMidAcc2D(currentPos,
                                                                            currentVel,
                                                                            zeroAcc,
                                                                            midPos,
                                                                            midVel,
                                                                            phaseStartPos,
                                                                            phaseStartVel,
                                                                            zeroAcc,
                                                                            t1,
                                                                            t2);

            Trajectory first = QuinticPolynomial::generate(
                currentPos, midPos, currentVel, midVel, zeroAcc, midAcc, t1, steps1);

            Trajectory second = QuinticPolynomial::generate(midPos,
                                                            phaseStartPos,
                                                            midVel,
                                                            phaseStartVel,
                                                            midAcc,
                                                            zeroAcc,
                                                            t2,
                                                            steps2);

            if (!first.points.empty() && !second.points.empty()) {
                if (first.points.back().pos.isApprox(second.points.front().pos)) {
                    second.points.erase(second.points.begin());
                    if (!second.time.empty()) {
                        second.time.erase(second.time.begin());
                    }
                }
            }

            transition = first + second;
        } else {
            // 后向相位起点：从地面移动到起点
            const float groundZ =
                std::max({baseHeight_, currentPos[1], phaseStartPos[1]});
            const Eigen::Vector2f groundStart(currentPos[0], groundZ);
            const Eigen::Vector2f groundEnd(phaseStartPos[0], groundZ);

            const size_t steps1 = std::max<size_t>(2, transitionSteps / 2);
            const size_t steps2 = std::max<size_t>(2, transitionSteps - steps1);
            const float t1 = transitionTime * 0.5f;
            const float t2 = transitionTime - t1;

            const Eigen::Vector2f midPos = 0.5f * (groundStart + groundEnd);

            const Eigen::Vector2f groundStartVel(currentVel[0], 0.0f);
            const Eigen::Vector2f groundEndVel(phaseStartVel[0], 0.0f);

            const Eigen::Vector2f midVel = QuinticPolynomial::solveMidVel2D(
                groundStart, midPos, groundEnd, groundStartVel, groundEndVel, t1, t2);

            const Eigen::Vector2f midAcc =
                QuinticPolynomial::solveMidAcc2D(groundStart,
                                                 groundStartVel,
                                                 zeroAcc,
                                                 midPos,
                                                 midVel,
                                                 groundEnd,
                                                 groundEndVel,
                                                 zeroAcc,
                                                 t1,
                                                 t2);

            Trajectory first = QuinticPolynomial::generate(
                groundStart, midPos, groundStartVel, midVel, zeroAcc, midAcc, t1, steps1);

            Trajectory second = QuinticPolynomial::generate(
                midPos, groundEnd, midVel, groundEndVel, midAcc, zeroAcc, t2, steps2);

            if (!first.points.empty() && !second.points.empty()) {
                if (first.points.back().pos.isApprox(second.points.front().pos)) {
                    second.points.erase(second.points.begin());
                    if (!second.time.empty()) {
                        second.time.erase(second.time.begin());
                    }
                }
            }

            transition = first + second;
        }

        // 去掉过渡段末点与相位段首点重复
        if (!transition.points.empty() && !phaseCycle.points.empty()) {
            if (transition.points.back().pos.isApprox(phaseCycle.points.front().pos)) {
                phaseCycle.points.erase(phaseCycle.points.begin());
                if (!phaseCycle.time.empty()) {
                    phaseCycle.time.erase(phaseCycle.time.begin());
                }
            }
        }

        trajectory_ = transition + phaseCycle;
        return !trajectory_.points.empty();
    }

    bool WalkTrajectory::start(const rclcpp::Time &startTime) {
        startTimeNanoSec_ = static_cast<size_t>(startTime.nanoseconds());
        trajectory_.progress = 0;

        // generate 已经判定“无需过渡”时，直接保持 target_
        if (finished_) {
            return true;
        }

        if (trajectory_.points.empty()) {
            return false;
        }

        for (size_t i = 0; i < trajectory_.time.size(); ++i) {
            trajectory_.time[i] = startTimeNanoSec_ + i * durationNanoSec_;
        }

        target_ = trajectory_.points.front();
        target_.kp[0] = parameters_.walkKp;
        target_.kp[1] = parameters_.walkKp;
        target_.kd[0] = parameters_.walkKd;
        target_.kd[1] = parameters_.walkKd;
        return true;
    }

    TrajectoryPoint WalkTrajectory::get(const rclcpp::Time &currentTime) {
        if (finished_) {
            return target_;
        }

        const size_t pointCount = trajectory_.points.size();
        const size_t currentNanoSec = static_cast<size_t>(currentTime.nanoseconds());

        auto assignTransitionGains = [this]() {
            target_.kp[0] = parameters_.walkKp;
            target_.kp[1] = parameters_.walkKp;
            target_.kd[0] = parameters_.walkKd;
            target_.kd[1] = parameters_.walkKd;
        };

        if (currentNanoSec <= trajectory_.time.front()) {
            target_ = trajectory_.points.front();
            assignTransitionGains();
            trajectory_.progress = 0;
            return target_;
        }

        if (currentNanoSec >= trajectory_.time.back()) {
            target_ = trajectory_.points.back();
            assignTransitionGains();
            trajectory_.progress = pointCount;
            finished_ = true;
            return target_;
        }

        interpolate(currentNanoSec);
        if (trajectory_.progress == trajectory_.points.size()) {
            finished_ = true;
        }
        assignTransitionGains();
        return target_;
    }

    Trajectory WalkTrajectory::generateClosedCycle(const float cycleTime,
                                                   const size_t totalSteps) {
        size_t swingSteps = static_cast<size_t>(
            std::lround(static_cast<float>(totalSteps) * parameters_.swingDutyCycle));
        swingSteps = std::clamp(swingSteps, static_cast<size_t>(2), totalSteps - 2);
        const size_t stanceSteps = totalSteps - swingSteps;

        Trajectory swing = generateSwing(cycleTime, swingSteps);
        Trajectory stance = generateStance(cycleTime, stanceSteps);
        const Trajectory blend = generateBlend(swing, stance);

        return swing + blend + stance;
    }

    size_t WalkTrajectory::findNearestPointIndex(const Trajectory &cycle,
                                                 const Body &currentTip) const {
        if (cycle.points.empty()) {
            return 0;
        }

        const Eigen::Vector2f currentPos(currentTip.pos[0], currentTip.pos[2]);
        size_t nearestIndex = 0;
        float bestDistSq = std::numeric_limits<float>::max();

        for (size_t i = 0; i < cycle.points.size(); ++i) {
            const Eigen::Vector2f pointPos(cycle.points[i].pos[0],
                                           cycle.points[i].pos[2]);
            const float distSq = (pointPos - currentPos).squaredNorm();
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                nearestIndex = i;
            }
        }

        return nearestIndex;
    }

    Trajectory WalkTrajectory::rotateCycleToStart(const Trajectory &cycle,
                                                  size_t startIndex) const {
        if (cycle.points.empty()) {
            return Trajectory(0);
        }

        const size_t n = cycle.points.size();
        startIndex %= n;
        if (startIndex == 0) {
            return cycle;
        }

        Trajectory rotated(0);
        rotated.points.reserve(n);
        rotated.time.resize(n);
        rotated.progress = 0;

        for (size_t i = 0; i < n; ++i) {
            rotated.points.push_back(cycle.points[(startIndex + i) % n]);
        }

        return rotated;
    }

    Trajectory WalkTrajectory::generateStance(const float transitionTime,
                                              const size_t steps) {
        const Eigen::Vector2f stanceStartPos(stepLength_ * 0.5f, baseHeight_);
        const Eigen::Vector2f stanceFinishPos(-stepLength_ * 0.5f, baseHeight_);
        // 支撑相起始位置

        const float phaseTime = transitionTime * (1.0f - parameters_.swingDutyCycle);
        const Eigen::Vector2f stanceAcc(0.0f, 0.0f);
        const Eigen::Vector2f stanceStartVel(0.0f, parameters_.deltaSpeed[1]);
        const Eigen::Vector2f stanceFinishVel(0.0f, -parameters_.deltaSpeed[1]);
        // 支撑相边界速度设为 x=0，避免与摆动相切换时 x 速度跳变

        Trajectory res = QuinticPolynomial::generate(stanceStartPos,
                                                     stanceFinishPos,
                                                     stanceStartVel,
                                                     stanceFinishVel,
                                                     stanceAcc,
                                                     stanceAcc,
                                                     phaseTime,
                                                     steps);

        const float pushForce = parameters_.horizontalPushGain * stepLength_;
        const float pressForce = Const::BODY_MASS * Const::G * 0.5 +
                                 parameters_.verticalPushGain * std::abs(pushForce);
        const size_t rampSteps = std::max<size_t>(1, static_cast<size_t>(steps * 0.1f));

        auto smoothstep01 = [](float x) {
            x = std::clamp(x, 0.0f, 1.0f);
            return x * x * (3.0f - 2.0f * x);
        };

        for (size_t i = 0; i < steps; i++) {
            float scale = 1.0f;
            if (i < rampSteps) {
                const float s = static_cast<float>(i + 1) / static_cast<float>(rampSteps);
                scale = smoothstep01(s);
            } else if (i + rampSteps >= steps) {
                const size_t tailIndex = steps - i;
                const float s =
                    static_cast<float>(tailIndex) / static_cast<float>(rampSteps);
                scale = smoothstep01(s);
            }

            res.points[i].tau[0] = pushForce * scale;
            res.points[i].tau[1] = 0.0f;
            res.points[i].tau[2] = pressForce * scale;
        }
        // 添加前馈支撑力矩
        return res;
    }

    Trajectory WalkTrajectory::generateSwing(const float transitionTime,
                                             const size_t steps) {
        const Eigen::Vector2f zeroAcc(0.0f, 0.0f);

        const Eigen::Vector2f swingStartPos(-stepLength_ * 0.5f, baseHeight_);
        const Eigen::Vector2f swingFinishPos(stepLength_ * 0.5f, baseHeight_);

        const Eigen::Vector2f tangentVec = swingFinishPos - swingStartPos;
        const Eigen::Vector2f directionVec = getHeightVec(tangentVec);

        const float phaseTime = transitionTime * parameters_.swingDutyCycle;
        const float T1 = phaseTime * 0.5;
        const float T2 = T1;

        const size_t steps1 = static_cast<size_t>(steps * 0.5f);
        const size_t steps2 = steps - steps1;
        // 计算摆动相两段的时间和分辨率占比

        const Eigen::Vector2f swingStartVel(0.0f, -parameters_.deltaSpeed[1]);
        const Eigen::Vector2f swingFinishVel(0.0f, parameters_.deltaSpeed[1]);
        // 起始加入微小的向上速度，结束添加微小向下速度，避免曲线退化为直线

        const Eigen::Vector2f swingMidPos =
            (swingFinishPos + swingStartPos) * 0.5 + directionVec * stepHeight_;
        const Eigen::Vector2f swingMidVel =
            QuinticPolynomial::solveMidVel2D(swingStartPos,
                                             swingMidPos,
                                             swingFinishPos,
                                             swingStartVel,
                                             swingFinishVel,
                                             T1,
                                             T2);
        const Eigen::Vector2f swingMidAcc =
            QuinticPolynomial::solveMidAcc2D(swingStartPos,
                                             swingStartVel,
                                             zeroAcc,
                                             swingMidPos,
                                             swingMidVel,
                                             swingFinishPos,
                                             swingFinishVel,
                                             zeroAcc,
                                             T1,
                                             T2);
        // 计算中点属性
        Trajectory first = QuinticPolynomial::generate(swingStartPos,
                                                       swingMidPos,
                                                       swingStartVel,
                                                       swingMidVel,
                                                       zeroAcc,
                                                       swingMidAcc,
                                                       T1,
                                                       steps1);

        Trajectory second = QuinticPolynomial::generate(swingMidPos,
                                                        swingFinishPos,
                                                        swingMidVel,
                                                        swingFinishVel,
                                                        swingMidAcc,
                                                        zeroAcc,
                                                        T2,
                                                        steps2);
        return first + second;
    }

    Trajectory WalkTrajectory::generateBlend(const Trajectory &lhs,
                                             const Trajectory &rhs) {
        const size_t blendSteps =
            std::clamp(stepsFromDurationNs(parameters_.blendTimeNs, 3, 20),
                       static_cast<size_t>(3),
                       static_cast<size_t>(20));
        const float blendTime = static_cast<float>(parameters_.blendTimeNs) / 1e9;
        if (lhs.points.empty() || rhs.points.empty()) {
            return Trajectory(0);
        }

        const TrajectoryPoint &p0 = lhs.points.back();
        const TrajectoryPoint &p1 = rhs.points.front();

        const Eigen::Vector2f blendStartPos(p0.pos[0], p0.pos[2]);
        const Eigen::Vector2f blendFinishPos(p1.pos[0], p1.pos[2]);
        const Eigen::Vector2f blendStartVel(p0.vel[0], p0.vel[2]);
        const Eigen::Vector2f blendFinishVel(p1.vel[0], p1.vel[2]);
        const Eigen::Vector2f zeroAcc = Eigen::Vector2f::Zero();

        Trajectory blend = QuinticPolynomial::generate(blendStartPos,
                                                       blendFinishPos,
                                                       blendStartVel,
                                                       blendFinishVel,
                                                       zeroAcc,
                                                       zeroAcc,
                                                       blendTime,
                                                       blendSteps);

        for (size_t i = 0; i < blend.points.size(); ++i) {
            const float s =
                (blend.points.size() <= 1)
                    ? 1.0f
                    : static_cast<float>(i) / static_cast<float>(blend.points.size() - 1);
            blend.points[i].tau = p0.tau + (p1.tau - p0.tau) * s;
        }

        if (blend.points.size() > 2) {
            blend.points.erase(blend.points.begin());
            blend.points.pop_back();
            blend.time.resize(blend.points.size());
        }

        return blend;
    };

    Trajectory WalkTrajectory::generateZeroLength() {
        const size_t totalSteps =
            stepsFromDurationNs(static_cast<size_t>(parameters_.totalTime * 1e9));
        if (totalSteps < 4) {
            return Trajectory(0);
        }

        const float dt = static_cast<float>(durationNanoSec_) * 1e-9f;
        const float cycleTime = dt * static_cast<float>(totalSteps);

        size_t swingSteps = static_cast<size_t>(
            std::lround(static_cast<float>(totalSteps) * parameters_.swingDutyCycle));
        swingSteps = std::clamp(swingSteps, static_cast<size_t>(2), totalSteps - 2);
        const size_t stanceSteps = totalSteps - swingSteps;

        const float x0 = 0.0f;
        const float z0 = baseHeight_;
        const float zLift = std::max(Const::BODY_HEIGHT_MIN, baseHeight_ - stepHeight_);

        const Eigen::Vector2f pStart(x0, z0);
        const Eigen::Vector2f pMid(x0, zLift);
        const Eigen::Vector2f pEnd(x0, z0);

        const Eigen::Vector2f zeroVel(0.0f, 0.0f);
        const Eigen::Vector2f zeroAcc(0.0f, 0.0f);

        const float swingTime = cycleTime * parameters_.swingDutyCycle;
        const float T1 = swingTime * 0.5f;
        const float T2 = swingTime - T1;

        size_t steps1 = swingSteps / 2;
        if (steps1 == 0) {
            steps1 = 1;
        }
        const size_t steps2 = swingSteps - steps1;

        Trajectory swingUp = QuinticPolynomial::generate(
            pStart, pMid, zeroVel, zeroVel, zeroAcc, zeroAcc, T1, steps1);
        Trajectory swingDown = QuinticPolynomial::generate(
            pMid, pEnd, zeroVel, zeroVel, zeroAcc, zeroAcc, T2, steps2);
        Trajectory swing = swingUp + swingDown;

        const float stanceTime = cycleTime - swingTime;
        Trajectory stance = QuinticPolynomial::generate(
            pEnd, pEnd, zeroVel, zeroVel, zeroAcc, zeroAcc, stanceTime, stanceSteps);

        const float pressForce = Const::BODY_MASS * Const::G * 0.5f;
        const size_t rampSteps =
            std::max<size_t>(1, static_cast<size_t>(stance.points.size() * 0.1f));

        auto smoothstep01 = [](float x) {
            x = std::clamp(x, 0.0f, 1.0f);
            return x * x * (3.0f - 2.0f * x);
        };

        for (size_t i = 0; i < stance.points.size(); ++i) {
            float scale = 1.0f;
            if (i < rampSteps) {
                const float s = static_cast<float>(i + 1) / static_cast<float>(rampSteps);
                scale = smoothstep01(s);
            } else if (i + rampSteps >= stance.points.size()) {
                const size_t tailIndex = stance.points.size() - i;
                const float s =
                    static_cast<float>(tailIndex) / static_cast<float>(rampSteps);
                scale = smoothstep01(s);
            }

            stance.points[i].tau[0] = 0.0f;
            stance.points[i].tau[1] = 0.0f;
            stance.points[i].tau[2] = pressForce * scale;
        }

        const Trajectory blend = generateBlend(swing, stance);
        return swing + blend + stance;
    }

} // namespace Dog8Dof

namespace YAML {
    template <> struct convert<Dog8Dof::WalkTrajectory::parameters_t> {
        static Node encode(const Dog8Dof::WalkTrajectory::parameters_t &rhs) {
            Node node;
            node["walkKp"] = rhs.walkKp;
            node["walkKd"] = rhs.walkKd;

            node["reachThreshold"] = std::sqrt(rhs.reachThresholdSQ);
            node["transitionVelMax"] = rhs.transitionVelMax;

            node["totalTime"] = rhs.totalTime;
            node["swingDutyCycle"] = rhs.swingDutyCycle;

            node["pushGain"]["horizontal"] = rhs.horizontalPushGain;
            node["pushGain"]["vertical"] = rhs.verticalPushGain;

            node["deltaSpeed"]["x"] = rhs.deltaSpeed[0];
            node["deltaSpeed"]["z"] = rhs.deltaSpeed[1];

            node["blendTime"] = rhs.blendTimeNs / 1e9;
            node["startblendTime"] = rhs.startblendTimeNs / 1e9;
            return node;
        }

        static bool decode(const Node &node, Dog8Dof::WalkTrajectory::parameters_t &rhs) {
            rhs.walkKp = node["walkKp"].as<float>(0.001);
            rhs.walkKd = node["walkKd"].as<float>(0.001);

            rhs.reachThresholdSQ = node["reachThreshold"].as<float>(100.0) *
                                   node["reachThreshold"].as<float>(100.0);
            rhs.transitionVelMax = node["transitionVelMax"].as<float>(100.0);

            rhs.horizontalPushGain = node["pushGain"]["horizontal"].as<float>(0.01);
            rhs.verticalPushGain = node["pushGain"]["vertical"].as<float>(0.01);

            rhs.swingDutyCycle = node["swingDutyCycle"].as<float>(0.5);
            rhs.totalTime = node["totalTime"].as<float>(2.0);

            rhs.deltaSpeed = Eigen::Vector2f(node["deltaSpeed"]["x"].as<float>(1.0),
                                             node["deltaSpeed"]["z"].as<float>(1.0));

            rhs.blendTimeNs = node["blendTime"].as<float>(0.03f) * 1e9;
            rhs.startblendTimeNs = node["startblendTime"].as<float>(1.0f) * 1e9;
            return true;
        }
    };
} // namespace YAML