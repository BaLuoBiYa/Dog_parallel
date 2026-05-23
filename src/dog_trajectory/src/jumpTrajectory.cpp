#include "dog_trajectory/jumpTrajectory.hpp"

namespace Dog8Dof {

    JumpTrajectory::JumpTrajectory(const std::string &configPath, rclcpp::Duration period)
        : BaseTrajectory(configPath, period) {
        reloadParameters();
    }

    bool JumpTrajectory::reloadParameters() {
        configFile_ = YAML::LoadFile(configPath_);
        for (const auto &node : configFile_) {
            uint8_t parID = node.first.as<uint8_t>();
            try {
                parameters_[parID] = node.second.as<parameters_t>();
            } catch (const YAML::Exception &e) {
                RCLCPP_ERROR(
                    logger_, "YAML Error in parameter ID %d: %s", parID, e.what());
                return false;
            }
        }

        // 构造内部 StandTrajectory（同路径 YAML 可能不是 Stand 格式，忽略异常即可）
        try {
            smooth_ = std::make_unique<StandTrajectory>(
                configPath_, rclcpp::Duration::from_nanoseconds(controlDurationNanoSec_));
        } catch (const std::invalid_argument &) {
            // Jump YAML 不支持 StandTrajectory 直接反序列化，后续通过 configure() 设置
        }

        return true;
    }

    bool JumpTrajectory::setTarget(uint8_t parID) {
        if (phase_ != FINISHED) {
            RCLCPP_WARN(logger_, "Cannot set new target while trajectory is active.");
            return false;
        }
        phase_ = CROUCH; // 从蹲伏开始
        isInit_ = false;
        targetSet_ = true;
        currentParID_ = parID;
        return true;
    }

    // ---- 参数映射：Jump → Stand -------
    static StandTrajectory::parameters_t
    jumpToStandParams(const JumpTrajectory::parameters_t &jp, float duration) {
        StandTrajectory::parameters_t sp;
        sp.transitionTime = duration;
        sp.transitionKp = jp.smoothKp;
        sp.transitionKd = jp.smoothKd;
        sp.standKp = jp.smoothKp;
        sp.standKd = jp.smoothKd;
        sp.transitionVelMax = 500.0f;
        sp.reachThresholdSQ = 0.0001f;
        sp.stepHeightFactor = 0.0f; // 纯 Z 轴运动，无需拱形
        sp.minReplanTime = 0.02f;
        return sp;
    }

    // =========================================================================
    //  generate 跳跃步态必然从 CROUCH 阶段开始，委托 StandTrajectory 生成过渡轨迹
    // =========================================================================
    TrajectoryPoint JumpTrajectory::generate(const Body &currentTip,
                                             const rclcpp::Time now) {

        if (!targetSet_) {
            throw std::logic_error("Target not set. Call setTarget() with valid "
                                   "ID before generate().");
        }
        launchTime_ = now; // 记录绝对起点
        smooth_->configure(jumpToStandParams(parameters_[currentParID_],
                                             parameters_[currentParID_].crouchDuration),
                           0.0f,
                           parameters_[currentParID_].startZ);
        idelTipState_ = currentTip; // 以当前状态为锚点，生成下蹲过渡;
        isInit_ = true;
        target_ = smooth_->generate(currentTip, now);
        return target_;
    }

    TrajectoryPoint JumpTrajectory::get(const rclcpp::Time &currentTime,
                                        const Body *currentTip) {
        if (!isInit_) {
            throw std::logic_error("not init yet. Call generate() before get().");
        }

        const float dt = (currentTime - lastUpdateTime_).seconds();
        const auto &param = parameters_[currentParID_];
        if (dt <= 0.0f) {
            // 同一时刻多次调用，直接返回上次结果
            return target_;
        }
        lastUpdateTime_ = currentTime;

        if (currentTip == nullptr) {
            throw std::invalid_argument(
                "Current tip state is required for JumpTrajectory.");
        }

        Phase currentPhase = getCurrentPhase(currentTime);

        // 状态切换时的初始化
        if (currentPhase != phase_) {
            switch (currentPhase) {
            case CROUCH:
                throw std::logic_error(
                    "CROUCH phase should be initialized in generate(), not get()!");
                // CROUCH从generate()开始，不可能在get()中切换进入，若切换到CROUCH说明generate()未正确调用，抛出异常
                break;
            case PUSH:
                phase_ = PUSH;
                // 进入 PUSH：TODO --- IGNORE ---
                break;
            case FLIGHT:
                smooth_->configure(jumpToStandParams(param, param.flightDuration),
                                   -currentTip->pos[0],
                                   currentTip->pos[2]);
                // 进入
                // FLIGHT：以当前状态为起点，水平位置反向，高度保持不变，过渡到冲击吸收

                target_ = smooth_->generate(*currentTip, currentTime);
                idelTipState_ = Body(target_.pos,
                                     target_.vel,
                                     target_.tau); // 以站立过渡的初始点为新状态锚点

                phase_ = FLIGHT;
                break;
            case ABSORPTION:
                idelTipState_ = *currentTip;
                phase_ = ABSORPTION;
                break;
            case RECOVER:
                smooth_->configure(jumpToStandParams(param, param.recoverDuration),
                                   0.0f,
                                   param.legLengthStand);
                // 进入 RECOVER：以当前状态为起点，水平位置回到目标，高度过渡到正常站立
                target_ = smooth_->generate(*currentTip, currentTime);
                idelTipState_ = Body(target_.pos,
                                     target_.vel,
                                     target_.tau); // 以飞行过渡的初始点为新状态

                phase_ = RECOVER;
                break;
            case FINISHED:
                finished_ = true;
                targetSet_ = false;
                isInit_ = false;
                // 进入 FINISHED：保持最后状态不变
                break;
            }

            return target_;
            // 处理完状态切换后直接返回，等待下一周期根据新状态生成轨迹
        }

        switch (currentPhase) {
        case CROUCH:
        case FLIGHT:
        case RECOVER:
            target_ = smooth_->get(currentTime, &idelTipState_);
            idelTipState_ = Body(target_.pos, target_.vel, target_.tau);
            break;
            // 这三个阶段都使用 smooth_ 生成轨迹，且以理想状态
            // 的输出作为状态追踪锚点，确保平滑过渡
        case PUSH:
            // PUSH阶段轨迹生成：TODO --- IGNORE ---
            break;
        case ABSORPTION:
            target_ = getAbsorptionTarget(param.absorbDamp);
            idelTipState_ = *currentTip;
            // 被动阻尼，没有明确目标位置
            break;
        case FINISHED:
            isInit_ = false;
            targetSet_ = false;
            RCLCPP_INFO(logger_, "Jump Finished");
            break;
        }
        return target_;
    }

    JumpTrajectory::Phase
    JumpTrajectory::getCurrentPhase(const rclcpp::Time &currentTime) {
        const float elapsed = (currentTime - launchTime_).seconds();
        const auto &param = parameters_[currentParID_];

        // 预计算累计边界
        const float t_crouch = param.crouchDuration;
        const float t_push = t_crouch + param.pushDuration;
        const float t_flight = t_push + param.flightDuration;
        const float t_absorb = t_flight + param.absorbDuration;
        const float t_recover = t_absorb + param.recoverDuration;

        if (elapsed <= t_crouch) {
            return CROUCH;
        } else if (elapsed <= t_push) {
            return PUSH;
        } else if (elapsed <= t_flight) {
            return FLIGHT;
        } else if (elapsed <= t_absorb) {
            return ABSORPTION;
        } else if (elapsed <= t_recover) {
            return RECOVER;
        } else {
            return FINISHED;
        }
    }

    TrajectoryPoint JumpTrajectory::getAbsorptionTarget(float absorbDamp) {
        TrajectoryPoint absorptionTarget;
        absorptionTarget.pos = idelTipState_.pos;
        absorptionTarget.vel.setZero();
        absorptionTarget.tau.setZero();
        absorptionTarget.kp[0] = 0.0f;
        absorptionTarget.kp[1] = 0.0f;
        absorptionTarget.kd[0] = absorbDamp;
        absorptionTarget.kd[1] = absorbDamp;
        // 生成一个以当前状态为锚点，零速度，零力矩，纯阻尼控制的吸收目标
        return absorptionTarget;
    }

} // namespace Dog8Dof

namespace YAML {
    template <> struct convert<Dog8Dof::JumpTrajectory::parameters_t> {
        static Node encode(const Dog8Dof::JumpTrajectory::parameters_t &rhs) {
            Node node;
            node["launchSpeed"] = rhs.launchSpeed;
            node["launchAngle"] = rhs.launchAngle;
            node["smoothKp"] = rhs.smoothKp;
            node["smoothKd"] = rhs.smoothKd;

            node["crouchDuration"] = rhs.crouchDuration;

            node["pushDuration"] = rhs.pushDuration;
            node["pushKp"] = rhs.pushKp;
            node["pushKd"] = rhs.pushKd;
            node["startZ"] = rhs.startZ;

            node["flightDuration"] = rhs.flightDuration;

            node["absorbDuration"] = rhs.absorbDuration;
            node["absorbDamp"] = rhs.absorbDamp;

            node["recoverDuration"] = rhs.recoverDuration;
            node["legLengthStand"] = rhs.legLengthStand;
            return node;
        }

        static bool decode(const Node &node, Dog8Dof::JumpTrajectory::parameters_t &rhs) {
            rhs.launchSpeed = node["launchSpeed"].as<float>(0.0f);
            rhs.launchAngle = node["launchAngle"].as<float>(3.0f);
            rhs.smoothKp = node["smoothKp"].as<float>(2.0f);
            rhs.smoothKd = node["smoothKd"].as<float>(0.02f);

            rhs.crouchDuration = node["crouchDuration"].as<float>(0.20f);

            rhs.pushDuration = node["pushDuration"].as<float>(0.15f);
            rhs.pushKp = node["pushKp"].as<float>(3.0f);
            rhs.pushKd = node["pushKd"].as<float>(0.05f);
            rhs.startZ = node["startZ"].as<float>(30.0f);

            rhs.flightDuration = node["flightDuration"].as<float>(0.30f);

            rhs.absorbDuration = node["absorbDuration"].as<float>(0.10f);
            rhs.absorbDamp = node["absorbDamp"].as<float>(4.0f);

            rhs.recoverDuration = node["recoverDuration"].as<float>(0.25f);
            rhs.legLengthStand = node["legLengthStand"].as<float>(270.0f);
            return true;
        }
    };
} // namespace YAML
