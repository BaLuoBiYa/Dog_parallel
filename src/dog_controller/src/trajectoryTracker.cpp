/**
 * @file trajectoryTracker.cpp
 * @brief ROS2 Controller — 在线重规划步态追踪
 */

#include "dog_controller/trajectoryTracker.hpp"

namespace Dog8Dof {

    // =========================================================================
    //  Lifecycle: on_init / on_configure / on_activate
    // =========================================================================
    controller_interface::CallbackReturn TrajectoryTracker::on_init() {
        robotName_ = auto_declare<std::string>("robotName", "dog");
        standProfile_ = auto_declare<std::string>("standProfile", "");
        walkProfile_ = auto_declare<std::string>("walkProfile", "");
        jumpProfile_ = auto_declare<std::string>("jumpProfile", "");
        updateDuration_ = 1.0f / static_cast<float>(this->get_update_rate());
        idleTipX_ = auto_declare<float>("idleTipX", 0.0f);
        idleTipZ_ = auto_declare<float>("idleTipZ", 180.0f);
        commandTopic_ = auto_declare<std::string>("commandTopic", "/dog/command");
        monitorTopic_ = auto_declare<std::string>("monitorTopic", "/dog/monitor");
        maxfailure_ = auto_declare<uint8_t>("maxFailure", 5);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    TrajectoryTracker::on_configure(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;

        if (standProfile_.empty() || walkProfile_.empty()) {
            RCLCPP_ERROR(logger_, "standProfile and walkProfile must not be empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        const auto period = rclcpp::Duration::from_seconds(updateDuration_);

        try {
            LF_ = std::make_unique<GaitManager>(
                period, walkProfile_, standProfile_, jumpProfile_);
            LR_ = std::make_unique<GaitManager>(
                period, walkProfile_, standProfile_, jumpProfile_);
            RF_ = std::make_unique<GaitManager>(
                period, walkProfile_, standProfile_, jumpProfile_);
            RR_ = std::make_unique<GaitManager>(
                period, walkProfile_, standProfile_, jumpProfile_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(logger_, "TrajectoryTracker configure failed: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        converter_ = GaitTwistConverter();

        commandSub_ = this->get_node()->create_subscription<dog_msg::msg::GaitTwist>(
            commandTopic_, 1, [this](const dog_msg::msg::GaitTwist::SharedPtr msg) {
                if (commandBox_.try_set(*msg)) {
                    newCmd_ = true;
                }
            });

        registerMonitorPublisher();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    TrajectoryTracker::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(8 * 5);
        for (auto &name : jointNames_) {
            config.names.push_back(std::string(name) + "/position");
            config.names.push_back(std::string(name) + "/velocity");
            config.names.push_back(std::string(name) + "/effort");
            config.names.push_back(std::string(name) + "/kp");
            config.names.push_back(std::string(name) + "/kd");
        }
        return config;
    }

    controller_interface::InterfaceConfiguration
    TrajectoryTracker::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(8 * 3);
        for (auto &name : jointNames_) {
            config.names.push_back(std::string(name) + "/position");
            config.names.push_back(std::string(name) + "/velocity");
            config.names.push_back(std::string(name) + "/effort");
            config.names.push_back(std::string(name) + "/temperature");
        }
        return config;
    }

    controller_interface::CallbackReturn
    TrajectoryTracker::on_activate(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;

        if (!cacheInterfaceIndices()) {
            return controller_interface::CallbackReturn::ERROR;
        }

        auto initLeg = [&](std::unique_ptr<GaitManager> &gm, size_t i0, size_t i1) {
            return gm->init(Eigen::Vector2f(getPos(i0), getPos(i1)),
                            Eigen::Vector2f(getVel(i0), getVel(i1)),
                            Eigen::Vector2f(getTau(i0), getTau(i1)));
        };
        if (!initLeg(LF_, 0, 1) || !initLeg(RF_, 2, 3) || !initLeg(LR_, 4, 5) ||
            !initLeg(RR_, 6, 7)) {
            return controller_interface::CallbackReturn::ERROR;
        }

        switchToStandMode(true); // 初始化时使用初始化站起的缓冲时间
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // =========================================================================
    //  on_error — 安全回落
    // =========================================================================
    controller_interface::CallbackReturn
    TrajectoryTracker::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCLCPP_ERROR(logger_,
                     "TrajectoryTracker entered error state from %s",
                     previous_state.label().c_str());

        if (!LF_ || !LR_ || !RF_ || !RR_) {
            return controller_interface::CallbackReturn::FAILURE;
        }

        while (commandBox_.try_get().has_value()) {
        }
        newCmd_ = false;

        if (command_interfaces_.empty() || state_interfaces_.empty()) {
            if (!cacheInterfaceIndices()) {
                switchToStandMode();
                return controller_interface::CallbackReturn::FAILURE;
            }
        }

        switchToStandMode();
        return setAllJointCommands() ? controller_interface::CallbackReturn::SUCCESS
                                     : controller_interface::CallbackReturn::FAILURE;
    }

    // =========================================================================
    //  update() — 实时控制循环
    // =========================================================================
    controller_interface::return_type
    TrajectoryTracker::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;
        bool hasFaliure = false;

        // ---- 监控采样（测量 update() 真实调用频率）----
        {
            std::array<int8_t, 8> temps{};
            std::array<float, 8> positions;
            std::array<float, 8> velocities;
            std::array<float, 8> torques;
            for (size_t i = 0; i < 8; ++i) {
                temps[i] = getTemp(i);
                positions[i] = getPos(i);
                velocities[i] = getVel(i);
                torques[i] = getTau(i);
            }
            monitor_.update(temps, positions, velocities, torques);
        }

        // ---- 监控发布（1/10 降频）----
        if (monitorPub_ && ++monitorPubCounter_ >= kMonitorPubDecimation) {
            monitorPubCounter_ = 0;
            publishMonitorState();
        }

        // 1. 处理新命令（立即应用，generate 已基于当前状态重规划）
        if (newCmd_) {
            auto msg_opt = commandBox_.try_get();
            if (msg_opt.has_value()) {
                applyCommand(msg_opt.value());
            }
            newCmd_ = false;
        }

        // 2. 每周期更新所有步态管理器（传感器反馈闭环在线重规划）
        if (!updateAllGaitManagers()) {
            failureCounter_++;
            hasFaliure = true;
        }

        // 3. 逆运动学结果下发到关节命令接口
        if (!setAllJointCommands()) {
            failureCounter_++;
            hasFaliure = true;
        }

        if (failureCounter_ > maxfailure_) {
            return controller_interface::return_type::ERROR;
        }
        if (!hasFaliure) {
            failureCounter_ = 0;
        }
        // 连续多周期失败才进入
        // on_error，偶尔失败可能是通信抖动或机械抖动引起的瞬态问题，不必过于敏感
        return controller_interface::return_type::OK;
    }

    // =========================================================================
    //  applyCommand() — GaitTwist → GaitManager 转译
    // =========================================================================
    bool TrajectoryTracker::applyCommand(const dog_msg::msg::GaitTwist &msg) {
        using GT = dog_msg::msg::GaitTwist;

        // ---- 跳跃保护：跳跃未完成时忽略一切 gait 切换命令 ----
        if (LF_->isJumping() || LR_->isJumping() || RF_->isJumping() ||
            RR_->isJumping()) {
            RCLCPP_WARN(
                logger_, "Jump in progress, ignoring command (gait=%d)", msg.gait);
            return true; // 不视为错误，但拒绝切换（包括重复 JUMP 命令）
        }

        GaitParameters params[4];
        if (!converter_.convert(msg, params)) {
            RCLCPP_WARN(logger_, "GaitTwist conversion failed, falling back to stand");
            return switchToStandMode();
        }

        switch (msg.gait) {
        case GT::STAND: {
            LF_->setStandGait(0.0f, params[0].baseHeight);
            LR_->setStandGait(0.0f, params[1].baseHeight);
            RF_->setStandGait(0.0f, params[2].baseHeight);
            RR_->setStandGait(0.0f, params[3].baseHeight);
            currentMode_ = ControlMode::STAND;
            break;
        }
        case GT::WALK: {
            LF_->setWalkGait(params[0].stepLength,
                             params[0].stepHeight,
                             params[0].baseHeight,
                             params[0].stepDuration,
                             kWalkPhaseOffset[0]);
            LR_->setWalkGait(params[1].stepLength,
                             params[1].stepHeight,
                             params[1].baseHeight,
                             params[1].stepDuration,
                             kWalkPhaseOffset[1]);
            RF_->setWalkGait(params[2].stepLength,
                             params[2].stepHeight,
                             params[2].baseHeight,
                             params[2].stepDuration,
                             kWalkPhaseOffset[2]);
            RR_->setWalkGait(params[3].stepLength,
                             params[3].stepHeight,
                             params[3].baseHeight,
                             params[3].stepDuration,
                             kWalkPhaseOffset[3]);
            currentMode_ = ControlMode::WALK;
            break;
        }
        case GT::JUMP: {
            // GaitTwist.step_height 作为跳跃配置索引 parID（上位机在 JUMP
            // 模式下将所有字段置为跳跃索引）
            const uint8_t parID = static_cast<uint8_t>(msg.step_height);
            if (!LF_->setJumpGait(parID) || !LR_->setJumpGait(parID) ||
                !RF_->setJumpGait(parID) || !RR_->setJumpGait(parID)) {
                RCLCPP_WARN(logger_,
                            "Jump gait activation failed (parID=%d), falling to "
                            "stand",
                            parID);
                return switchToStandMode();
            }
            currentMode_ = ControlMode::JUMP;
            break;
        }
        default:
            RCLCPP_WARN(logger_, "Unknown gait mode");
            return false;
        }
        return true;
    }

    // =========================================================================
    //  switchToStandMode() — 安全回落站立
    // =========================================================================
    bool TrajectoryTracker::switchToStandMode(bool useInitTransition) {
        LF_->setStandGait(idleTipX_, idleTipZ_, useInitTransition);
        LR_->setStandGait(idleTipX_, idleTipZ_, useInitTransition);
        RF_->setStandGait(idleTipX_, idleTipZ_, useInitTransition);
        RR_->setStandGait(idleTipX_, idleTipZ_, useInitTransition);
        currentMode_ = ControlMode::STAND;
        return updateAllGaitManagers();
    }

    // =========================================================================
    //  updateAllGaitManagers() — 四条腿同步更新
    // =========================================================================
    bool TrajectoryTracker::updateAllGaitManagers() {
        const bool lf = LF_->update(Eigen::Vector2f(getPos(0), getPos(1)),
                                    Eigen::Vector2f(getVel(0), getVel(1)),
                                    Eigen::Vector2f(getTau(0), getTau(1)));
        const bool rf = RF_->update(Eigen::Vector2f(getPos(2), getPos(3)),
                                    Eigen::Vector2f(getVel(2), getVel(3)),
                                    Eigen::Vector2f(getTau(2), getTau(3)));
        const bool lr = LR_->update(Eigen::Vector2f(getPos(4), getPos(5)),
                                    Eigen::Vector2f(getVel(4), getVel(5)),
                                    Eigen::Vector2f(getTau(4), getTau(5)));
        const bool rr = RR_->update(Eigen::Vector2f(getPos(6), getPos(7)),
                                    Eigen::Vector2f(getVel(6), getVel(7)),
                                    Eigen::Vector2f(getTau(6), getTau(7)));
        return lf && rf && lr && rr;
    }

    // =========================================================================
    //  Hardware interface helpers
    // =========================================================================
    bool TrajectoryTracker::cacheInterfaceIndices() {
        std::unordered_map<std::string, size_t> stMap;
        std::unordered_map<std::string, size_t> cmdMap;

        for (size_t i = 0; i < state_interfaces_.size(); ++i)
            stMap[state_interfaces_[i].get_name()] = i;
        for (size_t i = 0; i < command_interfaces_.size(); ++i)
            cmdMap[command_interfaces_[i].get_name()] = i;

        for (size_t i = 0; i < 8; ++i) {
            const auto stPos = stMap.find(jointNames_[i] + "/position");
            const auto stVel = stMap.find(jointNames_[i] + "/velocity");
            const auto stEff = stMap.find(jointNames_[i] + "/effort");
            const auto stTmp = stMap.find(jointNames_[i] + "/temperature");
            const auto cmdPos = cmdMap.find(jointNames_[i] + "/position");
            const auto cmdVel = cmdMap.find(jointNames_[i] + "/velocity");
            const auto cmdEff = cmdMap.find(jointNames_[i] + "/effort");
            const auto cmdKp = cmdMap.find(jointNames_[i] + "/kp");
            const auto cmdKd = cmdMap.find(jointNames_[i] + "/kd");

            if (stPos == stMap.end() || stVel == stMap.end() || stEff == stMap.end() ||
                cmdPos == cmdMap.end() || cmdVel == cmdMap.end() ||
                cmdEff == cmdMap.end() || cmdKp == cmdMap.end() ||
                cmdKd == cmdMap.end() || stTmp == stMap.end()) {
                RCLCPP_ERROR(logger_, "Missing interface for %s", jointNames_[i].c_str());
                return false;
            }
            stateIdx_[i] = {stPos->second, stVel->second, stEff->second, stTmp->second};
            cmdIdx_[i] = {cmdPos->second,
                          cmdVel->second,
                          cmdEff->second,
                          cmdKp->second,
                          cmdKd->second};
        }
        return true;
    }

    bool TrajectoryTracker::setJointCommands(size_t legIndex, const Motor &target) {
        const auto &idx = cmdIdx_[legIndex];
        return command_interfaces_[idx.pos].set_value(static_cast<double>(target.pos)) &&
               command_interfaces_[idx.vel].set_value(static_cast<double>(target.vel)) &&
               command_interfaces_[idx.eff].set_value(static_cast<double>(target.tau)) &&
               command_interfaces_[idx.kp].set_value(static_cast<double>(target.kp)) &&
               command_interfaces_[idx.kd].set_value(static_cast<double>(target.kd));
    }

    bool TrajectoryTracker::setAllJointCommands() {
        return setJointCommands(0, LF_->getFrontTarget()) &&
               setJointCommands(1, LF_->getRearTarget()) &&
               setJointCommands(2, RF_->getFrontTarget()) &&
               setJointCommands(3, RF_->getRearTarget()) &&
               setJointCommands(4, LR_->getFrontTarget()) &&
               setJointCommands(5, LR_->getRearTarget()) &&
               setJointCommands(6, RR_->getFrontTarget()) &&
               setJointCommands(7, RR_->getRearTarget());
    }

    float TrajectoryTracker::getPos(size_t i) {
        return static_cast<float>(
            state_interfaces_[stateIdx_[i].pos].get_optional<double>().value());
    }
    float TrajectoryTracker::getVel(size_t i) {
        return static_cast<float>(
            state_interfaces_[stateIdx_[i].vel].get_optional<double>().value());
    }
    float TrajectoryTracker::getTau(size_t i) {
        return static_cast<float>(
            state_interfaces_[stateIdx_[i].eff].get_optional<double>().value());
    }
    float TrajectoryTracker::getTemp(size_t i) {
        return static_cast<float>(
            state_interfaces_[stateIdx_[i].tmp].get_optional<double>().value());
    };

    // =========================================================================
    //  监控发布
    // =========================================================================
    void TrajectoryTracker::registerMonitorPublisher() {
        monitorPub_ = this->get_node()->create_publisher<dog_msg::msg::SystemState>(
            monitorTopic_, 10);
        RCLCPP_INFO(logger_,
                    "Monitor publisher registered on %s (1/%zu rate)",
                    monitorTopic_.c_str(),
                    kMonitorPubDecimation);
    }

    void TrajectoryTracker::publishMonitorState() {
        const SystemState &s = monitor_.getState();

        dog_msg::msg::SystemState msg;
        msg.cpu_usage = s.cpu_usage;
        msg.cpu_temperature = s.cpu_temperature;
        msg.control_frequency = s.control_frequency;
        msg.timestamp = this->get_node()->now();

        for (size_t i = 0; i < 8; ++i) {
            msg.motors[i].name = s.motors[i].name;
            msg.motors[i].position = s.motors[i].position;
            msg.motors[i].velocity = s.motors[i].velocity;
            msg.motors[i].torque = s.motors[i].torque;
            msg.motors[i].temperature = s.motors[i].temperature;
        }

        monitorPub_->publish(msg);
    }

} // namespace Dog8Dof

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(Dog8Dof::TrajectoryTracker,
                       controller_interface::ControllerInterface)
