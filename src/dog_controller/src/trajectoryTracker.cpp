/**
 * @file trajectoryTracker.cpp
 * @brief ROS2 Controller — 在线重规划步态追踪
 *
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
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    TrajectoryTracker::on_configure(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;

        if (standProfile_.empty() || walkProfile_.empty()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "standProfile and walkProfile must not be empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        const auto clock = this->get_node()->get_clock();
        const auto period = rclcpp::Duration::from_seconds(updateDuration_);

        try {
            LF_ = std::make_unique<GaitManager>(
                clock, period, walkProfile_, standProfile_, jumpProfile_);
            LR_ = std::make_unique<GaitManager>(
                clock, period, walkProfile_, standProfile_, jumpProfile_);
            RF_ = std::make_unique<GaitManager>(
                clock, period, walkProfile_, standProfile_, jumpProfile_);
            RR_ = std::make_unique<GaitManager>(
                clock, period, walkProfile_, standProfile_, jumpProfile_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "TrajectoryTracker configure failed: %s",
                         e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        converter_ = GaitTwistConverter();

        commandSub_ = this->get_node()->create_subscription<dog_msg::msg::GaitTwist>(
            commandTopic_, 1, [this](const dog_msg::msg::GaitTwist::SharedPtr msg) {
                if (commandBox_.try_set(*msg)) {
                    newCmd_ = true;
                }
            });

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

        switchToStandMode();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // =========================================================================
    //  on_error — 安全回落
    // =========================================================================
    controller_interface::CallbackReturn
    TrajectoryTracker::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCLCPP_ERROR(this->get_node()->get_logger(),
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
            return controller_interface::return_type::ERROR;
        }

        // 3. 逆运动学结果下发到关节命令接口
        if (!setAllJointCommands()) {
            return controller_interface::return_type::ERROR;
        }

        return controller_interface::return_type::OK;
    }

    // =========================================================================
    //  applyCommand() — GaitTwist → GaitManager 转译
    // =========================================================================
    bool TrajectoryTracker::applyCommand(const dog_msg::msg::GaitTwist &msg) {
        using GT = dog_msg::msg::GaitTwist;

        GaitParameters params[4];
        if (!converter_.convert(msg, params)) {
            RCLCPP_WARN(this->get_node()->get_logger(),
                        "GaitTwist conversion failed, falling back to stand");
            return switchToStandMode();
        }

        switch (msg.gait) {
        case GT::STAND: {
            bool finishLF = LF_->isFinished();
            bool finishLR = LR_->isFinished();
            bool finishRF = RF_->isFinished();
            bool finishRR = RR_->isFinished();
            bool allFinished = finishLF && finishLR && finishRF && finishRR;
            if (currentMode_ == ControlMode::STAND && allFinished) {
                LF_->setStandGait(0.0f, params[0].baseHeight);
                LR_->setStandGait(0.0f, params[1].baseHeight);
                RF_->setStandGait(0.0f, params[2].baseHeight);
                RR_->setStandGait(0.0f, params[3].baseHeight);
                currentMode_ = ControlMode::STAND;
                break;
            } else {
                break;
            }
            // 仅在当前已是站立模式且所有腿都完成当前动作时，才更新站立步态参数，否则保持现有步态继续执行（避免频繁切换导致不稳定）
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
            RCLCPP_WARN(this->get_node()->get_logger(), "Jump not implemented, skip");
            break; // 尚未实现跳跃步态，跳过
            LF_->setJumpGait(params[0].stepLength, params[0].stepHeight);
            LR_->setJumpGait(params[1].stepLength, params[1].stepHeight);
            RF_->setJumpGait(params[2].stepLength, params[2].stepHeight);
            RR_->setJumpGait(params[3].stepLength, params[3].stepHeight);
            currentMode_ = ControlMode::JUMP;
            break;
        }
        default:
            RCLCPP_WARN(this->get_node()->get_logger(), "Unknown gait mode");
            return false;
        }
        return true;
    }

    // =========================================================================
    //  switchToStandMode() — 安全回落站立
    // =========================================================================
    bool TrajectoryTracker::switchToStandMode() {
        LF_->setStandGait(idleTipX_, idleTipZ_);
        LR_->setStandGait(idleTipX_, idleTipZ_);
        RF_->setStandGait(idleTipX_, idleTipZ_);
        RR_->setStandGait(idleTipX_, idleTipZ_);
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
            const auto cmdPos = cmdMap.find(jointNames_[i] + "/position");
            const auto cmdVel = cmdMap.find(jointNames_[i] + "/velocity");
            const auto cmdEff = cmdMap.find(jointNames_[i] + "/effort");
            const auto cmdKp = cmdMap.find(jointNames_[i] + "/kp");
            const auto cmdKd = cmdMap.find(jointNames_[i] + "/kd");

            if (stPos == stMap.end() || stVel == stMap.end() || stEff == stMap.end() ||
                cmdPos == cmdMap.end() || cmdVel == cmdMap.end() ||
                cmdEff == cmdMap.end() || cmdKp == cmdMap.end() ||
                cmdKd == cmdMap.end()) {
                RCLCPP_ERROR(this->get_node()->get_logger(),
                             "Missing interface for %s",
                             jointNames_[i].c_str());
                return false;
            }
            stateIdx_[i] = {stPos->second, stVel->second, stEff->second};
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

} // namespace Dog8Dof

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(Dog8Dof::TrajectoryTracker,
                       controller_interface::ControllerInterface)
