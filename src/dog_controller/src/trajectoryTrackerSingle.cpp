#include "dog_controller/trajectoryTrackerSingle.hpp"

#include <cmath>

namespace Dog8Dof {
    controller_interface::CallbackReturn TrajectoryTrackerSingle::on_init() {
        robotName_ = auto_declare<std::string>("robotName", "dog");
        standProfile_ = auto_declare<std::string>("standProfile", "");
        walkProfile_ = auto_declare<std::string>("walkProfile", "");
        jumpProfile_ = auto_declare<std::string>("jumpProfile", "");

        updateDuration_ = 1.0f / this->get_update_rate();
        idleTipX_ = auto_declare<float>("idleTipX", 0.0f);
        idleTipZ_ = auto_declare<float>("idleTipZ", 300.0f);
        commandTopic_ = auto_declare<std::string>("commandTopic", "/dog/command");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    TrajectoryTrackerSingle::on_configure(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        try {
            if (standProfile_.empty() || walkProfile_.empty() || jumpProfile_.empty()) {
                RCLCPP_ERROR(this->get_node()->get_logger(),
                             "standProfile, walkProfile, jumpProfile must not be empty");
                return controller_interface::CallbackReturn::ERROR;
            }

            const auto clock = this->get_node()->get_clock();
            const auto period = rclcpp::Duration::from_seconds(updateDuration_);

            // 为兼容保留四个管理器成员，但单腿逻辑只使用 LF_
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
                         "TrajectoryTrackerSingle configure failed: %s",
                         e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        commandSub_ =
            this->get_node()->create_subscription<dog_msg::msg::TrajectoryCommand>(
                commandTopic_,
                1,
                [this](const dog_msg::msg::TrajectoryCommand::SharedPtr msg) {
                    if (commandBox_.try_set(*msg)) {
                        newCmd_ = true;
                    }
                });

        newCmd_ = false;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    TrajectoryTrackerSingle::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(2 * 5);
        for (size_t i = 0; i < 2; i++) {
            config.names.push_back(jointNames_[i] + "/position");
            config.names.push_back(jointNames_[i] + "/velocity");
            config.names.push_back(jointNames_[i] + "/effort");
            config.names.push_back(jointNames_[i] + "/kp");
            config.names.push_back(jointNames_[i] + "/kd");
        }
        return config;
    }

    controller_interface::InterfaceConfiguration
    TrajectoryTrackerSingle::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(2 * 3);
        for (size_t i = 0; i < 2; i++) {
            config.names.push_back(jointNames_[i] + "/position");
            config.names.push_back(jointNames_[i] + "/velocity");
            config.names.push_back(jointNames_[i] + "/effort");
        }
        return config;
    }

    controller_interface::CallbackReturn
    TrajectoryTrackerSingle::on_activate(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        if (!cacheInterfaceIndices()) {
            RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to cache interfaces");
            return controller_interface::CallbackReturn::ERROR;
        }

        const bool lfOk = LF_->init(Eigen::Vector2f(getPos(0), getPos(1)),
                                    Eigen::Vector2f(getVel(0), getVel(1)),
                                    Eigen::Vector2f(getTau(0), getTau(1)));

        if (!lfOk) {
            RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to init motor");
            return controller_interface::CallbackReturn::ERROR;
        }

        LF_->setStandGait(idleTipX_, idleTipZ_);
        currentMode_ = ControlMode::STAND;

        const auto syncStart = this->get_node()->get_clock()->now();
        LF_->start(syncStart);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    TrajectoryTrackerSingle::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCLCPP_ERROR(this->get_node()->get_logger(),
                     "TrajectoryTrackerSingle entered error state from %s",
                     previous_state.label().c_str());

        if (!LF_) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Gait manager LF_ not initialized in on_error");
            return controller_interface::CallbackReturn::FAILURE;
        }

        while (commandBox_.try_get().has_value()) {
        }
        newCmd_ = false;

        if (command_interfaces_.empty() || state_interfaces_.empty()) {
            RCLCPP_WARN(
                this->get_node()->get_logger(),
                "Command/state interfaces appear not cached; attempting to cache");
            if (!cacheInterfaceIndices()) {
                RCLCPP_WARN(
                    this->get_node()->get_logger(),
                    "Caching interfaces failed in on_error; will not write commands");
                LF_->setStandGait(idleTipX_, idleTipZ_);
                return controller_interface::CallbackReturn::FAILURE;
            }
        }

        if (!updateAllGaitManagers()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Failed to update gait managers");
            return controller_interface::CallbackReturn::ERROR;
        }

        LF_->setStandGait(idleTipX_, idleTipZ_);
        currentMode_ = ControlMode::STAND;

        if (!setAllJointCommands()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Failed to set initial joint commands in error state");
            return controller_interface::CallbackReturn::FAILURE;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type
    TrajectoryTrackerSingle::update(const rclcpp::Time &time,
                                    const rclcpp::Duration &period) {
        (void)time;
        (void)period;

        if (!updateAllGaitManagers()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Failed to update gait managers");
            return controller_interface::return_type::ERROR;
        }
        // 首先更新模型到最新状态

        if (newCmd_) {
            auto msg_opt = commandBox_.try_get();
            if (!msg_opt.has_value()) {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Failed to read command, continue current command");
                newCmd_ = false;
                goto updateCommandFinished;
            }

            dog_msg::msg::TrajectoryCommand::_gait_type gait = msg_opt.value().gait;

            std::array<std::array<float, 4>, 3> param;

            for (size_t i = 0; i < 4; i++) {
                param[0][i] = msg_opt.value().par1[i];
                param[1][i] = msg_opt.value().par2[i];
                param[2][i] = msg_opt.value().par3[i];
            }

            try {
                switch (gait) {
                case dog_msg::msg::TrajectoryCommand::STAND:
                    // 仅对 LF_ 应用参数
                    LF_->setStandGait(param[0][0], param[1][0]);
                    if (currentMode_ != ControlMode::STAND) {
                        RCLCPP_INFO_STREAM(this->get_node()->get_logger(),
                                           "Switched to stand");
                    }
                    currentMode_ = ControlMode::STAND;
                    break;
                case dog_msg::msg::TrajectoryCommand::WALK:
                    if (!LF_->isFinshed()) {
                        if (currentMode_ == ControlMode::WALK &&
                            isSameWalkCommand(param)) {
                            newCmd_ = false;
                            goto updateCommandFinished;
                        }
                    }

                    LF_->setWalkGait(param[0][0],
                                     param[1][0],
                                     param[2][0],
                                     kWalkPhaseOffset[0],
                                     currentMode_ == ControlMode::WALK);
                    if (currentMode_ != ControlMode::WALK) {
                        RCLCPP_INFO(this->get_node()->get_logger(), "Switched to walk");
                    }
                    currentMode_ = ControlMode::WALK;
                    lastWalkParams_ = param;
                    hasLastWalkParams_ = true;
                    break;
                case dog_msg::msg::TrajectoryCommand::JUMP:
                    LF_->setJumpGait(param[0][0]);
                    if (currentMode_ != ControlMode::JUMP) {
                        RCLCPP_INFO(this->get_node()->get_logger(), "Switched to jump");
                    }
                    currentMode_ = ControlMode::JUMP;
                    break;
                default:
                    RCLCPP_INFO(this->get_node()->get_logger(),
                                "Unknown gait command received, ignore");
                    newCmd_ = false;
                    goto updateCommandFinished;
                }

                const auto syncStart = this->get_node()->get_clock()->now();
                LF_->start(syncStart);

            } catch (std::exception &e) {
                RCLCPP_ERROR(this->get_node()->get_logger(),
                             "Failed to set gait parameters: %s",
                             e.what());
                newCmd_ = false;
                goto updateCommandFinished;
            }

            newCmd_ = false;
        }
        // 如果存在新命令，先处理命令切换轨迹

    updateCommandFinished:
        if (LF_->isFinshed() && currentMode_ != ControlMode::STAND) {
            LF_->setStandGait(idleTipX_, idleTipZ_);
            currentMode_ = ControlMode::STAND;

            const rclcpp::Time syncStart = this->get_node()->get_clock()->now();
            LF_->start(syncStart);

            if (!updateAllGaitManagers()) {
                RCLCPP_ERROR(this->get_node()->get_logger(),
                             "Failed to update stand gait managers");
                return controller_interface::return_type::ERROR;
            } else {
                if (currentMode_ != ControlMode::STAND) {
                    RCLCPP_INFO(this->get_node()->get_logger(),
                                "Current command finished, switch to stand");
                }
            }
        }
        // 若当前命令完成，切换到站立

        if (!setAllJointCommands()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Failed to set current joint commands");
            return controller_interface::return_type::ERROR;
        }
        // 最后更新命令到接口

        return controller_interface::return_type::OK;
    }

    bool TrajectoryTrackerSingle::cacheInterfaceIndices() {
        std::unordered_map<std::string, size_t> stMap;
        std::unordered_map<std::string, size_t> cmdMap;

        stMap.reserve(state_interfaces_.size());
        for (size_t i = 0; i < state_interfaces_.size(); ++i) {
            stMap[state_interfaces_[i].get_name()] = i;
        }

        cmdMap.reserve(command_interfaces_.size());
        for (size_t i = 0; i < command_interfaces_.size(); ++i) {
            cmdMap[command_interfaces_[i].get_name()] = i;
        }

        for (size_t i = 0; i < 2; ++i) {
            const std::string base = jointNames_[i];

            const auto stPos = stMap.find(base + "/position");
            const auto stVel = stMap.find(base + "/velocity");
            const auto stEff = stMap.find(base + "/effort");

            const auto cmdPos = cmdMap.find(base + "/position");
            const auto cmdVel = cmdMap.find(base + "/velocity");
            const auto cmdEff = cmdMap.find(base + "/effort");
            const auto cmdKp = cmdMap.find(base + "/kp");
            const auto cmdKd = cmdMap.find(base + "/kd");

            if (stPos == stMap.end() || stVel == stMap.end() || stEff == stMap.end() ||
                cmdPos == cmdMap.end() || cmdVel == cmdMap.end() ||
                cmdEff == cmdMap.end() || cmdKp == cmdMap.end() ||
                cmdKd == cmdMap.end()) {
                RCLCPP_ERROR(this->get_node()->get_logger(),
                             "Missing interface for joint %s",
                             base.c_str());
                return false;
            }

            stateIdx_[i] = JointStateIndex{stPos->second, stVel->second, stEff->second};
            cmdIdx_[i] = JointCmdIndex{cmdPos->second,
                                       cmdVel->second,
                                       cmdEff->second,
                                       cmdKp->second,
                                       cmdKd->second};
        }
        return true;
    }

    bool TrajectoryTrackerSingle::setJointCommands(size_t legIndex, const Motor &target) {
        const JointCmdIndex &idx = cmdIdx_[legIndex];

        command_interfaces_[idx.pos].set_value(target.pos);
        command_interfaces_[idx.vel].set_value(target.vel);
        command_interfaces_[idx.eff].set_value(target.tau);
        command_interfaces_[idx.kp].set_value(target.kp);
        command_interfaces_[idx.kd].set_value(target.kd);
        return true;
    }

    bool TrajectoryTrackerSingle::setAllJointCommands() {
        if (!LF_)
            return false;
        if (!setJointCommands(0, LF_->getFrontTarget()))
            return false;
        if (!setJointCommands(1, LF_->getRearTarget()))
            return false;
        return true;
    }

    bool TrajectoryTrackerSingle::updateAllGaitManagers() {
        if (!LF_) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "LF_ gait manager not initialized");
            return false;
        }

        const bool lfOk = LF_->update(Eigen::Vector2f(getPos(0), getPos(1)),
                                      Eigen::Vector2f(getVel(0), getVel(1)),
                                      Eigen::Vector2f(getTau(0), getTau(1)));

        return lfOk;
    }

    bool TrajectoryTrackerSingle::isSameWalkCommand(
        const std::array<std::array<float, 4>, 3> &walkParams) const {
        if (!hasLastWalkParams_) {
            return false;
        }

        constexpr float kWalkParamEps = 1e-6f;
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 4; ++j) {
                if (std::fabs(walkParams[i][j] - lastWalkParams_[i][j]) > kWalkParamEps) {
                    return false;
                }
            }
        }
        return true;
    }

    float TrajectoryTrackerSingle::getPos(size_t i) {
        return state_interfaces_[stateIdx_[i].pos].get_value();
        return 0.0f;
    }

    float TrajectoryTrackerSingle::getVel(size_t i) {
        return state_interfaces_[stateIdx_[i].vel].get_value();
        return 0.0f;
    };

    float TrajectoryTrackerSingle::getTau(size_t i) {
        return state_interfaces_[stateIdx_[i].eff].get_value();
    };

} // namespace Dog8Dof

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(Dog8Dof::TrajectoryTrackerSingle,
                       controller_interface::ControllerInterface)