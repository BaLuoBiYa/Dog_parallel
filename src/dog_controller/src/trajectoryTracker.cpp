#include "dog_controller/trajectoryTracker.hpp"

namespace Dog8Dof {
    controller_interface::CallbackReturn TrajectoryTracker::on_init() {
        // Initialize parameters
        robotName_ = auto_declare<std::string>("robotName", "dog");
        standProfile_ = auto_declare<std::string>("standProfile", "");
        walkProfile_ = auto_declare<std::string>("walkProfile", "");
        jumpProfile_ = auto_declare<std::string>("jumpProfile", "");

        updateDuration_ = 1.0f / (this->get_update_rate() *
                                  10); // 以更高频率更新步态管理器，确保平滑过渡

        idleTipX_ = auto_declare<float>("idleTipX", 0.0f);
        idleTipZ_ = auto_declare<float>("idleTipZ", 180.0f);
        commandTopic_ = auto_declare<std::string>("commandTopic", "/dog/command");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    TrajectoryTracker::on_configure(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;

        try {
            if (standProfile_.empty() || walkProfile_.empty() || jumpProfile_.empty()) {
                RCLCPP_ERROR(this->get_node()->get_logger(),
                             "standProfile, walkProfile, jumpProfile must not be empty");
                return controller_interface::CallbackReturn::ERROR;
            }

            const auto clock = this->get_node()->get_clock();
            const auto period = rclcpp::Duration::from_seconds(updateDuration_);

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
        // 设置步态管理器

        commandSub_ =
            this->get_node()->create_subscription<dog_msg::msg::TrajectoryCommand>(
                commandTopic_,
                1,
                [this](const dog_msg::msg::TrajectoryCommand::SharedPtr msg) {
                    if (commandBox_.try_set(*msg)) {
                        newCmd_ = true;
                    }
                });
        // 设置控制指令订阅
        newCmd_ = false;

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    TrajectoryTracker::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;

        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(8 * 5);
        for (size_t i = 0; i < 8; i++) {
            config.names.push_back(jointNames_[i] + "/position");
            config.names.push_back(jointNames_[i] + "/velocity");
            config.names.push_back(jointNames_[i] + "/effort");
            config.names.push_back(jointNames_[i] + "/kp");
            config.names.push_back(jointNames_[i] + "/kd");
        }
        // 申请关节命令接口
        return config;
    }

    controller_interface::InterfaceConfiguration
    TrajectoryTracker::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;

        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(8 * 3);

        for (size_t i = 0; i < 8; i++) {
            config.names.push_back(jointNames_[i] + "/position");
            config.names.push_back(jointNames_[i] + "/velocity");
            config.names.push_back(jointNames_[i] + "/effort");
        }

        return config;
    }

    controller_interface::CallbackReturn
    TrajectoryTracker::on_activate(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;
        if (!cacheInterfaceIndices()) {
            RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to cache interfaces");
            return controller_interface::CallbackReturn::ERROR;
        }
        // 缓存硬件命令/状态接口索引

        bool ok;
        ok = LF_->init(Eigen::Vector2f(getPos(0), getPos(1)),
                       Eigen::Vector2f(getVel(0), getVel(1)),
                       Eigen::Vector2f(getTau(0), getTau(1)));
        if (!ok) {
            RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to init LF motor");
            return controller_interface::CallbackReturn::ERROR;
        }
        // 初始化LF

        ok = RF_->init(Eigen::Vector2f(getPos(2), getPos(3)),
                       Eigen::Vector2f(getVel(2), getVel(3)),
                       Eigen::Vector2f(getTau(2), getTau(3)));
        if (!ok) {
            RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to init RF motor");
            return controller_interface::CallbackReturn::ERROR;
        }
        // 初始化RF

        ok = LR_->init(Eigen::Vector2f(getPos(4), getPos(5)),
                       Eigen::Vector2f(getVel(4), getVel(5)),
                       Eigen::Vector2f(getTau(4), getTau(5)));

        if (!ok) {
            RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to init LR motor");
            return controller_interface::CallbackReturn::ERROR;
        }
        // 初始化LR

        ok = RR_->init(Eigen::Vector2f(getPos(6), getPos(7)),
                       Eigen::Vector2f(getVel(6), getVel(7)),
                       Eigen::Vector2f(getTau(6), getTau(7)));
        if (!ok) {
            RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to init RR motor");
            return controller_interface::CallbackReturn::ERROR;
        }
        // 初始化RR

        if (!LF_->setStandGait(idleTipX_, idleTipZ_)) {
            RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to init LF");
            return controller_interface::CallbackReturn::ERROR;
        }
        if (!LR_->setStandGait(idleTipX_, idleTipZ_)) {
            RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to init LR");
            return controller_interface::CallbackReturn::ERROR;
        }
        if (!RF_->setStandGait(idleTipX_, idleTipZ_)) {
            RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to init RF");
            return controller_interface::CallbackReturn::ERROR;
        }
        if (!RR_->setStandGait(idleTipX_, idleTipZ_)) {
            RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to init RR");
            return controller_interface::CallbackReturn::ERROR;
        }
        currentMode_ = ControlMode::STAND;
        initialStandPending_ = true;

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    TrajectoryTracker::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCLCPP_ERROR(this->get_node()->get_logger(),
                     "TrajectoryTracker entered error state from %s",
                     previous_state.label().c_str());

        // 防御性检查：步态管理器必须存在
        if (!LF_ || !LR_ || !RF_ || !RR_) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Gait managers not initialized in on_error");
            return controller_interface::CallbackReturn::FAILURE;
        }

        // 清空实时命令盒子并重置标志（在非实时线程安全）
        while (commandBox_.try_get().has_value()) {
            // 丢弃所有残留命令
        }
        newCmd_ = false;

        // 确保已缓存命令/状态接口；如果未缓存，尝试缓存一次
        if (command_interfaces_.empty() || state_interfaces_.empty()) {
            RCLCPP_WARN(
                this->get_node()->get_logger(),
                "Command/state interfaces appear not cached; attempting to cache");
            if (!cacheInterfaceIndices()) {
                RCLCPP_WARN(
                    this->get_node()->get_logger(),
                    "Caching interfaces failed in on_error; will not write commands");
                // 仍然设置步态管理器为安全站立，但无法下发到硬件
                LF_->setStandGait(idleTipX_, idleTipZ_);
                LR_->setStandGait(idleTipX_, idleTipZ_);
                RF_->setStandGait(idleTipX_, idleTipZ_);
                RR_->setStandGait(idleTipX_, idleTipZ_);
                return controller_interface::CallbackReturn::FAILURE;
            }
        }

        if (!updateAllGaitManagers()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Failed to update gait managers");
            return controller_interface::CallbackReturn::ERROR;
        }
        // 更新步态管理器状态

        // 设置为初始站立姿态
        LF_->setStandGait(idleTipX_, idleTipZ_);
        LR_->setStandGait(idleTipX_, idleTipZ_);
        RF_->setStandGait(idleTipX_, idleTipZ_);
        RR_->setStandGait(idleTipX_, idleTipZ_);
        currentMode_ = ControlMode::STAND;

        // 下发命令到硬件
        if (!setAllJointCommands()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Failed to set initial joint commands in error state");
            return controller_interface::CallbackReturn::FAILURE;
        }

        // 成功将控制器置于安全状态
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type
    TrajectoryTracker::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        (void)time;
        (void)period;

        if (initialStandPending_) {
            try {
                LF_->setStandGait(idleTipX_, idleTipZ_);
                LR_->setStandGait(idleTipX_, idleTipZ_);
                RF_->setStandGait(idleTipX_, idleTipZ_);
                RR_->setStandGait(idleTipX_, idleTipZ_);

                const auto syncStart = this->get_node()->get_clock()->now();
                LF_->start(syncStart);
                LR_->start(syncStart);
                RF_->start(syncStart);
                RR_->start(syncStart);
                initialStandPending_ = false;
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_node()->get_logger(),
                             "Failed to initialize stand trajectory: %s",
                             e.what());
                return controller_interface::return_type::ERROR;
            }
        }

        if (!updateAllGaitManagers()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Failed to update gait managers");
            return controller_interface::return_type::ERROR;
        }
        // 更新步态管理器状态

        if (!setAllJointCommands()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Failed to set currrent joint commands");
            return controller_interface::return_type::ERROR;
        } // 应用当前目标

        // —— 步骤1：收到新消息时，仅缓存，不立即应用 ——
        if (newCmd_) {
            auto msg_opt = commandBox_.try_get();
            if (!msg_opt.has_value()) {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Failed to read command, continue current command");
                newCmd_ = false;
            } else {
                const auto gait = msg_opt.value().gait;

                std::array<std::array<float, 4>, 4> param;
                for (size_t i = 0; i < 4; i++) {
                    param[0][i] = msg_opt.value().par1[i];
                    param[1][i] = msg_opt.value().par2[i];
                    param[2][i] = msg_opt.value().par3[i];
                    param[3][i] = msg_opt.value().par4[i];
                }

                switch (gait) {
                case dog_msg::msg::TrajectoryCommand::STAND:
                    pendingMode_ = ControlMode::STAND;
                    break;
                case dog_msg::msg::TrajectoryCommand::WALK:
                    pendingMode_ = ControlMode::WALK;
                    break;
                case dog_msg::msg::TrajectoryCommand::JUMP:
                    pendingMode_ = ControlMode::JUMP;
                    break;
                default:
                    RCLCPP_INFO(this->get_node()->get_logger(),
                                "Unknown gait command received, ignore");
                    newCmd_ = false;
                    goto checkFinished;
                }

                pendingParams_ = param;
                pendingCmd_ = true;
                newCmd_ = false;

                RCLCPP_DEBUG(this->get_node()->get_logger(),
                             "Command cached, will apply after current gait finishes");
            }
        }

    checkFinished:
        // —— 步骤2：检查当前步态是否已完成 ——
        const bool lfFinished = LF_->isFinshed();
        const bool lrFinished = LR_->isFinshed();
        const bool rfFinished = RF_->isFinshed();
        const bool rrFinished = RR_->isFinshed();
        const bool allFinished = lfFinished && lrFinished && rfFinished && rrFinished;

        // —— 步骤3：当前步态完成后，应用缓存的新命令 ——
        if (allFinished && pendingCmd_) {
            try {
                switch (pendingMode_) {
                case ControlMode::STAND:
                    LF_->setStandGait(pendingParams_[0][0], pendingParams_[1][0]);
                    LR_->setStandGait(pendingParams_[0][1], pendingParams_[1][1]);
                    RF_->setStandGait(pendingParams_[0][2], pendingParams_[1][2]);
                    RR_->setStandGait(pendingParams_[0][3], pendingParams_[1][3]);
                    RCLCPP_INFO(this->get_node()->get_logger(), "Switched to stand");
                    break;

                case ControlMode::WALK:
                    LF_->setWalkGait(pendingParams_[0][0],
                                     pendingParams_[1][0],
                                     pendingParams_[2][0],
                                     pendingParams_[3][0],
                                     kWalkPhaseOffset[0]);
                    LR_->setWalkGait(pendingParams_[0][1],
                                     pendingParams_[1][1],
                                     pendingParams_[2][1],
                                     pendingParams_[3][1],
                                     kWalkPhaseOffset[1]);
                    RF_->setWalkGait(pendingParams_[0][2],
                                     pendingParams_[1][2],
                                     pendingParams_[2][2],
                                     pendingParams_[3][2],
                                     kWalkPhaseOffset[2]);
                    RR_->setWalkGait(pendingParams_[0][3],
                                     pendingParams_[1][3],
                                     pendingParams_[2][3],
                                     pendingParams_[3][3],
                                     kWalkPhaseOffset[3]);
                    // 同时保存为步行循环参数
                    lastWalkParams_ = pendingParams_;
                    hasLastWalkParams_ = true;
                    RCLCPP_INFO(this->get_node()->get_logger(), "Switched to walk");
                    break;

                case ControlMode::JUMP:
                    LF_->setJumpGait(pendingParams_[0][0]);
                    LR_->setJumpGait(pendingParams_[0][1]);
                    RF_->setJumpGait(pendingParams_[0][2]);
                    RR_->setJumpGait(pendingParams_[0][3]);
                    RCLCPP_INFO(this->get_node()->get_logger(), "Switched to jump");
                    break;
                }

                const auto syncStart = this->get_node()->get_clock()->now();
                LF_->start(syncStart);
                LR_->start(syncStart);
                RF_->start(syncStart);
                RR_->start(syncStart);

                currentMode_ = pendingMode_;
                pendingCmd_ = false;

            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_node()->get_logger(),
                             "Failed to apply pending gait: %s, fallback to stand",
                             e.what());
                pendingCmd_ = false;
                if (!switchToStandMode()) {
                    return controller_interface::return_type::ERROR;
                }
            }
        }

        // —— 步骤4：步行模式周期循环（无新命令时自动重启） ——
        if (!pendingCmd_ && currentMode_ == ControlMode::WALK && allFinished) {
            if (!hasLastWalkParams_) {
                RCLCPP_WARN(this->get_node()->get_logger(),
                            "Walk finished but no last walk params, fallback to stand");
                if (!switchToStandMode()) {
                    return controller_interface::return_type::ERROR;
                }
            } else {
                LF_->setWalkGait(lastWalkParams_[0][0],
                                 lastWalkParams_[1][0],
                                 lastWalkParams_[2][0],
                                 kWalkPhaseOffset[0]);
                LR_->setWalkGait(lastWalkParams_[0][1],
                                 lastWalkParams_[1][1],
                                 lastWalkParams_[2][1],
                                 kWalkPhaseOffset[1]);
                RF_->setWalkGait(lastWalkParams_[0][2],
                                 lastWalkParams_[1][2],
                                 lastWalkParams_[2][2],
                                 kWalkPhaseOffset[2]);
                RR_->setWalkGait(lastWalkParams_[0][3],
                                 lastWalkParams_[1][3],
                                 lastWalkParams_[2][3],
                                 kWalkPhaseOffset[3]);

                const auto syncStart = this->get_node()->get_clock()->now();
                LF_->start(syncStart);
                LR_->start(syncStart);
                RF_->start(syncStart);
                RR_->start(syncStart);

                RCLCPP_DEBUG(this->get_node()->get_logger(),
                             "Walk cycle finished, restarting next cycle");
            }
        }

        return controller_interface::return_type::OK;
    }

    bool TrajectoryTracker::cacheInterfaceIndices() {
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

        for (size_t i = 0; i < 8; ++i) {
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

    bool TrajectoryTracker::switchToStandMode() {
        LF_->setStandGait(idleTipX_, idleTipZ_);
        LR_->setStandGait(idleTipX_, idleTipZ_);
        RF_->setStandGait(idleTipX_, idleTipZ_);
        RR_->setStandGait(idleTipX_, idleTipZ_);
        currentMode_ = ControlMode::STAND;

        const rclcpp::Time syncStart = this->get_node()->get_clock()->now();
        LF_->start(syncStart);
        LR_->start(syncStart);
        RF_->start(syncStart);
        RR_->start(syncStart);

        if (!updateAllGaitManagers()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Failed to update stand gait managers");
            return false;
        }
        return true;
    }

    bool TrajectoryTracker::setJointCommands(size_t legIndex, const Motor &target) {
        const JointCmdIndex &idx = cmdIdx_[legIndex];
        rclcpp::Logger logger = this->get_node()->get_logger();

        if (!command_interfaces_[idx.pos].set_value(static_cast<double>(target.pos))) {
            RCLCPP_ERROR(
                logger, "Failed to set %s/position", jointNames_[legIndex].c_str());
            return false;
        }
        if (!command_interfaces_[idx.vel].set_value(static_cast<double>(target.vel))) {
            RCLCPP_ERROR(
                logger, "Failed to set %s/velocity", jointNames_[legIndex].c_str());
            return false;
        }
        if (!command_interfaces_[idx.eff].set_value(static_cast<double>(target.tau))) {
            RCLCPP_ERROR(
                logger, "Failed to set %s/effort", jointNames_[legIndex].c_str());
            return false;
        }
        if (!command_interfaces_[idx.kp].set_value(static_cast<double>(target.kp))) {
            RCLCPP_ERROR(logger, "Failed to set %s/kp", jointNames_[legIndex].c_str());
            return false;
        }
        if (!command_interfaces_[idx.kd].set_value(static_cast<double>(target.kd))) {
            RCLCPP_ERROR(logger, "Failed to set %s/kd", jointNames_[legIndex].c_str());
            return false;
        }
        return true;
    }

    bool TrajectoryTracker::setAllJointCommands() {
        if (!setJointCommands(0, LF_->getFrontTarget()))
            return false;
        if (!setJointCommands(1, LF_->getRearTarget()))
            return false;
        if (!setJointCommands(2, RF_->getFrontTarget()))
            return false;
        if (!setJointCommands(3, RF_->getRearTarget()))
            return false;
        if (!setJointCommands(4, LR_->getFrontTarget()))
            return false;
        if (!setJointCommands(5, LR_->getRearTarget()))
            return false;
        if (!setJointCommands(6, RR_->getFrontTarget()))
            return false;
        if (!setJointCommands(7, RR_->getRearTarget()))
            return false;
        return true;
    }

    bool TrajectoryTracker::updateAllGaitManagers() {
        const bool lfOk = LF_->update(Eigen::Vector2f(getPos(0), getPos(1)),
                                      Eigen::Vector2f(getVel(0), getVel(1)),
                                      Eigen::Vector2f(getTau(0), getTau(1)));

        const bool rfOk = RF_->update(Eigen::Vector2f(getPos(2), getPos(3)),
                                      Eigen::Vector2f(getVel(2), getVel(3)),
                                      Eigen::Vector2f(getTau(2), getTau(3)));

        const bool lrOk = LR_->update(Eigen::Vector2f(getPos(4), getPos(5)),
                                      Eigen::Vector2f(getVel(4), getVel(5)),
                                      Eigen::Vector2f(getTau(4), getTau(5)));

        const bool rrOk = RR_->update(Eigen::Vector2f(getPos(6), getPos(7)),
                                      Eigen::Vector2f(getVel(6), getVel(7)),
                                      Eigen::Vector2f(getTau(6), getTau(7)));

        return lfOk && rfOk && lrOk && rrOk;
    }

    bool TrajectoryTracker::isSameWalkCommand(
        const std::array<std::array<float, 4>, 3> &walkParams) const {
        if (!hasLastWalkParams_) {
            return false;
        }

        constexpr float kWalkParamEps = 1.0f;
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 4; ++j) {
                if (std::fabs(walkParams[i][j] - lastWalkParams_[i][j]) > kWalkParamEps) {
                    return false;
                }
            }
        }
        return true;
    }

    float TrajectoryTracker::getPos(size_t i) {
        auto opt = state_interfaces_[stateIdx_[i].pos].get_optional<double>();
        if (!opt.has_value()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Position state interface for joint %s is not available",
                         jointNames_[i].c_str());
            throw std::runtime_error("Missing position state interface");
        } else {
            return static_cast<float>(opt.value());
        }
    }

    float TrajectoryTracker::getVel(size_t i) {
        auto opt = state_interfaces_[stateIdx_[i].vel].get_optional<double>();
        if (!opt.has_value()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Velocity state interface for joint %s is not available",
                         jointNames_[i].c_str());
            throw std::runtime_error("Missing velocity state interface");
        } else {
            return static_cast<float>(opt.value());
        }
    };

    float TrajectoryTracker::getTau(size_t i) {
        auto opt = state_interfaces_[stateIdx_[i].eff].get_optional<double>();
        if (!opt.has_value()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Effort state interface for joint %s is not available",
                         jointNames_[i].c_str());
            throw std::runtime_error("Missing effort state interface");
        } else {
            return static_cast<float>(opt.value());
        }
    };

} // namespace Dog8Dof

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(Dog8Dof::TrajectoryTracker,
                       controller_interface::ControllerInterface)