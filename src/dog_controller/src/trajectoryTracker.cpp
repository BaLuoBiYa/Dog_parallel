#include "dog_controller/trajectoryTracker.hpp"

namespace Dog8Dof {
    controller_interface::CallbackReturn TrajectoryTracker::on_init() {
        // Initialize parameters
        robotName_ = auto_declare<std::string>("robotName", "dog");
        standProfile_ = auto_declare<std::string>("standProfile", "");
        walkProfile_ = auto_declare<std::string>("walkProfile", "");
        jumpProfile_ = auto_declare<std::string>("jumpProfile", "");

        generateDuration_ = auto_declare<float>("generateDuration", 0.001f);
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
                RCLCPP_ERROR(rclcpp::get_logger("TrajectoryTracker"),
                             "standProfile, walkProfile, jumpProfile must not be empty");
                return controller_interface::CallbackReturn::ERROR;
            }

            const auto clock = this->get_node()->get_clock();
            const auto period = rclcpp::Duration::from_seconds(generateDuration_);

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

        LF_->setStandGait(idleTipX_, idleTipZ_);
        LR_->setStandGait(idleTipX_, idleTipZ_);
        RF_->setStandGait(idleTipX_, idleTipZ_);
        RR_->setStandGait(idleTipX_, idleTipZ_);
        // 设置初始站立姿态

        if (!setAllJointCommands()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Failed to set initial joint commands");
            return controller_interface::CallbackReturn::ERROR;
        } // 应用初始站姿

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

        // 设置为初始站立姿态
        LF_->setStandGait(idleTipX_, idleTipZ_);
        LR_->setStandGait(idleTipX_, idleTipZ_);
        RF_->setStandGait(idleTipX_, idleTipZ_);
        RR_->setStandGait(idleTipX_, idleTipZ_);

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

        if (newCmd_) {
            auto msg_opt = commandBox_.try_get();
            if (msg_opt == std::nullopt) {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Failed to read command,continue current command");
                newCmd_ = false;
                goto updateCommandFinished;
            }

            auto gait = msg_opt.value().gait;
            auto param1 = msg_opt.value().par1.data();
            auto param2 = msg_opt.value().par2.data();
            auto param3 = msg_opt.value().par3.data();

            switch (gait) {
            case dog_msg::msg::TrajectoryCommand::STAND:
                LF_->setStandGait(param1[0], param2[0]);
                LR_->setStandGait(param1[1], param2[1]);
                RF_->setStandGait(param1[2], param2[2]);
                RR_->setStandGait(param1[3], param2[3]);
                break;
            case dog_msg::msg::TrajectoryCommand::WALK:
                LF_->setWalkGait(param1[0], param2[0], param3[0]);
                LR_->setWalkGait(param1[1], param2[1], param3[1]);
                RF_->setWalkGait(param1[2], param2[2], param3[2]);
                RR_->setWalkGait(param1[3], param2[3], param3[3]);
                break;
            case dog_msg::msg::TrajectoryCommand::JUMP:
                LF_->setJumpGait(param1[0]);
                LR_->setJumpGait(param1[1]);
                RF_->setJumpGait(param1[2]);
                RR_->setJumpGait(param1[3]);
                break;
            default:
                return controller_interface::return_type::ERROR;
            }
            newCmd_ = false;
        }

    updateCommandFinished:
        if (LF_->isFinshed() || LR_->isFinshed() || RF_->isFinshed() ||
            RR_->isFinshed()) {
            // 如果当前步态完成，切换回站立
            LF_->setStandGait(idleTipX_, idleTipZ_);
            LR_->setStandGait(idleTipX_, idleTipZ_);
            RF_->setStandGait(idleTipX_, idleTipZ_);
            RR_->setStandGait(idleTipX_, idleTipZ_);
        }

        if (!setAllJointCommands()) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "Failed to set currrent joint commands");
            return controller_interface::return_type::ERROR;
        } // 应用当前目标

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

    bool TrajectoryTracker::setJointCommands(size_t legIndex, const Motor &target) {
        const JointCmdIndex &idx = cmdIdx_[legIndex];
        rclcpp::Logger logger = this->get_node()->get_logger();

        if (!command_interfaces_[idx.pos].set_value(target.pos)) {
            RCLCPP_ERROR(
                logger, "Failed to set %s/position", jointNames_[legIndex].c_str());
            return false;
        }
        if (!command_interfaces_[idx.vel].set_value(target.vel)) {
            RCLCPP_ERROR(
                logger, "Failed to set %s/velocity", jointNames_[legIndex].c_str());
            return false;
        }
        if (!command_interfaces_[idx.eff].set_value(target.tau)) {
            RCLCPP_ERROR(
                logger, "Failed to set %s/effort", jointNames_[legIndex].c_str());
            return false;
        }
        if (!command_interfaces_[idx.kp].set_value(target.kp)) {
            RCLCPP_ERROR(logger, "Failed to set %s/kp", jointNames_[legIndex].c_str());
            return false;
        }
        if (!command_interfaces_[idx.kd].set_value(target.kd)) {
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

} // namespace Dog8Dof

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(Dog8Dof::TrajectoryTracker,
                       controller_interface::ControllerInterface)