/**
 * @file trajectoryTrackerSingle.cpp
 * @brief ROS2 Single-leg Controller — 在线重规划步态追踪（单腿版本）
 *
 * 与 TrajectoryTracker 对应，但仅管理一条腿（LF, 2 个关节）。
 */

#include "dog_controller/trajectoryTrackerSingle.hpp"

namespace Dog8Dof {

    // =========================================================================
    //  Lifecycle: on_init / on_configure / on_activate
    // =========================================================================
    controller_interface::CallbackReturn TrajectoryTrackerSingle::on_init() {
        robotName_ = auto_declare<std::string>("robotName", "dog");
        standProfile_ = auto_declare<std::string>("standProfile", "");
        walkProfile_ = auto_declare<std::string>("walkProfile", "");
        jumpProfile_ = auto_declare<std::string>("jumpProfile", "");
        updateDuration_ = 1.0f / static_cast<float>(this->get_update_rate());
        idleTipX_ = auto_declare<float>("idleTipX", 0.0f);
        idleTipZ_ = auto_declare<float>("idleTipZ", 270.0f);
        commandTopic_ = auto_declare<std::string>("commandTopic", "/dog/command");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    TrajectoryTrackerSingle::on_configure(const rclcpp_lifecycle::State &previous_state) {
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
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_node()->get_logger(),
                         "TrajectoryTrackerSingle configure failed: %s",
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
    TrajectoryTrackerSingle::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(2 * 5);
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
    TrajectoryTrackerSingle::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(2 * 3);
        for (auto &name : jointNames_) {
            config.names.push_back(std::string(name) + "/position");
            config.names.push_back(std::string(name) + "/velocity");
            config.names.push_back(std::string(name) + "/effort");
        }
        return config;
    }

    controller_interface::CallbackReturn
    TrajectoryTrackerSingle::on_activate(const rclcpp_lifecycle::State &previous_state) {
        (void)previous_state;

        if (!cacheInterfaceIndices()) {
            return controller_interface::CallbackReturn::ERROR;
        }

        if (!LF_->init(Eigen::Vector2f(getPos(0), getPos(1)),
                       Eigen::Vector2f(getVel(0), getVel(1)),
                       Eigen::Vector2f(getTau(0), getTau(1)))) {
            return controller_interface::CallbackReturn::ERROR;
        }

        switchToStandMode();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // =========================================================================
    //  on_error — 安全回落
    // =========================================================================
    controller_interface::CallbackReturn
    TrajectoryTrackerSingle::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCLCPP_ERROR(this->get_node()->get_logger(),
                     "TrajectoryTrackerSingle entered error state from %s",
                     previous_state.label().c_str());

        if (!LF_) {
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
    TrajectoryTrackerSingle::update(const rclcpp::Time &time,
                                    const rclcpp::Duration &period) {
        (void)time;
        (void)period;

        // 1. 处理新命令
        if (newCmd_) {
            auto msg_opt = commandBox_.try_get();
            if (msg_opt.has_value()) {
                applyCommand(msg_opt.value());
            }
            newCmd_ = false;
        }

        // 2. 更新步态管理器（传感器反馈闭环在线重规划）
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
    //  applyCommand() — GaitTwist → GaitManager 转译（单腿版本）
    // =========================================================================
    bool TrajectoryTrackerSingle::applyCommand(const dog_msg::msg::GaitTwist &msg) {
        using GT = dog_msg::msg::GaitTwist;

        GaitParameters params[4];
        if (!converter_.convert(msg, params)) {
            RCLCPP_WARN(this->get_node()->get_logger(),
                        "GaitTwist conversion failed, falling back to stand");
            return switchToStandMode();
        }

        // 单腿版本仅使用 params[0]（LF）
        switch (msg.gait) {
        case GT::STAND: {
            LF_->setStandGait(params[0].stepLength, params[0].stepHeight);
            currentMode_ = ControlMode::STAND;
            break;
        }
        case GT::WALK: {
            LF_->setWalkGait(params[0].stepLength,
                             params[0].stepHeight,
                             params[0].baseHeight,
                             params[0].stepDuration,
                             0.0f); // 单腿无相位偏移
            currentMode_ = ControlMode::WALK;
            break;
        }
        case GT::JUMP: {
            LF_->setJumpGait(params[0].stepLength, params[0].stepHeight);
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
    bool TrajectoryTrackerSingle::switchToStandMode() {
        LF_->setStandGait(idleTipX_, idleTipZ_);
        currentMode_ = ControlMode::STAND;
        return updateAllGaitManagers();
    }

    // =========================================================================
    //  updateAllGaitManagers() — 单腿更新
    // =========================================================================
    bool TrajectoryTrackerSingle::updateAllGaitManagers() {
        return LF_->update(Eigen::Vector2f(getPos(0), getPos(1)),
                           Eigen::Vector2f(getVel(0), getVel(1)),
                           Eigen::Vector2f(getTau(0), getTau(1)));
    }

    // =========================================================================
    //  Hardware interface helpers
    // =========================================================================
    bool TrajectoryTrackerSingle::cacheInterfaceIndices() {
        std::unordered_map<std::string, size_t> stMap;
        std::unordered_map<std::string, size_t> cmdMap;

        for (size_t i = 0; i < state_interfaces_.size(); ++i)
            stMap[state_interfaces_[i].get_name()] = i;
        for (size_t i = 0; i < command_interfaces_.size(); ++i)
            cmdMap[command_interfaces_[i].get_name()] = i;

        for (size_t i = 0; i < 2; ++i) {
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

    bool TrajectoryTrackerSingle::setJointCommands(size_t legIndex, const Motor &target) {
        const auto &idx = cmdIdx_[legIndex];
        return command_interfaces_[idx.pos].set_value(static_cast<double>(target.pos)) &&
               command_interfaces_[idx.vel].set_value(static_cast<double>(target.vel)) &&
               command_interfaces_[idx.eff].set_value(static_cast<double>(target.tau)) &&
               command_interfaces_[idx.kp].set_value(static_cast<double>(target.kp)) &&
               command_interfaces_[idx.kd].set_value(static_cast<double>(target.kd));
    }

    bool TrajectoryTrackerSingle::setAllJointCommands() {
        return setJointCommands(0, LF_->getFrontTarget()) &&
               setJointCommands(1, LF_->getRearTarget());
    }

    float TrajectoryTrackerSingle::getPos(size_t i) {
        return static_cast<float>(
            state_interfaces_[stateIdx_[i].pos].get_optional<double>().value());
    }
    float TrajectoryTrackerSingle::getVel(size_t i) {
        return static_cast<float>(
            state_interfaces_[stateIdx_[i].vel].get_optional<double>().value());
    }
    float TrajectoryTrackerSingle::getTau(size_t i) {
        return static_cast<float>(
            state_interfaces_[stateIdx_[i].eff].get_optional<double>().value());
    }

} // namespace Dog8Dof

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(Dog8Dof::TrajectoryTrackerSingle,
                       controller_interface::ControllerInterface)
