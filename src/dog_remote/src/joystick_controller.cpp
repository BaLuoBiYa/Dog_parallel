#include "dog_remote/joystick_controller.hpp"

#include <chrono>
#include <cmath>
#include <thread>

namespace Dog8Dof {

    JoyStickController::JoyStickController(const std::string &devicePath,
                                           const std::string &nodeName,
                                           const std::string &yamlPath,
                                           double pollRateHz)
        : rclcpp::Node(nodeName), devicePath_(devicePath), joystick_(devicePath, false),
          yamlPath_(yamlPath) {
        // ── 参数声明 ──
        declareParameters();

        // ── YAML 配置文件热加载 ──
        if (!yamlPath_.empty()) {
            loadYamlConfig();
            yamlLastMtime_ = file_mtime(yamlPath_);

            int watchMs = static_cast<int>(yamlWatchInterval_ * 1000.0);
            yamlWatchTimer_ = create_wall_timer(std::chrono::milliseconds(watchMs),
                                                [this]() { watchYamlCallback(); });

            RCLCPP_INFO(get_logger(),
                        "YAML 热加载已启动: %s (监听间隔 %.1f s)",
                        yamlPath_.c_str(),
                        yamlWatchInterval_);
        }

        // ── 检查手柄是否就绪 ──
        if (!joystick_.isFound()) {
            RCLCPP_ERROR(get_logger(),
                         "手柄设备未找到: %s  请确认已连接并具有读取权限",
                         devicePath_.c_str());
            RCLCPP_INFO(get_logger(),
                        "提示: sudo chmod a+rw %s  或将自己加入 input 组",
                        devicePath_.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "手柄已连接: %s", devicePath_.c_str());
        }

        // ── GaitTwist 发布器 ──
        commandPub_ =
            create_publisher<dog_msg::msg::GaitTwist>("/dog/command", rclcpp::QoS(10));

        if (DebugMode) {
            // ── 状态打印定时器 (1 Hz) ──
            statusTimer_ =
                create_wall_timer(std::chrono::seconds(1), [this]() { printStatus(); });
        }

        // ── 启动轮询线程 ──
        int intervalUs = static_cast<int>(1'000'000.0 / pollRateHz);
        pollThread_ = std::thread([this, intervalUs]() {
            while (running_.load()) {
                pollLoop();
                std::this_thread::sleep_for(std::chrono::microseconds(intervalUs));
            }
        });

        RCLCPP_INFO(
            get_logger(), "JoyStickController 启动完成 (轮询频率: %.1f Hz)", pollRateHz);
    }

    JoyStickController::~JoyStickController() {
        running_.store(false);
        if (pollThread_.joinable()) {
            pollThread_.join();
        }
        RCLCPP_INFO(get_logger(), "JoyStickController 已停止");
    }

    // ══════════════════════════════════════════════════════════════════════════════
    // 参数声明
    // ══════════════════════════════════════════════════════════════════════════════

    void JoyStickController::declareParameters() {
        declare_parameter("device_path", devicePath_);
        declare_parameter("deadzone", deadzone_);
        declare_parameter("exponent", exponent_);
        declare_parameter("max_linear_vel", maxLinearVel_);
        declare_parameter("max_angular_vel", maxAngularVel_);
        declare_parameter("yaml_watch_interval", yamlWatchInterval_);

        devicePath_ = get_parameter("device_path").as_string();
        deadzone_ = get_parameter("deadzone").as_double();
        exponent_ = get_parameter("exponent").as_double();
        maxLinearVel_ = get_parameter("max_linear_vel").as_double();
        maxAngularVel_ = get_parameter("max_angular_vel").as_double();
        yamlWatchInterval_ = get_parameter("yaml_watch_interval").as_double();
    }

    // ══════════════════════════════════════════════════════════════════════════════
    // 摇杆映射
    // ══════════════════════════════════════════════════════════════════════════════

    double JoyStickController::mapAxis(short rawValue) const {
        // 归一化到 [-1.0, 1.0]
        double norm = static_cast<double>(rawValue) / 32767.0;

        // 死区
        if (std::abs(norm) < deadzone_) {
            return 0.0;
        }

        // 重映射使死区边界连续: (x - deadzone) / (1 - deadzone)
        double sign = (norm > 0) ? 1.0 : -1.0;
        double scaled = (std::abs(norm) - deadzone_) / (1.0 - deadzone_);

        // 非线性响应曲线 (指数映射，使小幅度操作更精细)
        double curved = std::pow(scaled, exponent_);

        return sign * curved;
    }

    // ══════════════════════════════════════════════════════════════════════════════
    // 手柄轮询
    // ══════════════════════════════════════════════════════════════════════════════

    void JoyStickController::pollLoop() {
        if (!joystick_.isFound()) {
            return;
        }

        // 最新轴值 (static 保持跨轮询周期的持续性)
        static short axisLinX = 0; // 左摇杆上下 (axis 1)
        static short axisAngZ = 0; // 右摇杆左右 (axis 2)

        JoystickEvent event;

        // 一次消费所有积压事件
        while (joystick_.sample(&event)) {
            if (event.isInitialState()) {
                continue;
            }

            if (event.isButton()) {
                bool pressed = (event.value == 1);

                switch (event.number) {
                case 0: // A → WALK
                    if (pressed && !lastBtnA_) {
                        currentGait_.store(GaitMode::WALK);
                        RCLCPP_INFO(get_logger(), "🕹 步态 → WALK");
                    }
                    lastBtnA_ = pressed;
                    break;
                case 1: // B → STAND
                    if (pressed && !lastBtnB_) {
                        currentGait_.store(GaitMode::STAND);
                        RCLCPP_INFO(get_logger(), "🕹 步态 → STAND");
                    }
                    lastBtnB_ = pressed;
                    break;
                case 2: // X → JUMP (一次性命令，原子标志保证只发一条消息)
                    if (pressed && !lastBtnX_) {
                        jumpRequested_.store(true);
                        RCLCPP_INFO(get_logger(), "🕹 步态 → JUMP (单次)");
                    }
                    lastBtnX_ = pressed;
                    break;
                case 4: // LB → 降低步高
                    if (pressed && !lastBtnLB_) {
                        stepHeight_ -= stepHeightStep_;
                        stepHeight_ =
                            std::clamp(stepHeight_, MinstepHeight_, MaxstepHeight_);
                        RCLCPP_INFO(get_logger(),
                                    "⬇ 步高 -%.3f → %.3f m",
                                    stepHeightStep_,
                                    stepHeight_);
                    }
                    lastBtnLB_ = pressed;
                    break;
                case 5: // RB → 升高步高
                    if (pressed && !lastBtnRB_) {
                        stepHeight_ += stepHeightStep_;
                        stepHeight_ =
                            std::clamp(stepHeight_, MinstepHeight_, MaxstepHeight_);
                        RCLCPP_INFO(get_logger(),
                                    "⬆ 步高 +%.3f → %.3f m",
                                    stepHeightStep_,
                                    stepHeight_);
                    }
                    lastBtnRB_ = pressed;
                    break;
                case 6:
                    break; //"Back";
                case 7:
                    break; //"Start";
                case 8:
                    break; //"Home";
                case 9:
                    break; //"LS";
                case 10:
                    break; //"RS";
                default:
                    break;
                }
            }

            if (event.isAxis()) {
                switch (event.number) {
                case 0: // 左摇杆 左右 (向左默认负，ROS向左转正，需取反)
                    axisAngZ = -event.value;
                    break;
                case 1: // 左摇杆 上下 (向上默认负，ROS前进正，需取反)
                    axisLinX = -event.value;
                    break;
                case 2:
                    break; //"LT (刹车) "
                case 3:
                    currentAngularRoll_ = -event.value;
                    break; //"右摇杆 X  "
                case 4:
                    currentAngularPitch_ = -event.value;
                    break; //"右摇杆 Y  "
                case 5:
                    break; //"RT (油门) "
                case 6:    // 十字键 X → 步频调节
                {
                    // 边沿检测：仅在从 0 变为非 0 时触发一次
                    if (lastDpadX_ == 0 && event.value != 0) {
                        if (event.value < 0) {
                            // 十字键左 → 降低步频 (增大周期)
                            stepDuration_ += stepDurationStep_;
                            RCLCPP_INFO(get_logger(),
                                        "◀ 步频周期 +%.2f → %.2f s",
                                        stepDurationStep_,
                                        stepDuration_);
                        } else {
                            // 十字键右 → 升高步频 (减小周期)
                            stepDuration_ -= stepDurationStep_;
                            RCLCPP_INFO(get_logger(),
                                        "▶ 步频周期 -%.2f → %.2f s",
                                        stepDurationStep_,
                                        stepDuration_);
                        }
                        stepDuration_ = std::clamp(stepDuration_, 0.10, 0.80);
                    }
                    lastDpadX_ = event.value;
                } break;
                case 7: // 十字键 Y → 身高调节
                {
                    if (lastDpadY_ == 0 && event.value != 0) {
                        if (event.value < 0) {
                            // 十字键上 → 增加身高
                            bodyHeight_ += heightStep_;
                            RCLCPP_INFO(get_logger(),
                                        "▲ 身高 +%.2f → %.2f m",
                                        heightStep_,
                                        bodyHeight_);
                        } else {
                            // 十字键下 → 降低身高
                            bodyHeight_ -= heightStep_;
                            RCLCPP_INFO(get_logger(),
                                        "▼ 身高 -%.2f → %.2f m",
                                        heightStep_,
                                        bodyHeight_);
                        }
                        bodyHeight_ =
                            std::clamp(bodyHeight_, MinbodyHeight_, MaxbodyHeight_);
                    }
                    lastDpadY_ = event.value;
                } break;
                default:
                    break;
                }
            }
        }

        // ── 组装 GaitTwist 消息并发布 ──
        auto msg = dog_msg::msg::GaitTwist();

        // JUMP 是一次性命令：用原子 exchange 保证每次按键只发一条 JUMP 消息
        if (jumpRequested_.exchange(false)) {
            msg.gait = static_cast<uint8_t>(GaitMode::JUMP);
        } else {
            msg.gait = static_cast<uint8_t>(currentGait_.load());
        }

        msg.step_height = static_cast<float>(stepHeight_);
        msg.base_height = static_cast<float>(bodyHeight_);
        msg.step_duration = static_cast<float>(stepDuration_);
        msg.linear_x = static_cast<float>(mapAxis(axisLinX) * maxLinearVel_);
        msg.linear_y = static_cast<float>(mapAxis(axisAngZ) * maxAngularVel_);
        // msg.roll = static_cast<float>(mapAxis(currentAngularRoll_.load()));
        // msg.pitch = static_cast<float>(mapAxis(currentAngularPitch_.load()));

        if (msg.gait == static_cast<uint8_t>(GaitMode::JUMP)) {
            // JUMP 模式下，linear_y 用于携带跳跃配置索引（上位机约定）
            msg.step_height = 0.0f;
        }

        // 记录当前值用于日志打印
        currentLinearX_.store(msg.linear_x);
        currentAngularZ_.store(msg.linear_y);
        currentAngularRoll_.store(msg.roll);
        currentAngularPitch_.store(msg.pitch);

        commandPub_->publish(msg);
    }

    // ══════════════════════════════════════════════════════════════════════════════
    // 状态输出
    // ══════════════════════════════════════════════════════════════════════════════

    void JoyStickController::printStatus() {
        const char *gaitStr = "STAND";
        switch (currentGait_.load()) {
        case GaitMode::WALK:
            gaitStr = "WALK";
            break;
        case GaitMode::JUMP:
            gaitStr = "JUMP";
            break;
        default:
            break;
        }

        RCLCPP_INFO(
            get_logger(),
            "[状态] 步态: %-5s | 身高: %.2f m | 步高: %.2f m | 步频周期: %.2f s | "
            "速度(x: %+.2f m/s, wz: %+.2f rad/s) | 手柄: %s",
            gaitStr,
            bodyHeight_,
            stepHeight_,
            stepDuration_,
            currentLinearX_.load(),
            currentAngularZ_.load(),
            joystick_.isFound() ? "已连接" : "未连接");
    }

    time_t JoyStickController::file_mtime(const std::string &path) {
        struct stat st;
        return stat(path.c_str(), &st) == 0 ? st.st_mtime : 0;
    }

    // ══════════════════════════════════════════════════════════════════════════════
    // YAML 动态热加载
    // ══════════════════════════════════════════════════════════════════════════════

    void JoyStickController::loadYamlConfig() {
        if (yamlPath_.empty()) {
            return;
        }

        try {
            YAML::Node n = YAML::LoadFile(yamlPath_);

            auto params = n["joystick_controller"]["ros__parameters"];
            if (!params) {
                RCLCPP_WARN(get_logger(),
                            "YAML 中未找到 joystick_controller.ros__parameters 节点");
                return;
            }

            // 手柄设备 & 调试
            if (params["device_path"])
                devicePath_ = params["device_path"].as<std::string>();
            if (params["debug_mode"])
                DebugMode = params["debug_mode"].as<bool>();

            // 摇杆映射
            if (params["deadzone"])
                deadzone_ = params["deadzone"].as<double>();
            if (params["exponent"])
                exponent_ = params["exponent"].as<double>();

            // 速度限制
            if (params["max_linear_vel"])
                maxLinearVel_ = params["max_linear_vel"].as<double>();
            if (params["max_angular_vel"])
                maxAngularVel_ = params["max_angular_vel"].as<double>();

            // 身高参数 (mm)
            if (params["body_height"])
                bodyHeight_ = params["body_height"].as<double>();
            if (params["height_step"])
                heightStep_ = params["height_step"].as<double>();
            if (params["max_body_height"])
                MaxbodyHeight_ = params["max_body_height"].as<double>();
            if (params["min_body_height"])
                MinbodyHeight_ = params["min_body_height"].as<double>();

            // 步高参数 (mm)
            if (params["step_height"])
                stepHeight_ = params["step_height"].as<double>();
            if (params["step_height_step"])
                stepHeightStep_ = params["step_height_step"].as<double>();
            if (params["max_step_height"])
                MaxstepHeight_ = params["max_step_height"].as<double>();
            if (params["min_step_height"])
                MinstepHeight_ = params["min_step_height"].as<double>();

            // 步频参数
            if (params["step_duration"])
                stepDuration_ = params["step_duration"].as<double>();
            if (params["step_duration_step"])
                stepDurationStep_ = params["step_duration_step"].as<double>();

            // 监听间隔
            if (params["yaml_watch_interval"])
                yamlWatchInterval_ = params["yaml_watch_interval"].as<double>();

            RCLCPP_INFO(get_logger(), "YAML 参数热加载完成: %s", yamlPath_.c_str());
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(
                get_logger(), "YAML 解析失败 [%s]: %s", yamlPath_.c_str(), e.what());
        }
    }

    void JoyStickController::watchYamlCallback() {
        if (yamlPath_.empty()) {
            return;
        }

        time_t mtime = file_mtime(yamlPath_);
        if (mtime == 0) {
            return; // 文件不存在
        }

        if (mtime != yamlLastMtime_) {
            yamlLastMtime_ = mtime;
            RCLCPP_INFO(get_logger(), "检测到 YAML 变更, 热重载中...");
            loadYamlConfig();
        }
    }

} // namespace Dog8Dof

// ══════════════════════════════════════════════════════════════════════════════
// main
// ══════════════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Dog8Dof::JoyStickController>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
