/**
 * @file sbusGaitTwistNode.cpp
 * @brief SBUS 遥控器 → GaitTwist 转换节点实现
 *
 * 从 SbusReader 读取遥控器各通道值，映射为 dog_msg/GaitTwist 高层步态指令，
 * 以固定频率发布到 /dog/command 话题。
 *
 * 通道映射:
 *   CH0  RIGHTX        → roll          (副翼)
 *   CH1  RIGHTY        → pitch         (升降舵)
 *   CH2  LEFTY         → linear_x      (油门，前进/后退)
 *   CH3  LEFTX         → linear_y      (方向舵，侧向)
 *   CH4  LEFTSWITCH3   → gait 模式切换  (上=WALK, 中=STAND, 下=PARALYSE)
 *   CH5  LEFTSWITCH2   → 使能控制       (上=使能, 下=急停)
 *   CH6  LEFTROTARY    → base_height   (旋钮，高度调整)
 *   CH7  RIGHTROTARY   → step_duration  (旋钮，步频调整)
 *   CH8  LEFTBOTTOMX   → step_height   (旋钮，步高调整)
 *   CH12 LEFTTOPBOTTOM → JUMP 触发      (按钮，跳跃)
 */

#include "dog_remote/sbusGaitTwistNode.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sys/stat.h>

#include <yaml-cpp/yaml.h>

namespace Dog8Dof {

    // ═══════════════════════════════════════════════════════════════
    // 构造函数
    // ═══════════════════════════════════════════════════════════════
    SbusGaitTwistNode::SbusGaitTwistNode(const std::string &port,
                                         uint32_t baudrate,
                                         const std::string &nodeName,
                                         const std::string &yamlPath,
                                         double publishRateHz)
        : Node(nodeName), serialPort_(port), baudrate_(baudrate),
          yamlPath_(yamlPath), publishRateHz_(publishRateHz) {

        // ── 从 YAML 加载 (唯一配置来源) ──
        if (!yamlPath_.empty()) {
            loadYamlConfig();
        } else {
            RCLCPP_WARN(get_logger(),
                        "未提供 YAML 配置文件路径，使用构造函数默认值");
        }

        // ── 创建 SBUS 读取器 ──
        RCLCPP_INFO(get_logger(),
                    "正在打开 SBUS 串口: %s @ %d baud",
                    serialPort_.c_str(),
                    baudrate_);
        try {
            sbus_ = std::make_unique<SbusReader>(serialPort_, baudrate_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "SBUS 串口初始化失败: %s", e.what());
            throw;
        }

        // ── 创建发布者 ──
        commandPub_ = create_publisher<dog_msg::msg::GaitTwist>(
            "/dog/command", rclcpp::SystemDefaultsQoS());

        // ── 创建发布定时器 ──
        const auto period = std::chrono::duration<double>(1.0 / publishRateHz_);
        publishTimer_ = create_wall_timer(
            period, std::bind(&SbusGaitTwistNode::publishCallback, this));

        // ── YAML 热加载定时器 ──
        if (!yamlPath_.empty()) {
            const auto yamlPeriod = std::chrono::duration<double>(yamlWatchInterval_);
            yamlWatchTimer_ = create_wall_timer(
                yamlPeriod, std::bind(&SbusGaitTwistNode::watchYamlCallback, this));
            RCLCPP_INFO(get_logger(), "YAML 热加载已启用: %s", yamlPath_.c_str());
        }

        RCLCPP_INFO(
            get_logger(), "SbusGaitTwistNode 启动完成, 发布频率: %.1f Hz", publishRateHz);
    }

    // ═══════════════════════════════════════════════════════════════
    // 析构函数
    // ═══════════════════════════════════════════════════════════════
    SbusGaitTwistNode::~SbusGaitTwistNode() {
        RCLCPP_INFO(get_logger(), "SbusGaitTwistNode 正在关闭...");
        // sbus_ unique_ptr 自动析构，关闭串口
    }

    // ═══════════════════════════════════════════════════════════════
    // 定时发布回调
    // ═══════════════════════════════════════════════════════════════
    void SbusGaitTwistNode::publishCallback() {
        // 检查是否有新鲜数据帧
        if (!sbus_->hasFreshFrame(0.5)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "SBUS 数据超时，无新鲜数据帧");
            return;
        }

        // ── 读取所有通道 ──
        const auto channels = sbus_->getAllChannelValues();

        // ── 使能检测 (CH5 LEFTSWITCH2) ──
        // sw2 上 (>1200) = 使能, 下 (<800) = 急停
        const uint16_t enableVal = channels[Channel::LEFTSWITCH2];
        const bool prevEnabled = enabled_.load();
        const bool nowEnabled = (enableVal > 1200);

        if (nowEnabled != prevEnabled) {
            enabled_.store(nowEnabled);
            if (nowEnabled) {
                RCLCPP_INFO(get_logger(), "遥控器使能: ON");
            } else {
                RCLCPP_INFO(get_logger(), "遥控器使能: OFF (急停)");
            }
        }

        // ── 构建 GaitTwist 消息 ──
        dog_msg::msg::GaitTwist cmd;

        if (!nowEnabled) {
            // 急停模式: 强制 STAND, 零速度
            cmd.gait = Gait::STAND;
            cmd.linear_x = 0.0f;
            cmd.linear_y = 0.0f;
            cmd.roll = 0.0f;
            cmd.pitch = 0.0f;
            cmd.base_height = (baseHeightMin_ + baseHeightMax_) * 0.5f;
            cmd.step_height = 0.0f;
            cmd.step_duration = stepDurationMin_;
            commandPub_->publish(cmd);
            return;
        }

        // ── 步态模式 (CH4 LEFTSWITCH3) ──
        // sw3: 上 (>1200) = WALK, 中 (800~1200) = STAND, 下 (<800) = PARALYSE → STAND
        const uint16_t gaitVal = channels[Channel::LEFTSWITCH3];
        if (gaitVal > 1200) {
            cmd.gait = Gait::WALK;
        } else {
            cmd.gait = Gait::STAND;
        }

        // ── 跳跃触发 (CH12 LEFTTOPBOTTOM) ──
        // 按钮按下 (>1400) 且上一帧未按下 → 触发 JUMP
        const bool jumpBtn = (channels[Channel::LEFTTOPBOTTOM] > 1400);
        if (jumpBtn && !lastJumpBtn_) {
            cmd.gait = Gait::JUMP;
            RCLCPP_INFO(get_logger(), "跳跃触发!");
        }
        lastJumpBtn_ = jumpBtn;

        // ── 摇杆映射 ──
        cmd.roll = mapStick(channels[Channel::RIGHTX]) * maxRoll_;
        cmd.pitch = mapStick(channels[Channel::RIGHTY]) * maxPitch_;
        cmd.linear_x = mapStick(channels[Channel::LEFTY]) * maxLinearX_;
        cmd.linear_y = mapStick(channels[Channel::LEFTX]) * maxLinearY_;

        // ── 旋钮映射 ──
        cmd.base_height =
            mapKnob(channels[Channel::LEFTROTARY], baseHeightMin_, baseHeightMax_);
        cmd.step_duration =
            mapKnob(channels[Channel::RIGHTROTARY], stepDurationMin_, stepDurationMax_);
        cmd.step_height =
            mapKnob(channels[Channel::LEFTBOTTOMX], stepHeightMin_, stepHeightMax_);

        // ── 发布 ──
        commandPub_->publish(cmd);
    }

    // ═══════════════════════════════════════════════════════════════
    // 摇杆映射: SBUS → 归一化 [-1, 1] (带死区与指数响应)
    // ═══════════════════════════════════════════════════════════════
    float SbusGaitTwistNode::mapStick(uint16_t sbusVal) const {
        // 将 SBUS 值线性映射到 [-1, 1]
        // SBUS 范围: [192, 1792], 中点: 992
        float normalized;

        if (sbusVal <= kSbusMid) {
            // 下半段: [192, 992] → [-1, 0]
            if (sbusVal <= kSbusMin) {
                normalized = -1.0f;
            } else {
                normalized = -static_cast<float>(kSbusMid - sbusVal) /
                             static_cast<float>(kSbusMid - kSbusMin);
            }
        } else {
            // 上半段: [992, 1792] → [0, 1]
            if (sbusVal >= kSbusMax) {
                normalized = 1.0f;
            } else {
                normalized = static_cast<float>(sbusVal - kSbusMid) /
                             static_cast<float>(kSbusMax - kSbusMid);
            }
        }

        // ── 死区处理 ──
        if (std::abs(normalized) < stickDeadzone_) {
            return 0.0f;
        }

        // 重映射: 死区外 → [0, 1] 范围
        const float sign = (normalized > 0.0f) ? 1.0f : -1.0f;
        const float absNorm =
            (std::abs(normalized) - stickDeadzone_) / (1.0f - stickDeadzone_);

        // ── 指数响应 ──
        return sign * std::pow(absNorm, stickExponent_);
    }

    // ═══════════════════════════════════════════════════════════════
    // 旋钮映射: SBUS → [outMin, outMax] 线性
    // ═══════════════════════════════════════════════════════════════
    float SbusGaitTwistNode::mapKnob(uint16_t sbusVal, float outMin, float outMax) const {
        // 钳位 SBUS 值到有效范围
        const uint16_t clamped = std::clamp(sbusVal, kSbusMin, kSbusMax);

        // 线性映射: [kSbusMin, kSbusMax] → [outMin, outMax]
        const float ratio = static_cast<float>(clamped - kSbusMin) /
                            static_cast<float>(kSbusMax - kSbusMin);
        return outMin + ratio * (outMax - outMin);
    }

    // ═══════════════════════════════════════════════════════════════
    // YAML 配置文件加载 (唯一配置来源)
    // ═══════════════════════════════════════════════════════════════
    void SbusGaitTwistNode::loadYamlConfig() {
        if (yamlPath_.empty()) {
            return;
        }

        std::ifstream file(yamlPath_);
        if (!file.is_open()) {
            RCLCPP_WARN(get_logger(), "无法打开 YAML 配置文件: %s", yamlPath_.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "正在加载 YAML 配置: %s", yamlPath_.c_str());

        try {
            YAML::Node config = YAML::LoadFile(yamlPath_);

            // ── 发布频率 ──
            if (config["publish_rate_hz"]) {
                publishRateHz_ = config["publish_rate_hz"].as<double>();
            }

            // ── 串口配置 ──
            if (config["serial"]) {
                const auto &s = config["serial"];
                if (s["port"]) {
                    serialPort_ = s["port"].as<std::string>();
                }
                if (s["baudrate"]) {
                    baudrate_ = s["baudrate"].as<uint32_t>();
                }
            }

            // ── 限幅参数 ──
            if (config["limits"]) {
                const auto &lim = config["limits"];
                if (lim["max_linear_x"])
                    maxLinearX_ = lim["max_linear_x"].as<float>();
                if (lim["max_linear_y"])
                    maxLinearY_ = lim["max_linear_y"].as<float>();
                if (lim["max_roll"])
                    maxRoll_ = lim["max_roll"].as<float>();
                if (lim["max_pitch"])
                    maxPitch_ = lim["max_pitch"].as<float>();
                if (lim["base_height_min"])
                    baseHeightMin_ = lim["base_height_min"].as<float>();
                if (lim["base_height_max"])
                    baseHeightMax_ = lim["base_height_max"].as<float>();
                if (lim["step_height_min"])
                    stepHeightMin_ = lim["step_height_min"].as<float>();
                if (lim["step_height_max"])
                    stepHeightMax_ = lim["step_height_max"].as<float>();
                if (lim["step_duration_min"])
                    stepDurationMin_ = lim["step_duration_min"].as<float>();
                if (lim["step_duration_max"])
                    stepDurationMax_ = lim["step_duration_max"].as<float>();
            }

            // ── 摇杆参数 ──
            if (config["stick"]) {
                const auto &stk = config["stick"];
                if (stk["deadzone"])
                    stickDeadzone_ = stk["deadzone"].as<float>();
                if (stk["exponent"])
                    stickExponent_ = stk["exponent"].as<float>();
            }

            // ── YAML 监视间隔 ──
            if (config["yaml_watch_interval"]) {
                yamlWatchInterval_ = config["yaml_watch_interval"].as<double>();
            }

            yamlLastMtime_ = fileMtime(yamlPath_);

            RCLCPP_INFO(get_logger(),
                        "YAML 配置加载完成: port=%s, baud=%d, "
                        "maxLinearX=%.2f, maxLinearY=%.2f, "
                        "height=[%.1f, %.1f], stepH=[%.1f, %.1f], "
                        "stepDur=[%.2f, %.2f]",
                        serialPort_.c_str(),
                        baudrate_,
                        maxLinearX_,
                        maxLinearY_,
                        baseHeightMin_,
                        baseHeightMax_,
                        stepHeightMin_,
                        stepHeightMax_,
                        stepDurationMin_,
                        stepDurationMax_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "YAML 解析失败: %s", e.what());
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // YAML 文件热加载检查
    // ═══════════════════════════════════════════════════════════════
    void SbusGaitTwistNode::watchYamlCallback() {
        if (yamlPath_.empty()) {
            return;
        }

        const time_t mtime = fileMtime(yamlPath_);
        if (mtime == 0) {
            // 文件不存在
            return;
        }

        if (mtime != yamlLastMtime_) {
            RCLCPP_INFO(get_logger(), "检测到 YAML 文件更新, 重新加载...");
            yamlLastMtime_ = mtime;
            loadYamlConfig();
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // 文件修改时间
    // ═══════════════════════════════════════════════════════════════
    time_t SbusGaitTwistNode::fileMtime(const std::string &path) {
        struct stat st;
        if (::stat(path.c_str(), &st) != 0) {
            return 0;
        }
        return st.st_mtime;
    }

} // namespace Dog8Dof

// ═══════════════════════════════════════════════════════════════════
// 主函数
// ═══════════════════════════════════════════════════════════════════
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Dog8Dof::SbusGaitTwistNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
