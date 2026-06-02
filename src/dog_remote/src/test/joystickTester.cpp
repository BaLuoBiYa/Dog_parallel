/**
 * @file joystickTester.cpp
 * @brief 手柄原始数据测试工具
 *
 *   - 不依赖 ROS，直接使用 Joystick 库读取 /dev/input/jsX
 *   - 实时显示所有轴/按键的原始值
 *   - 同时展示映射后的 Twist 速度
 *   - 支持命令行指定设备路径
 *
 * 用法:
 *   ./joystickTester                      # 默认 /dev/input/js0
 *   ./joystickTester /dev/input/js1       # 指定设备
 *
 * 按键:
 *   Ctrl+C 退出
 */

#include "dog_remote/joystick.hh"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

// ══════════════════════════════════════════════════════════════════════════════
// 常量
// ══════════════════════════════════════════════════════════════════════════════

static constexpr int MAX_AXES = 8;
static constexpr int MAX_BUTTONS = 16;

// 显示参数 (与 JoyStickController 保持一致)
static constexpr double DEADZONE = 0.05;
static constexpr double EXPONENT = 1.5;
static constexpr double MAX_LINEAR_VEL = 0.8;  // m/s
static constexpr double MAX_ANGULAR_VEL = 1.2; // rad/s

// 全局 running 标志 (信号安全)
static volatile sig_atomic_t g_running = 1;

static void sigHandler(int) {
    g_running = 0;
}

// ══════════════════════════════════════════════════════════════════════════════
// 工具函数
// ══════════════════════════════════════════════════════════════════════════════

/** 归一化 + 死区 + 指数映射 (与 JoyStickController::mapAxis 相同) */
static double mapAxis(short rawValue) {
    double norm = static_cast<double>(rawValue) / 32767.0;

    if (std::abs(norm) < DEADZONE) {
        return 0.0;
    }

    double sign = (norm > 0) ? 1.0 : -1.0;
    double scaled = (std::abs(norm) - DEADZONE) / (1.0 - DEADZONE);
    double curved = std::pow(scaled, EXPONENT);

    return sign * curved;
}

/** 将轴原始值映射为百分比字符串 */
static std::string axisBar(short val) {
    double pct = static_cast<double>(val) / 32767.0; // [-1, 1]
    int width = 20;
    int pos = static_cast<int>((pct + 1.0) / 2.0 * width); // [0, width]
    pos = std::clamp(pos, 0, width);

    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < width; ++i) {
        if (i == width / 2) {
            oss << (pos == i ? "\033[1;32m◆\033[0m" : "|");
        } else {
            oss << (i == pos ? "\033[1;32m●\033[0m" : "·");
        }
    }
    oss << "]";
    return oss.str();
}

// ══════════════════════════════════════════════════════════════════════════════
// 轴名称 (Xbox 布局)
// ══════════════════════════════════════════════════════════════════════════════

static const char *axisName(int id) {
    switch (id) {
    case 0:
        return "左摇杆 X  ";
    case 1:
        return "左摇杆 Y  ";
    case 2:
        return "LT (刹车) ";
    case 3:
        return "右摇杆 X  ";
    case 4:
        return "右摇杆 Y  ";
    case 5:
        return "RT (油门) ";
    case 6:
        return "十字键 X  ";
    case 7:
        return "十字键 Y  ";
    default:
        return "? 未知轴  ";
    }
}

static const char *btnName(int id) {
    switch (id) {
    case 0:
        return "A";
    case 1:
        return "B";
    case 2:
        return "X";
    case 3:
        return "Y";
    case 4:
        return "LB";
    case 5:
        return "RB";
    case 6:
        return "Back";
    case 7:
        return "Start";
    case 8:
        return "Home";
    case 9:
        return "LS";
    case 10:
        return "RS";
    default:
        return "?";
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// 主逻辑
// ══════════════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);

    // ── 设备路径 ──
    std::string devicePath = "/dev/input/js1";
    if (argc > 1) {
        devicePath = argv[1];
    }

    // ── 打开手柄 ──
    Joystick js(devicePath);
    if (!js.isFound()) {
        std::cerr << "❌ 无法打开手柄设备: " << devicePath << "\n"
                  << "   提示: 请确认设备已连接\n"
                  << "         sudo chmod a+rw " << devicePath << "\n"
                  << "         或将自己加入 input 组: sudo usermod -aG input $USER\n";
        return 1;
    }

    std::cout << "✅ 手柄已连接: " << devicePath << "\n"
              << "   按 Ctrl+C 退出\n\n";

    // ── 状态数组 ──
    std::array<short, MAX_AXES> axes{};
    std::array<bool, MAX_BUTTONS> buttons{};
    axes.fill(0);

    // ── 用于判断是否首次渲染 ──
    int displayLines = 0;
    bool firstRender = true;

    // ── 主循环 ──
    while (g_running) {
        JoystickEvent event;
        bool hasEvent = false;

        // 消费所有积压事件
        while (js.sample(&event)) {
            hasEvent = true;

            if (event.isInitialState()) {
                continue;
            }

            if (event.isAxis() && event.number < MAX_AXES) {
                axes[event.number] = event.value;
            }

            if (event.isButton() && event.number < MAX_BUTTONS) {
                buttons[event.number] = (event.value == 1);
            }
        }

        if (!hasEvent) {
            // 没有新事件，短暂休眠避免 CPU 空转
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // ── 构建显示内容 ──
        std::ostringstream out;

        // 标题
        out << "╔══════════════════════════════════════════════════════════════╗\n";
        out << "║                手柄测试 - " << devicePath
            << std::setw(23 - devicePath.size()) << "║\n";
        out << "╠══════════════════════════════════════════════════════════════╣\n";

        // ── 轴显示 ──
        out << "║  轴 (Axes)                                                  ║\n";
        out << "╠══════════════════════════════════════════════════════════════╣\n";

        for (int i = 0; i < MAX_AXES; ++i) {
            short val = axes[i];
            double mapped = mapAxis(val);
            int pct = static_cast<int>(val / 327.67); // -100 ~ 100

            out << "║ " << axisName(i) << " " << std::setw(6) << val << " ("
                << std::setw(4) << pct << "%) " << axisBar(val) << "  → " << std::fixed
                << std::setprecision(2) << std::setw(6) << mapped << " ║\n";
        }

        // ── 按钮显示 ──
        out << "╠══════════════════════════════════════════════════════════════╣\n";
        out << "║  按键 (Buttons)                                             ║\n";
        out << "╠══════════════════════════════════════════════════════════════╣\n";

        out << "║ ";
        for (int i = 0; i < MAX_BUTTONS; ++i) {
            if (buttons[i]) {
                out << "\033[1;33m[" << std::setw(5) << btnName(i) << "]\033[0m ";
            } else {
                out << "[" << std::setw(5) << btnName(i) << "] ";
            }
            if (i == 7) { // 换行
                out << " ║\n║ ";
            }
        }
        out << "        ║\n";

        // ── Twist 预览 ──
        out << "╠══════════════════════════════════════════════════════════════╣\n";
        out << "║  映射速度 (Twist)                                           ║\n";
        out << "╠══════════════════════════════════════════════════════════════╣\n";

        double linX = mapAxis(-axes[1]) * MAX_LINEAR_VEL;  // 左摇杆 Y → 前进 (Y反向)
        double angZ = mapAxis(-axes[3]) * MAX_ANGULAR_VEL; // 右摇杆 X → 转向 (X需转正负)

        out << "║  linear.x  = " << std::setw(7) << std::fixed << std::setprecision(3)
            << linX << " m/s                                   ║\n";
        out << "║  angular.z = " << std::setw(7) << angZ
            << " rad/s                                 ║\n";

        out << "╚══════════════════════════════════════════════════════════════╝\n";

        // ── 输出 (首次后回退覆盖) ──
        auto str = out.str();

        // 计算需要回退的行数
        int newLines = static_cast<int>(std::count(str.begin(), str.end(), '\n'));
        if (!firstRender && displayLines > 0) {
            std::cout << "\033[" << displayLines << "A"; // 光标回退
        }
        std::cout << str << std::flush;

        displayLines = newLines;
        firstRender = false;
    }

    // ── 清理 ──
    std::cout << "\n\n👋 手柄测试已退出\n";
    return 0;
}
