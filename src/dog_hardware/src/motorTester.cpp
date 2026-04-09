#include "dog_hardware/goM8010.hpp"
#include <atomic>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

static std::vector<uint8_t> parseIds(const std::string &line) {
    std::string s = line;
    for (char &c : s)
        if (c == ',')
            c = ' ';
    std::istringstream iss(s);
    std::vector<uint8_t> ids;
    int v;
    while (iss >> v) {
        if (v >= 0 && v <= 255)
            ids.push_back(static_cast<uint8_t>(v));
    }
    return ids;
}

static std::shared_ptr<Dog8Dof::goM8010Interface>
findById(const std::vector<std::shared_ptr<Dog8Dof::goM8010Interface>> &motors,
         uint8_t id) {
    for (auto &m : motors)
        if (m && m->getID() == id)
            return m;
    return nullptr;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::cout << "请输入电机 ID 列表（空格或逗号分隔，例如: 1 2 3 或 1,2,3）: ";
    std::string idLine;
    if (!std::getline(std::cin, idLine) || idLine.empty()) {
        rclcpp::shutdown();
        return 0;
    }

    auto ids = parseIds(idLine);
    if (ids.empty()) {
        std::cerr << "未提供有效 ID，退出。\n";
        rclcpp::shutdown();
        return 1;
    }

    std::shared_ptr<SerialPort> port;
    std::vector<std::shared_ptr<Dog8Dof::goM8010Interface>> motors;
    std::mutex motors_mtx;
    try {
        port = std::make_shared<SerialPort>("/dev/ttyACM0");
        for (auto id : ids) {
            motors.push_back(std::make_shared<Dog8Dof::goM8010Interface>(id, port));
        }

        // 创建 ROS2 节点与发布器（topic: motor_states）
        auto node = rclcpp::Node::make_shared("motor_state_publisher");
        auto pub = node->create_publisher<std_msgs::msg::String>("motor_states", 10);

        std::atomic<bool> running(true);

        // 后台线程：采集电机状态并通过 ROS2 发布（JSON 数组）
        std::thread publisher_thread([&]() {
            rclcpp::WallRate rate(5ms * 40); // 200ms
            while (running.load()) {
                std::ostringstream out;
                out << "[";
                {
                    std::lock_guard<std::mutex> lk(motors_mtx);
                    bool first = true;
                    for (auto &m : motors) {
                        if (!m)
                            continue;
                        float pos = m->getPos();
                        float vel = m->getVel();
                        float tor = m->getTor();
                        int tmp = m->getTemp();
                        std::string err = m->getError();
                        int mid = int(m->getID());
                        if (!first)
                            out << ",";
                        out << "{";
                        out << "\"id\":" << mid << ",";
                        out << "\"pos\":" << std::fixed << std::setprecision(6) << pos
                            << ",";
                        out << "\"vel\":" << std::fixed << std::setprecision(6) << vel
                            << ",";
                        out << "\"tor\":" << std::fixed << std::setprecision(6) << tor
                            << ",";
                        out << "\"temp\":" << tmp << ",";
                        out << "\"err\":\"" << err << "\"";
                        out << "}";
                        first = false;
                    }
                }
                out << "]";
                std_msgs::msg::String msg;
                msg.data = out.str();
                pub->publish(msg);
                rate.sleep();
            }
        });

        // 主线程：命令交互（与之前逻辑相同），对 motors 操作需加锁
        auto applyTo = [&](const std::string &target, auto fn, const std::string &name) {
            if (target == "all") {
                std::lock_guard<std::mutex> lk(motors_mtx);
                for (auto &m : motors) {
                    bool ok = fn(m);
                    std::cout << name << " ID:" << int(m->getID())
                              << (ok ? " -> OK" : " -> FAIL") << std::endl;
                }
            } else {
                int vid = std::stoi(target);
                std::lock_guard<std::mutex> lk(motors_mtx);
                auto m = findById(motors, static_cast<uint8_t>(vid));
                if (!m) {
                    std::cout << "找不到 ID:" << vid << std::endl;
                    return;
                }
                bool ok = fn(m);
                std::cout << name << " ID:" << vid << (ok ? " -> OK" : " -> FAIL")
                          << std::endl;
            }
        };

        std::cout << "可用命令：help pos vel tor hyb zero setzero damp stop estop exit"
                  << std::endl;
        std::cout << "命令格式示例：pos <id|all> <target> <kp> <kd>" << std::endl;

        std::string line;
        while (true) {
            std::cout << "> ";
            if (!std::getline(std::cin, line))
                break;
            std::istringstream ss(line);
            std::string cmd;
            ss >> cmd;
            if (cmd.empty())
                continue;
            if (cmd == "exit" || cmd == "quit" || cmd == "q")
                break;
            if (cmd == "help") {
                std::cout << "pos <id|all> <target> <kp> <kd>\n"
                          << "vel <id|all> <target> <kd>\n"
                          << "tor <id|all> <target>\n"
                          << "hyb <id|all> <torque> <pos> <vel> <kp> <kd>\n"
                          << "zero <id|all>\n"
                          << "setzero <id|all> <offset>\n"
                          << "damp <id|all> <kd>\n"
                          << "lock <id|all>\n"
                          << "release <id|all>\n"
                          << "estop <id|all>\n"
                          << "exit\n";
                continue;
            }

            std::string tgt;
            ss >> tgt;
            if (tgt.empty()) {
                std::cout << "缺少目标 ID 或 all\n";
                continue;
            }

            if (cmd == "pos") {
                float t, kp, kd;
                ss >> t >> kp >> kd;
                applyTo(tgt, [&](auto m) { return m->setPos(t, kp, kd); }, "setPos");
            } else if (cmd == "vel") {
                float t, kd;
                ss >> t >> kd;
                applyTo(tgt, [&](auto m) { return m->setVel(t, kd); }, "setVel");
            } else if (cmd == "tor") {
                float t;
                ss >> t;
                applyTo(tgt, [&](auto m) { return m->setTor(t); }, "setTor");
            } else if (cmd == "hyb" || cmd == "hybrid") {
                float torque, pos, vel, kp, kd;
                ss >> torque >> pos >> vel >> kp >> kd;
                applyTo(
                    tgt,
                    [&](auto m) { return m->setHybrid(torque, pos, vel, kp, kd); },
                    "setHybrid");
            } else if (cmd == "zero") {
                applyTo(tgt, [&](auto m) { return m->zeroTor(); }, "zeroTor");
            } else if (cmd == "setzero") {
                float off;
                ss >> off;
                if (tgt == "all") {
                    std::lock_guard<std::mutex> lk(motors_mtx);
                    for (auto &m : motors) {
                        m->setZero(off);
                        std::cout << "setZero ID:" << int(m->getID()) << " -> DONE\n";
                    }
                } else {
                    int vid = std::stoi(tgt);
                    std::lock_guard<std::mutex> lk(motors_mtx);
                    auto m = findById(motors, static_cast<uint8_t>(vid));
                    if (m) {
                        m->setZero(off);
                        std::cout << "setZero ID:" << vid << " -> DONE\n";
                    } else
                        std::cout << "找不到 ID:" << vid << std::endl;
                }
            } else if (cmd == "damp") {
                float kd;
                ss >> kd;
                applyTo(tgt, [&](auto m) { return m->damp(kd); }, "damp");
            } else if (cmd == "lock") {
                applyTo(tgt, [&](auto m) { return m->lock(); }, "stop");
            } else if (cmd == "estop") {
                if (tgt == "all") {
                    std::lock_guard<std::mutex> lk(motors_mtx);
                    for (auto &m : motors) {
                        m->emergencyStop();
                        std::cout << "emergencyStop ID:" << int(m->getID())
                                  << " -> DONE\n";
                    }
                } else {
                    int vid = std::stoi(tgt);
                    std::lock_guard<std::mutex> lk(motors_mtx);
                    auto m = findById(motors, static_cast<uint8_t>(vid));
                    if (m) {
                        m->emergencyStop();
                        std::cout << "emergencyStop ID:" << vid << " -> DONE\n";
                    } else
                        std::cout << "找不到 ID:" << vid << std::endl;
                }
            } else if (cmd == "release") {
                applyTo(tgt, [&](auto m) { return m->release(); }, "stop");

            } else {
                std::cout << "未知命令，输入 help 查看" << std::endl;
            }
        }

        running.store(false);
        if (publisher_thread.joinable())
            publisher_thread.join();
        rclcpp::shutdown();
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << "\n";
        rclcpp::shutdown();
        return 1;
    }

    return 0;
}