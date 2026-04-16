#include "dog_hardware/goM8010.hpp"
#include <atomic>
#include <ctime>
#include <exception>
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

class MotorTesterApp {
  public:
    using MotorPtr = std::shared_ptr<Dog8Dof::goM8010Interface>;

    enum class ContinuousCmdType {
        NONE,
        POS,
        VEL,
        TOR,
        HYB,
        ZERO,
        DAMP,
        LOCK,
        RELEASE,
    };

    struct ContinuousCommand {
        ContinuousCmdType type{ContinuousCmdType::NONE};
        std::string target;
        float p1{0.0f};
        float p2{0.0f};
        float p3{0.0f};
        float p4{0.0f};
        float p5{0.0f};
    };

    int run(int argc, char **argv) {
        rclcpp::init(argc, argv);
        try {
            if (!initMotors()) {
                rclcpp::shutdown();
                return 1;
            }

            initRosPublisher();
            startPublisherThread();
            startCommandThread();
            runCommandLoop();
            stopCommandThread();
            stopPublisherThread();
            rclcpp::shutdown();
            return 0;
        } catch (const std::exception &e) {
            stopCommandThread();
            stopPublisherThread();
            std::cerr << "Exception: " << e.what() << "\n";
            rclcpp::shutdown();
            return 1;
        }
    }

  private:
    std::shared_ptr<SerialPort> port_;
    std::vector<MotorPtr> motors_;
    std::mutex motors_mtx_;
    std::atomic<bool> running_{false};
    std::thread publisher_thread_;
    std::thread command_thread_;
    std::mutex cmd_mtx_;
    ContinuousCommand current_cmd_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    static std::vector<uint8_t> parseIds(const std::string &line) {
        std::string s = line;
        for (char &c : s) {
            if (c == ',') {
                c = ' ';
            }
        }

        std::istringstream iss(s);
        std::vector<uint8_t> ids;
        int v = 0;
        while (iss >> v) {
            if (v >= 0 && v <= 255) {
                ids.push_back(static_cast<uint8_t>(v));
            }
        }
        return ids;
    }

    static bool parseTargetId(const std::string &target, uint8_t &id) {
        try {
            int v = std::stoi(target);
            if (v < 0 || v > 255) {
                return false;
            }
            id = static_cast<uint8_t>(v);
            return true;
        } catch (...) {
            return false;
        }
    }

    MotorPtr findByIdLocked(uint8_t id) {
        for (const auto &m : motors_) {
            if (m && m->getID() == id) {
                return m;
            }
        }
        return nullptr;
    }

    bool initMotors() {
        std::cout << "请输入电机 ID 列表（空格或逗号分隔，例如: 1 2 3 或 1,2,3）: ";
        std::string id_line;
        if (!std::getline(std::cin, id_line) || id_line.empty()) {
            return false;
        }

        std::vector<uint8_t> ids = parseIds(id_line);
        if (ids.empty()) {
            std::cerr << "未提供有效 ID，退出。\n";
            return false;
        }

        port_ = std::make_shared<SerialPort>("/dev/ttyACM0");
        for (uint8_t id : ids) {
            motors_.push_back(std::make_shared<Dog8Dof::goM8010Interface>(id, port_));
        }
        return true;
    }

    void initRosPublisher() {
        node_ = rclcpp::Node::make_shared("motor_state_publisher");
        pub_ = node_->create_publisher<std_msgs::msg::String>("motor_states", 10);
    }

    void startPublisherThread() {
        running_.store(true);
        publisher_thread_ = std::thread(&MotorTesterApp::publisherLoop, this);
    }

    void stopPublisherThread() {
        running_.store(false);
        if (publisher_thread_.joinable()) {
            publisher_thread_.join();
        }
    }

    void publisherLoop() {
        rclcpp::WallRate rate(5ms);
        while (running_.load()) {
            std::ostringstream out;
            out << "[";
            {
                std::lock_guard<std::mutex> lk(motors_mtx_);
                bool first = true;
                for (const auto &m : motors_) {
                    if (!m) {
                        continue;
                    }
                    if (!first) {
                        out << ",";
                    }

                    out << "{";
                    out << "\"id\":" << int(m->getID()) << ",";
                    out << "\"pos\":" << std::fixed << std::setprecision(6) << m->getPos()
                        << ",";
                    out << "\"vel\":" << std::fixed << std::setprecision(6) << m->getVel()
                        << ",";
                    out << "\"tor\":" << std::fixed << std::setprecision(6) << m->getTor()
                        << ",";
                    out << "\"temp\":" << m->getTemp() << ",";
                    out << "\"err\":\"" << m->getError() << "\"";
                    out << "}";
                    first = false;
                }
            }
            out << "]";

            std_msgs::msg::String msg;
            msg.data = out.str();
            pub_->publish(msg);
            rate.sleep();
        }
    }

    void startCommandThread() {
        command_thread_ = std::thread(&MotorTesterApp::commandLoop, this);
    }

    void stopCommandThread() {
        if (command_thread_.joinable()) {
            command_thread_.join();
        }
    }

    void commandLoop() {
        rclcpp::WallRate rate(2ms);
        while (running_.load()) {
            ContinuousCommand cmd;
            {
                std::lock_guard<std::mutex> lk(cmd_mtx_);
                cmd = current_cmd_;
            }

            if (cmd.type != ContinuousCmdType::NONE) {
                executeContinuousCommand(cmd);
            }
            rate.sleep();
        }
    }

    void setCurrentCommand(const ContinuousCommand &cmd) {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        current_cmd_ = cmd;
    }

    void clearCurrentCommand() {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        current_cmd_ = ContinuousCommand{};
    }

    void executeContinuousCommand(const ContinuousCommand &cmd) {
        if (cmd.type == ContinuousCmdType::POS) {
            executePos(cmd.target, cmd.p1, cmd.p2, cmd.p3, false);
        } else if (cmd.type == ContinuousCmdType::VEL) {
            executeVel(cmd.target, cmd.p1, cmd.p2, false);
        } else if (cmd.type == ContinuousCmdType::TOR) {
            executeTor(cmd.target, cmd.p1, false);
        } else if (cmd.type == ContinuousCmdType::HYB) {
            executeHybrid(cmd.target, cmd.p1, cmd.p2, cmd.p3, cmd.p4, cmd.p5, false);
        } else if (cmd.type == ContinuousCmdType::ZERO) {
            executeZero(cmd.target, false);
        } else if (cmd.type == ContinuousCmdType::DAMP) {
            executeDamp(cmd.target, cmd.p1, false);
        } else if (cmd.type == ContinuousCmdType::LOCK) {
            executeLock(cmd.target, false);
        } else if (cmd.type == ContinuousCmdType::RELEASE) {
            executeRelease(cmd.target, false);
        }
    }

    void printIfVerbose(bool verbose, const std::string &text) {
        if (verbose) {
            std::cout << text;
        }
    }

    void printHelp() const {
        std::cout << "pos <id|all> <target> <kp> <kd>\n"
                  << "vel <id|all> <target> <kd>\n"
                  << "tor <id|all> <target>\n"
                  << "hyb <id|all> <torque> <pos> <vel> <kp> <kd>\n"
                  << "zero <id|all>\n"
                  << "setzero <id|all> <offset>\n"
                  << "damp <id|all> <kd>\n"
                  << "lock <id|all>\n"
                  << "release <id|all>\n"
                  << "clear (清除持续命令)\n"
                  << "estop <id|all>\n"
                  << "exit\n";
    }

    bool tryGetTargetMotor(const std::string &target, MotorPtr &motor, int &vid) {
        uint8_t id = 0;
        if (!parseTargetId(target, id)) {
            std::cout << "目标 ID 非法: " << target << std::endl;
            return false;
        }

        vid = static_cast<int>(id);
        std::lock_guard<std::mutex> lk(motors_mtx_);
        motor = findByIdLocked(id);
        if (!motor) {
            std::cout << "找不到 ID:" << vid << std::endl;
            return false;
        }
        return true;
    }

    void
    executePos(const std::string &target, float t, float kp, float kd, bool verbose) {
        if (target == "all") {
            std::lock_guard<std::mutex> lk(motors_mtx_);
            for (const auto &m : motors_) {
                bool ok = m->setPos(t, kp, kd);
                if (verbose) {
                    std::cout << "setPos ID:" << int(m->getID())
                              << (ok ? " -> OK" : " -> FAIL") << std::endl;
                }
            }
            return;
        }

        MotorPtr motor;
        int vid = 0;
        if (!tryGetTargetMotor(target, motor, vid)) {
            return;
        }
        bool ok = motor->setPos(t, kp, kd);
        if (verbose) {
            std::cout << "setPos ID:" << vid << (ok ? " -> OK" : " -> FAIL") << std::endl;
        }
    }

    void handlePos(const std::string &target, std::istringstream &ss) {
        float t = 0.0f, kp = 0.0f, kd = 0.0f;
        ss >> t >> kp >> kd;
        if (!ss) {
            std::cout << "参数错误，格式: pos <id|all> <target> <kp> <kd>\n";
            return;
        }

        ContinuousCommand cmd;
        cmd.type = ContinuousCmdType::POS;
        cmd.target = target;
        cmd.p1 = t;
        cmd.p2 = kp;
        cmd.p3 = kd;
        setCurrentCommand(cmd);
        executePos(target, t, kp, kd, true);
        printIfVerbose(true, "已设置持续命令: pos\n");
    }

    void executeVel(const std::string &target, float t, float kd, bool verbose) {
        if (target == "all") {
            std::lock_guard<std::mutex> lk(motors_mtx_);
            for (const auto &m : motors_) {
                bool ok = m->setVel(t, kd);
                if (verbose) {
                    std::cout << "setVel ID:" << int(m->getID())
                              << (ok ? " -> OK" : " -> FAIL") << std::endl;
                }
            }
            return;
        }

        MotorPtr motor;
        int vid = 0;
        if (!tryGetTargetMotor(target, motor, vid)) {
            return;
        }
        bool ok = motor->setVel(t, kd);
        if (verbose) {
            std::cout << "setVel ID:" << vid << (ok ? " -> OK" : " -> FAIL") << std::endl;
        }
    }

    void handleVel(const std::string &target, std::istringstream &ss) {
        float t = 0.0f, kd = 0.0f;
        ss >> t >> kd;
        if (!ss) {
            std::cout << "参数错误，格式: vel <id|all> <target> <kd>\n";
            return;
        }

        ContinuousCommand cmd;
        cmd.type = ContinuousCmdType::VEL;
        cmd.target = target;
        cmd.p1 = t;
        cmd.p2 = kd;
        setCurrentCommand(cmd);
        executeVel(target, t, kd, true);
        printIfVerbose(true, "已设置持续命令: vel\n");
    }

    void executeTor(const std::string &target, float t, bool verbose) {
        if (target == "all") {
            std::lock_guard<std::mutex> lk(motors_mtx_);
            for (const auto &m : motors_) {
                bool ok = m->setTor(t);
                if (verbose) {
                    std::cout << "setTor ID:" << int(m->getID())
                              << (ok ? " -> OK" : " -> FAIL") << std::endl;
                }
            }
            return;
        }

        MotorPtr motor;
        int vid = 0;
        if (!tryGetTargetMotor(target, motor, vid)) {
            return;
        }
        bool ok = motor->setTor(t);
        if (verbose) {
            std::cout << "setTor ID:" << vid << (ok ? " -> OK" : " -> FAIL") << std::endl;
        }
    }

    void handleTor(const std::string &target, std::istringstream &ss) {
        float t = 0.0f;
        ss >> t;
        if (!ss) {
            std::cout << "参数错误，格式: tor <id|all> <target>\n";
            return;
        }

        ContinuousCommand cmd;
        cmd.type = ContinuousCmdType::TOR;
        cmd.target = target;
        cmd.p1 = t;
        setCurrentCommand(cmd);
        executeTor(target, t, true);
        printIfVerbose(true, "已设置持续命令: tor\n");
    }

    void executeHybrid(const std::string &target,
                       float torque,
                       float pos,
                       float vel,
                       float kp,
                       float kd,
                       bool verbose) {
        if (target == "all") {
            std::lock_guard<std::mutex> lk(motors_mtx_);
            for (const auto &m : motors_) {
                bool ok = m->setHybrid(torque, pos, vel, kp, kd);
                if (verbose) {
                    std::cout << "setHybrid ID:" << int(m->getID())
                              << (ok ? " -> OK" : " -> FAIL") << std::endl;
                }
            }
            return;
        }

        MotorPtr motor;
        int vid = 0;
        if (!tryGetTargetMotor(target, motor, vid)) {
            return;
        }
        bool ok = motor->setHybrid(torque, pos, vel, kp, kd);
        if (verbose) {
            std::cout << "setHybrid ID:" << vid << (ok ? " -> OK" : " -> FAIL")
                      << std::endl;
        }
    }

    void handleHybrid(const std::string &target, std::istringstream &ss) {
        float torque = 0.0f, pos = 0.0f, vel = 0.0f, kp = 0.0f, kd = 0.0f;
        ss >> torque >> pos >> vel >> kp >> kd;
        if (!ss) {
            std::cout << "参数错误，格式: hyb <id|all> <torque> <pos> <vel> <kp> <kd>\n";
            return;
        }

        ContinuousCommand cmd;
        cmd.type = ContinuousCmdType::HYB;
        cmd.target = target;
        cmd.p1 = torque;
        cmd.p2 = pos;
        cmd.p3 = vel;
        cmd.p4 = kp;
        cmd.p5 = kd;
        setCurrentCommand(cmd);
        executeHybrid(target, torque, pos, vel, kp, kd, true);
        printIfVerbose(true, "已设置持续命令: hyb\n");
    }

    void executeZero(const std::string &target, bool verbose) {
        if (target == "all") {
            std::lock_guard<std::mutex> lk(motors_mtx_);
            for (const auto &m : motors_) {
                bool ok = m->zeroTor();
                if (verbose) {
                    std::cout << "zeroTor ID:" << int(m->getID())
                              << (ok ? " -> OK" : " -> FAIL") << std::endl;
                }
            }
            return;
        }

        MotorPtr motor;
        int vid = 0;
        if (!tryGetTargetMotor(target, motor, vid)) {
            return;
        }
        bool ok = motor->zeroTor();
        if (verbose) {
            std::cout << "zeroTor ID:" << vid << (ok ? " -> OK" : " -> FAIL")
                      << std::endl;
        }
    }

    void handleZero(const std::string &target) {
        ContinuousCommand cmd;
        cmd.type = ContinuousCmdType::ZERO;
        cmd.target = target;
        setCurrentCommand(cmd);
        executeZero(target, true);
        printIfVerbose(true, "已设置持续命令: zero\n");
    }

    void handleSetZero(const std::string &target, std::istringstream &ss) {
        float off = 0.0f;
        ss >> off;
        if (!ss) {
            std::cout << "参数错误，格式: setzero <id|all> <offset>\n";
            return;
        }

        if (target == "all") {
            std::lock_guard<std::mutex> lk(motors_mtx_);
            for (const auto &m : motors_) {
                m->setZero(off);
                std::cout << "setZero ID:" << int(m->getID()) << " -> DONE\n";
            }
            return;
        }

        MotorPtr motor;
        int vid = 0;
        if (!tryGetTargetMotor(target, motor, vid)) {
            return;
        }
        motor->setZero(off);
        std::cout << "setZero ID:" << vid << " -> DONE\n";
    }

    void executeDamp(const std::string &target, float kd, bool verbose) {
        if (target == "all") {
            std::lock_guard<std::mutex> lk(motors_mtx_);
            for (const auto &m : motors_) {
                bool ok = m->damp(kd);
                if (verbose) {
                    std::cout << "damp ID:" << int(m->getID())
                              << (ok ? " -> OK" : " -> FAIL") << std::endl;
                }
            }
            return;
        }

        MotorPtr motor;
        int vid = 0;
        if (!tryGetTargetMotor(target, motor, vid)) {
            return;
        }
        bool ok = motor->damp(kd);
        if (verbose) {
            std::cout << "damp ID:" << vid << (ok ? " -> OK" : " -> FAIL") << std::endl;
        }
    }

    void handleDamp(const std::string &target, std::istringstream &ss) {
        float kd = 0.0f;
        ss >> kd;
        if (!ss) {
            std::cout << "参数错误，格式: damp <id|all> <kd>\n";
            return;
        }

        ContinuousCommand cmd;
        cmd.type = ContinuousCmdType::DAMP;
        cmd.target = target;
        cmd.p1 = kd;
        setCurrentCommand(cmd);
        executeDamp(target, kd, true);
        printIfVerbose(true, "已设置持续命令: damp\n");
    }

    void executeLock(const std::string &target, bool verbose) {
        if (target == "all") {
            std::lock_guard<std::mutex> lk(motors_mtx_);
            for (const auto &m : motors_) {
                bool ok = m->lock();
                if (verbose) {
                    std::cout << "stop ID:" << int(m->getID())
                              << (ok ? " -> OK" : " -> FAIL") << std::endl;
                }
            }
            return;
        }

        MotorPtr motor;
        int vid = 0;
        if (!tryGetTargetMotor(target, motor, vid)) {
            return;
        }
        bool ok = motor->lock();
        if (verbose) {
            std::cout << "stop ID:" << vid << (ok ? " -> OK" : " -> FAIL") << std::endl;
        }
    }

    void handleLock(const std::string &target) {
        ContinuousCommand cmd;
        cmd.type = ContinuousCmdType::LOCK;
        cmd.target = target;
        setCurrentCommand(cmd);
        executeLock(target, true);
        printIfVerbose(true, "已设置持续命令: lock\n");
    }

    void executeRelease(const std::string &target, bool verbose) {
        if (target == "all") {
            std::lock_guard<std::mutex> lk(motors_mtx_);
            for (const auto &m : motors_) {
                bool ok = m->release();
                if (verbose) {
                    std::cout << "stop ID:" << int(m->getID())
                              << (ok ? " -> OK" : " -> FAIL") << std::endl;
                }
            }
            return;
        }

        MotorPtr motor;
        int vid = 0;
        if (!tryGetTargetMotor(target, motor, vid)) {
            return;
        }
        bool ok = motor->release();
        if (verbose) {
            std::cout << "stop ID:" << vid << (ok ? " -> OK" : " -> FAIL") << std::endl;
        }
    }

    void handleRelease(const std::string &target) {
        ContinuousCommand cmd;
        cmd.type = ContinuousCmdType::RELEASE;
        cmd.target = target;
        setCurrentCommand(cmd);
        executeRelease(target, true);
        printIfVerbose(true, "已设置持续命令: release\n");
    }

    void handleEstop(const std::string &target) {
        clearCurrentCommand();
        if (target == "all") {
            std::lock_guard<std::mutex> lk(motors_mtx_);
            for (const auto &m : motors_) {
                m->emergencyStop();
                std::cout << "emergencyStop ID:" << int(m->getID()) << " -> DONE\n";
            }
            return;
        }

        MotorPtr motor;
        int vid = 0;
        if (!tryGetTargetMotor(target, motor, vid)) {
            return;
        }
        motor->emergencyStop();
        std::cout << "emergencyStop ID:" << vid << " -> DONE\n";
    }

    void runCommandLoop() {
        std::cout << "可用命令：help pos vel tor hyb zero setzero damp lock release "
                     "clear estop "
                     "exit"
                  << std::endl;
        std::cout << "命令格式示例：pos <id|all> <target> <kp> <kd>" << std::endl;
        std::cout << "控制命令会持续发送，直到输入新命令覆盖；可用 clear 清除。"
                  << std::endl;

        std::string line;
        while (true) {
            std::cout << "> ";
            if (!std::getline(std::cin, line)) {
                break;
            }

            std::istringstream ss(line);
            std::string cmd;
            ss >> cmd;
            if (cmd.empty()) {
                continue;
            }

            if (cmd == "exit" || cmd == "quit" || cmd == "q") {
                break;
            }
            if (cmd == "help") {
                printHelp();
                continue;
            }
            if (cmd == "clear") {
                clearCurrentCommand();
                std::cout << "已清除持续命令\n";
                continue;
            }

            std::string target;
            ss >> target;
            if (target.empty()) {
                std::cout << "缺少目标 ID 或 all\n";
                continue;
            }

            if (cmd == "pos") {
                handlePos(target, ss);
            } else if (cmd == "vel") {
                handleVel(target, ss);
            } else if (cmd == "tor") {
                handleTor(target, ss);
            } else if (cmd == "hyb" || cmd == "hybrid") {
                handleHybrid(target, ss);
            } else if (cmd == "zero") {
                handleZero(target);
            } else if (cmd == "setzero") {
                handleSetZero(target, ss);
            } else if (cmd == "damp") {
                handleDamp(target, ss);
            } else if (cmd == "lock") {
                handleLock(target);
            } else if (cmd == "release") {
                handleRelease(target);
            } else if (cmd == "estop") {
                handleEstop(target);
            } else {
                std::cout << "未知命令，输入 help 查看" << std::endl;
            }
        }
    }
};

int main(int argc, char **argv) {
    MotorTesterApp app;
    return app.run(argc, argv);
}