#include "dog_hardware/serial.hpp"

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>

// ============================================================================
// Debug 耗时分析开关
// ============================================================================
#define SERIAL_TIMING_DEBUG 0

#if SERIAL_TIMING_DEBUG
#include <algorithm>
#include <fstream>
#include <mutex>
#include <numeric>
#include <sstream>

namespace {
    void timing_log(const std::string &msg) {
        static std::mutex mtx;
        std::lock_guard<std::mutex> lock(mtx);
        std::ofstream f("/tmp/serial_timing.log", std::ios::app);
        if (f.is_open()) {
            auto now = std::chrono::steady_clock::now();
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                          now.time_since_epoch())
                          .count();
            f << "[" << us << "] " << msg << std::endl;
        }
    }
} // namespace
#endif

// ============================================================================
// 构造函数 / 析构函数
// ============================================================================

namespace Dog8Dof {
    SerialPort::SerialPort(const std::string &portName,
                           size_t recvLength,
                           uint32_t baudrate,
                           size_t timeOutUs,
                           bool lowLatency)
        : io_(), serial_(io_), portName_(portName), recvLength_(recvLength),
          timeOutUs_(timeOutUs), lowLatency_(lowLatency) {
        try {
            serial_.open(portName_);
            configure_port(baudrate);
            if (lowLatency_)
                apply_low_latency();
        } catch (const boost::system::system_error &e) {
            throw std::ios_base::failure("SerialPort: failed to open " + portName_ +
                                         " - " + e.what());
        }
    }

    SerialPort::~SerialPort() {
        try {
            if (serial_.is_open()) {
                serial_.cancel();
                serial_.close();
            }
        } catch (...) {
            // 析构函数不抛出异常
        }
    }

    // ============================================================================
    // 串口配置
    // ============================================================================

    void SerialPort::configure_port(uint32_t baudrate) {
        using namespace boost::asio;
        serial_.set_option(serial_port_base::baud_rate(baudrate));
        serial_.set_option(serial_port_base::character_size(8));
        serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
        serial_.set_option(
            serial_port_base::flow_control(serial_port_base::flow_control::none));
    }

    void SerialPort::apply_low_latency() {
        int fd = serial_.native_handle();
        struct serial_struct serial_info;
        if (ioctl(fd, TIOCGSERIAL, &serial_info) == 0) {
            serial_info.flags |= ASYNC_LOW_LATENCY;
            ioctl(fd, TIOCSSERIAL, &serial_info);
        }
    }

    void SerialPort::resetSerial(size_t recvLength,
                                 uint32_t baudrate,
                                 size_t timeOutUs,
                                 bool lowLatency) {
        recvLength_ = recvLength;
        timeOutUs_ = timeOutUs;
        lowLatency_ = lowLatency;

        if (serial_.is_open()) {
            serial_.cancel();
            serial_.close();
        }
        io_.restart();

        serial_.open(portName_);
        configure_port(baudrate);
        if (lowLatency_)
            apply_low_latency();
    }

    // ============================================================================
    // 原始字节 I/O
    // ============================================================================

    size_t SerialPort::send(uint8_t *sendMsg, size_t sendLength) {
#if SERIAL_TIMING_DEBUG
        auto t0 = std::chrono::steady_clock::now();
#endif
        boost::system::error_code ec;
        size_t n =
            boost::asio::write(serial_, boost::asio::buffer(sendMsg, sendLength), ec);
        if (ec) {
            std::cerr << "[SerialPort] send error: " << ec.message() << std::endl;
            return 0;
        }
#if SERIAL_TIMING_DEBUG
        auto t1 = std::chrono::steady_clock::now();
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        std::ostringstream oss;
        oss << "[TIMING] send " << sendLength << "B: " << us << " us";
        timing_log(oss.str());
#endif
        return n;
    }

    size_t SerialPort::recv(uint8_t *recvMsg, size_t recvLength) {
        int fd = serial_.native_handle();
        size_t total = 0;

        auto start = std::chrono::steady_clock::now();
        auto deadline = start + std::chrono::microseconds(timeOutUs_);

#if SERIAL_TIMING_DEBUG
        int select_calls = 0;
        long select_total_us = 0;
        int read_calls = 0;
        long read_total_us = 0;
#endif

        while (total < recvLength) {
            auto now = std::chrono::steady_clock::now();
            if (now >= deadline)
                break;

            auto remaining =
                std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);

            struct timeval tv;
            tv.tv_sec = static_cast<time_t>(remaining.count() / 1000000);
            tv.tv_usec = static_cast<suseconds_t>(remaining.count() % 1000000);

#if SERIAL_TIMING_DEBUG
            auto ts0 = std::chrono::steady_clock::now();
#endif
            int ret = select(fd + 1, &rfds, nullptr, nullptr, &tv);
#if SERIAL_TIMING_DEBUG
            auto ts1 = std::chrono::steady_clock::now();
            select_total_us +=
                std::chrono::duration_cast<std::chrono::microseconds>(ts1 - ts0).count();
            ++select_calls;
#endif
            if (ret < 0) {
                // select 被信号中断，重试
                if (errno == EINTR)
                    continue;
                return 0;
            }
            if (ret == 0)
                break; // 超时

#if SERIAL_TIMING_DEBUG
            auto tr0 = std::chrono::steady_clock::now();
#endif
            boost::system::error_code ec;
            size_t n = serial_.read_some(
                boost::asio::buffer(recvMsg + total, recvLength - total), ec);
#if SERIAL_TIMING_DEBUG
            auto tr1 = std::chrono::steady_clock::now();
            read_total_us +=
                std::chrono::duration_cast<std::chrono::microseconds>(tr1 - tr0).count();
            ++read_calls;
#endif
            if (ec) {
                if (ec != boost::asio::error::would_block &&
                    ec != boost::asio::error::try_again) {
                    std::cerr << "[SerialPort] recv error: " << ec.message() << std::endl;
                }
                break;
            }
            total += n;
        }

#if SERIAL_TIMING_DEBUG
        auto end = std::chrono::steady_clock::now();
        auto total_us =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::ostringstream oss;
        oss << "[TIMING] recv " << total << "/" << recvLength << "B: " << total_us
            << " us (select: " << select_calls << "x=" << select_total_us
            << " us, read: " << read_calls << "x=" << read_total_us << " us)";
        timing_log(oss.str());
#endif
        return total;
    }

    size_t SerialPort::recv(uint8_t *recvMsg) {
        return recv(recvMsg, recvLength_);
    }

    // ============================================================================
    // 原始字节 sendRecv
    // ============================================================================

    bool SerialPort::sendRecv(uint8_t *sendMsg, uint8_t *recvMsg, size_t sendLength) {
        if (send(sendMsg, sendLength) != sendLength)
            return false;
        return recv(recvMsg, recvLength_) == recvLength_;
    }

    // ============================================================================
    // MotorCmd / MotorData sendRecv（委托给 SDK 的 modify_data / extract_data）
    // ============================================================================

    bool SerialPort::sendRecv(MotorCmd *cmd, MotorData *data) {
        if (cmd->motorType != data->motorType) {
            std::cerr << "[SerialPort] motorType mismatch" << std::endl;
            return false;
        }

        // 1) 使用 SDK 方法打包 MotorCmd → 内部 ControlData_t / MasterComdDataV3
        cmd->modify_data(cmd);
        uint8_t *send_buf = cmd->get_motor_send_data();
        size_t send_len = static_cast<size_t>(cmd->hex_len);

        // 2) 获取接收缓冲区指针
        uint8_t *recv_buf = data->get_motor_recv_data();

        // 3) 发送 → 接收
        if (!sendRecv(send_buf, recv_buf, send_len)) {
            data->correct = false;
            return false;
        }

        // 4) 使用 SDK 方法解析原始数据 → MotorData
        return data->extract_data(data);
    }

    bool SerialPort::sendRecv(std::vector<MotorCmd> &sendVec,
                              std::vector<MotorData> &recvVec) {
        if (sendVec.size() != recvVec.size()) {
            std::cerr << "[SerialPort] sendVec & recvVec size mismatch" << std::endl;
            return false;
        }

#if SERIAL_TIMING_DEBUG
        auto t0 = std::chrono::steady_clock::now();
        std::vector<long> per_motor_us;
        per_motor_us.reserve(sendVec.size());
#endif

        bool all_ok = true;
        for (size_t i = 0; i < sendVec.size(); ++i) {
#if SERIAL_TIMING_DEBUG
            auto tm0 = std::chrono::steady_clock::now();
#endif
            if (!sendRecv(&sendVec[i], &recvVec[i])) {
                all_ok = false;
            }
#if SERIAL_TIMING_DEBUG
            auto tm1 = std::chrono::steady_clock::now();
            per_motor_us.push_back(
                std::chrono::duration_cast<std::chrono::microseconds>(tm1 - tm0).count());
#endif
        }

#if SERIAL_TIMING_DEBUG
        auto t1 = std::chrono::steady_clock::now();
        auto total_us =
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        auto sum_motor = std::accumulate(per_motor_us.begin(), per_motor_us.end(), 0L);
        auto min_motor = *std::min_element(per_motor_us.begin(), per_motor_us.end());
        auto max_motor = *std::max_element(per_motor_us.begin(), per_motor_us.end());
        auto avg_motor =
            per_motor_us.empty() ? 0 : sum_motor / static_cast<long>(per_motor_us.size());
        std::ostringstream oss;
        oss << "[TIMING] sendRecv " << sendVec.size() << " motors: TOTAL=" << total_us
            << " us, sum=" << sum_motor << " us, " << "avg=" << avg_motor
            << " us, min=" << min_motor << " us, max=" << max_motor << " us, "
            << "overhead=" << (total_us - sum_motor) << " us";
        timing_log(oss.str());
#endif
        return all_ok;
    }

} // namespace Dog8Dof
