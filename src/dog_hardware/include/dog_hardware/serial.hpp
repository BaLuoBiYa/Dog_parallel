#pragma once

#include <boost/asio.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include "unitree_actuator_sdk/unitreeMotor/unitreeMotor.h"

/**
 * @brief 基于 Boost.Asio 的独立串口通信类
 *
 * 串口 I/O 使用 Boost.Asio 自行实现（不依赖 libUnitreeMotorSDK .so 中的 SerialPort）,
 * MotorCmd 打包/MotorData 解析使用 MotorCmd/MotorData 结构体自带的 .so 方法
 * (modify_data / extract_data / get_motor_send_data / get_motor_recv_data)。
 */
namespace Dog8Dof {
    class SerialPort {
      public:
        /**
         * @brief 构造函数，打开并配置串口
         * @param portName   设备路径，如 /dev/ttyACM0
         * @param recvLength 期望接收长度 (GO_M8010_6: 16)
         * @param baudrate   波特率 (GO_M8010_6: 4000000)
         * @param timeOutUs  接收超时，微秒
         * @param lowLatency 是否启用 USB low_latency 标志
         */
        SerialPort(const std::string &portName,
                   size_t recvLength = 16,
                   uint32_t baudrate = 4000000,
                   size_t timeOutUs = 20000,
                   bool lowLatency = true);

        ~SerialPort();

        /// 重新配置串口参数（会关闭再重新打开）
        void resetSerial(size_t recvLength = 16,
                         uint32_t baudrate = 4000000,
                         size_t timeOutUs = 20000,
                         bool lowLatency = true);

        // ---- 原始字节 I/O ----
        size_t send(uint8_t *sendMsg, size_t sendLength);
        size_t recv(uint8_t *recvMsg, size_t recvLength);
        size_t recv(uint8_t *recvMsg);

        // ---- 高层 send/recv ----
        bool sendRecv(uint8_t *sendMsg, uint8_t *recvMsg, size_t sendLength);
        bool sendRecv(MotorCmd *sendMsg, MotorData *recvMsg);
        bool sendRecv(std::vector<MotorCmd> &sendVec, std::vector<MotorData> &recvVec);

      private:
        // ---- USB low_latency ----
        void apply_low_latency();

        // ---- 串口配置 ----
        void configure_port(uint32_t baudrate);

        boost::asio::io_context io_;
        boost::asio::serial_port serial_;
        std::string portName_;
        size_t recvLength_;
        size_t timeOutUs_;
        bool lowLatency_;
    };
} // namespace Dog8Dof
