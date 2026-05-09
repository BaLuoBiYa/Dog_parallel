#include <atomic>
#include <chrono>
#include <csignal>
#include <dog_hardware/CanManager.hpp>
#include <iomanip>
#include <iostream>
#include <thread>

std::atomic<bool> keep_running(true);

void signal_handler(int) {
    keep_running = false;
}

int main(int argc, char **argv) {
    // 如果运行时带了参数（如 ros2 run dog_remote canTester can1），则使用参数，否则默认
    // can0
    std::string can_iface = (argc > 1) ? argv[1] : "can0";
    std::cout << "Starting CAN Tester on interface: " << can_iface << std::endl;
    std::cout << "Press Ctrl+C to stop..." << std::endl;

    std::signal(SIGINT, signal_handler);

    try {
        CanManager can_manager(can_iface);
        std::cout << "Successfully opened " << can_iface << std::endl;

        auto last_send_time = std::chrono::steady_clock::now();
        unsigned char count = 0;

        while (keep_running) {
            auto current_time = std::chrono::steady_clock::now();

            // 1. 测试发送：大疆 M2006 (C610电调) 控制报文
            // C610电调要求较高频率下发命令（比如1ms~10ms），这里使用 10ms(100Hz) 发送一次
            if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time -
                                                                      last_send_time)
                    .count() >= 10) {
                // DLC = 8，每两字节（高在前、低在后）代表一个电机的命令（电流值）
                // 测试连接建议直接发全0，这会让电机保持原位不转，但 C610
                // 只要收到数据，就会稳定回传反馈（0x201）
                unsigned char test_data[8] = {
                    0x00, 0xF0, 0x00, 0xF0, 0x00, 0xF0, 0x00, 0xF0};

                if (can_manager.CanSend(0x1FF, 8, test_data) &&
                    can_manager.CanSend(0x200, 8, test_data)) {
                    // 控制台不用每次 10ms 都打印，不然就刷屏了。1秒打印1次发送成功的提示
                    if (count % 100 == 0) {
                        std::cout
                            << "[TX] Sending DJI Control (ID: 0x200), Safe Current (0A)."
                            << std::endl;
                    }
                } else {
                    std::cerr << "[TX Error] Failed to send DJI frame!" << std::endl;
                }
                count++;
                last_send_time = current_time;
            }

            // 2. 测试接收：持续非阻塞地读取总线上的报文并打印
            struct can_frame rx_frame;
            if (can_manager.CanRead(rx_frame)) {
                std::cout << "[RX] ID: 0x" << std::hex << std::uppercase << std::setw(3)
                          << std::setfill('0') << rx_frame.can_id
                          << " | DLC: " << std::dec << (int)rx_frame.can_dlc
                          << " | Data: ";
                for (int i = 0; i < rx_frame.can_dlc; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0')
                              << (int)rx_frame.data[i] << " ";
                }
                std::cout << std::dec << std::endl;
            } else {
                // 没有读取到数据时休眠一小会儿（1ms），防止 while(true) 把 CPU 占满到
                // 100%
                // std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "\nInitialization failed: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
