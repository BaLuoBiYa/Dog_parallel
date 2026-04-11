#include "dog_remote/sbusReader.hpp"
#include <chrono>
#include <iostream>

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    try {
        Dog8Dof::SbusReader reader("/dev/ttyUSB0", 100000);

        std::chrono::steady_clock clock;
        std::chrono::time_point<std::chrono::steady_clock> start;
        std::chrono::time_point<std::chrono::steady_clock> end;
        while (true) {
            end = clock.now();
            if (!reader.hasFreshFrame()) {
                std::cout << "waiting SBUS frame..." << std::endl;
                continue;
            }
            end = clock.now();
            if (!reader.hasFreshFrame()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            start = clock.now();
            auto channels = reader.getAllChannelValues();

            static bool first = true;
            // 每次更新前定位到 block 起始行（第一次直接打印整个 block）
            if (!first) {
                // 向上移动 17 行到 block 顶部
                std::cout << "\033[17A";
            }

            for (size_t i = 0; i < channels.size(); ++i) {
                std::cout << "\r\033[2K" << "CH" << i << "=" << channels[i] << std::endl;
            }
            auto dur = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            std::cout << "\r\033[2K" << "Duration=" << dur.count() << std::endl;
            std::cout << std::flush;
            first = false;
        }
    } catch (const std::exception &e) {
        std::cerr << "SBUSReader exception: " << e.what() << std::endl;
        return 1;
    }
}