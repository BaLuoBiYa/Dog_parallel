#include "dog_remote/sbusReader.hpp"
#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    try {
        Dog8Dof::SbusDecoder decoder("/dev/ttyUSB0", 100000);

        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            if (!decoder.hasFreshFrame()) {
                std::cout << "waiting SBUS frame..." << std::endl;
                continue;
            }

            auto channels = decoder.getAllChannelValues();
            std::cout << "CH0=" << channels[0] << " CH1=" << channels[1]
                      << " CH2=" << channels[2] << " CH3=" << channels[3] << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << "SBUSDecoder exception: " << e.what() << std::endl;
        return 1;
    }
}