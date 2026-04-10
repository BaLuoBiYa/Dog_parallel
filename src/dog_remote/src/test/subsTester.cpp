#include "dog_remote/sbusReader.hpp"
#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    try {
        Dog8Dof::SbusReader reader("/dev/ttyVIR1", 100000);

        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            if (!reader.hasFreshFrame()) {
                std::cout << "waiting SBUS frame..." << std::endl;
                continue;
            }

            auto channels = reader.getAllChannelValues();
            std::cout << "CH0=" << channels[0] << std::endl
                      << "CH1=" << channels[1] << std::endl
                      << "CH2=" << channels[2] << std::endl
                      << "CH3=" << channels[3] << std::endl
                      << "CH4=" << channels[4] << std::endl
                      << "CH5=" << channels[5] << std::endl
                      << "CH6=" << channels[6] << std::endl
                      << "CH7=" << channels[7] << std::endl
                      << "CH8=" << channels[8] << std::endl
                      << "CH9=" << channels[9] << std::endl
                      << "CH10=" << channels[10] << std::endl
                      << "CH11=" << channels[11] << std::endl
                      << "CH12=" << channels[12] << std::endl
                      << "CH13=" << channels[13] << std::endl
                      << "CH14=" << channels[14] << std::endl
                      << "CH15=" << channels[15] << std::endl
                      << "-----------------------------" << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << "SBUSReader exception: " << e.what() << std::endl;
        return 1;
    }
}