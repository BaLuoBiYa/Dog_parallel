#include "unitree_actuator_sdk/serialPort/SerialPort.h"
#include "unitree_actuator_sdk/unitreeMotor/unitreeMotor.h"
#include <chrono>
#include <iostream>
#include <unistd.h>

int main() {

    SerialPort serial("/dev/ttyACM0");
    MotorCmd cmd;
    MotorData data;
    uint64_t loop = 0;

    while (true) {
        cmd.motorType = MotorType::GO_M8010_6;
        data.motorType = MotorType::GO_M8010_6;
        cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        cmd.id = 1;
        cmd.kp = 0.0;
        cmd.kd = 0.01;
        cmd.q = 0.0;
        // cmd.dq = -6.28 * queryGearRatio(MotorType::GO_M8010_6);
        cmd.dq = 0.0;
        cmd.tau = 0.0;

        auto t0 = std::chrono::steady_clock::now();
        serial.sendRecv(&cmd, &data);
        auto t1 = std::chrono::steady_clock::now();
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        std::cout << "[" << loop << "] sendRecv: " << us << " us" << " | q=" << data.q
                  << " dq=" << data.dq << " temp=" << static_cast<int>(data.temp)
                  << " err=" << static_cast<int>(data.merror)
                  << " force=" << data.footForce << std::endl;

        ++loop;
        usleep(200);
    }
}