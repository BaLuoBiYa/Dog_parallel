#include "dog_hardware/goM8010.hpp"
#include "unitree_actuator_sdk/serialPort/SerialPort.h"
#include "unitree_actuator_sdk/unitreeMotor/unitreeMotor.h"
#include <chrono>
#include <cstdint>
#include <ios>
#include <memory>
#include <rclcpp/utilities.hpp>
#include <thread>

namespace Dog8Dof {

    goM8010Interface::goM8010Interface(uint8_t ID, std::shared_ptr<SerialPort> port) {
        command_.motorType = MotorType::GO_M8010_6;
        state_.motorType = MotorType::GO_M8010_6;
        command_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        command_.id = ID;

        posOffset_ = 0.0;
        port_ = port;

        if (!this->zeroTor()) {
            throw std::ios_base::failure("Motor" + std::to_string(ID) + "time out");
        }
    }

    bool
    goM8010Interface::setHybrid(float torque, float pos, float vel, float kp, float kd) {
        command_.tau = torque / queryGearRatio(MotorType::GO_M8010_6);
        command_.q = (pos + posOffset_) * queryGearRatio(MotorType::GO_M8010_6);
        command_.dq = vel * queryGearRatio(MotorType::GO_M8010_6);
        command_.kp = kp;
        command_.kd = kd;
        return port_->sendRecv(&command_, &state_);
    }

    bool goM8010Interface::setPos(float target, float kp, float kd) {
        command_.tau = 0.0;
        command_.q = (target + posOffset_) * queryGearRatio(MotorType::GO_M8010_6);
        command_.dq = 0.0;
        command_.kp = kp;
        command_.kd = kd;
        return port_->sendRecv(&command_, &state_);
    }

    bool goM8010Interface::setVel(float target, float kd) {
        command_.tau = 0.0;
        command_.q = 0.0;
        command_.dq = target * queryGearRatio(MotorType::GO_M8010_6);
        command_.kp = 0.0;
        command_.kd = kd;
        return port_->sendRecv(&command_, &state_);
    }

    bool goM8010Interface::setTor(float target) {
        command_.tau = target / queryGearRatio(MotorType::GO_M8010_6);
        command_.q = 0.0;
        command_.dq = 0.0;
        command_.kp = 0.0;
        command_.kd = 0.0;
        return port_->sendRecv(&command_, &state_);
    }

    void goM8010Interface::setZero(float offset) {
        posOffset_ = offset;
    }

    bool goM8010Interface::zeroTor() {
        command_.tau = 0.0;
        command_.q = 0.0;
        command_.dq = 0.0;
        command_.kp = 0.0;
        command_.kd = 0.0;
        return port_->sendRecv(&command_, &state_);
    }

    bool goM8010Interface::damp(float kd) {
        command_.tau = 0.0;
        command_.q = 0.0;
        command_.dq = 0.0;
        command_.kp = 0.0;
        command_.kd = kd;
        return port_->sendRecv(&command_, &state_);
    }

    bool goM8010Interface::stop() {
        command_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
        return port_->sendRecv(&command_, &state_);
    }

    float goM8010Interface::getPos() {
        return state_.q / queryGearRatio(MotorType::GO_M8010_6) - posOffset_;
    }

    float goM8010Interface::getVel() {
        return state_.dq / queryGearRatio(MotorType::GO_M8010_6);
    }

    float goM8010Interface::getTor() {
        return state_.tau * queryGearRatio(MotorType::GO_M8010_6);
    }

    int8_t goM8010Interface::getTemp() {
        return state_.temp;
    }

    uint8_t goM8010Interface::getID() {
        return state_.motor_id;
    }

    std::string goM8010Interface::getError() {
        switch (state_.merror) {
        case 0:
            return "OK";
        case 1:
            return "Over Heat";
        case 2:
            return "Over Current";
        case 3:
            return "Over Voltage";
        case 4:
            return "Encoder Error";
        case 5:
            return "Under Voltage";
        case 6:
            return "Over Heat on Coil";
        default:
            return "Unknow Error";
        }
    }

    void goM8010Interface::emergencyStop() {
        command_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
        command_.id = 15;
        port_->sendRecv(&command_, &state_);
    }

} // namespace Dog8Dof