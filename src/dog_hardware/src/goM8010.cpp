#include "dog_hardware/goM8010.hpp"
#include "dog_hardware/serial.hpp"
#include "unitree_actuator_sdk/unitreeMotor/unitreeMotor.h"
#include <cstdint>
#include <ios>
#include <memory>
#include <rclcpp/utilities.hpp>

namespace Dog8Dof {

    goM8010Interface::goM8010Interface(uint8_t ID,
                                       std::shared_ptr<Dog8Dof::SerialPort> port) {
        command_.motorType = MotorType::GO_M8010_6;
        state_.motorType = MotorType::GO_M8010_6;
        command_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        command_.id = ID;

        posOffset_ = 0.0;
        port_ = port;

        if (!this->zeroTor()) {
            throw std::ios_base::failure("Motor " + std::to_string(ID) + " time out");
        }
    }

    bool
    goM8010Interface::setHybrid(float torque, float pos, float vel, float kp, float kd) {
        command_.tau = torque / gearRatio_;
        command_.q = (pos + posOffset_) * gearRatio_;
        command_.dq = vel * gearRatio_;
        command_.kp = kp;
        command_.kd = kd;
        return port_->sendRecv(&command_, &state_);
    }

    bool goM8010Interface::setPos(float target, float kp, float kd) {
        command_.tau = 0.0;
        command_.q = (target + posOffset_) * gearRatio_;
        command_.dq = 0.0;
        command_.kp = kp;
        command_.kd = kd;
        return port_->sendRecv(&command_, &state_);
    }

    bool goM8010Interface::setVel(float target, float kd) {
        command_.tau = 0.0;
        command_.q = 0.0;
        command_.dq = target * gearRatio_;
        command_.kp = 0.0;
        command_.kd = kd;
        return port_->sendRecv(&command_, &state_);
    }

    bool goM8010Interface::setTor(float target) {
        command_.tau = target / gearRatio_;
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

    bool goM8010Interface::lock() {
        command_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
        return port_->sendRecv(&command_, &state_);
    }

    bool goM8010Interface::release() {
        command_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        return this->zeroTor();
    }

    void goM8010Interface::emergencyStop() {
        command_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
        command_.id = 15;
        port_->sendRecv(&command_, &state_);
    }

} // namespace Dog8Dof