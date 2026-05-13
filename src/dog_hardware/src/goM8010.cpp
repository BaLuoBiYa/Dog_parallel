#include "dog_hardware/goM8010.hpp"
#include "unitree_actuator_sdk/serialPort/SerialPort.h"
#include "unitree_actuator_sdk/unitreeMotor/unitreeMotor.h"
#include <cstdint>
#include <ios>
#include <memory>
#include <rclcpp/utilities.hpp>

namespace Dog8Dof {

    goM8010Interface::goM8010Interface(uint8_t ID, std::shared_ptr<SerialPort> port) {
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

    bool goM8010Interface::sendRecvRaw() {
        command_.modify_data(&command_);
        uint8_t *sendData = command_.get_motor_send_data();
        uint8_t *recvData = state_.get_motor_recv_data();
        bool ok =
            port_->sendRecv(sendData, recvData, static_cast<size_t>(command_.hex_len));
        if (ok) {
            state_.extract_data(&state_);
        }
        return ok;
    }

    bool
    goM8010Interface::setHybrid(float torque, float pos, float vel, float kp, float kd) {
        command_.tau = torque / gearRatio_;
        command_.q = (pos + posOffset_) * gearRatio_;
        command_.dq = vel * gearRatio_;
        command_.kp = kp;
        command_.kd = kd;
        return sendRecvRaw();
    }

    bool goM8010Interface::setPos(float target, float kp, float kd) {
        command_.tau = 0.0;
        command_.q = (target + posOffset_) * gearRatio_;
        command_.dq = 0.0;
        command_.kp = kp;
        command_.kd = kd;
        return sendRecvRaw();
    }

    bool goM8010Interface::setVel(float target, float kd) {
        command_.tau = 0.0;
        command_.q = 0.0;
        command_.dq = target * gearRatio_;
        command_.kp = 0.0;
        command_.kd = kd;
        return sendRecvRaw();
    }

    bool goM8010Interface::setTor(float target) {
        command_.tau = target / gearRatio_;
        command_.q = 0.0;
        command_.dq = 0.0;
        command_.kp = 0.0;
        command_.kd = 0.0;
        return sendRecvRaw();
    }

    MotorCmd goM8010Interface::makeCmd(
        float torque, float pos, float vel, float kp, float kd) const {
        MotorCmd cmd;
        cmd.motorType = MotorType::GO_M8010_6;
        cmd.hex_len = 0;
        cmd.id = command_.id;
        cmd.mode = command_.mode;
        cmd.tau = torque / gearRatio_;
        cmd.q = (pos + posOffset_) * gearRatio_;
        cmd.dq = vel * gearRatio_;
        cmd.kp = kp;
        cmd.kd = kd;
        return cmd;
    }

    void goM8010Interface::applyState(const MotorData &data) {
        state_ = data;
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

    float goM8010Interface::getPos() {
        return state_.q / gearRatio_ - posOffset_;
    }

    float goM8010Interface::getVel() {
        return state_.dq / gearRatio_;
    }

    float goM8010Interface::getTor() {
        return state_.tau * gearRatio_;
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
            return "OK";
        }
    }

    void goM8010Interface::emergencyStop() {
        command_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
        command_.id = 15;
        port_->sendRecv(&command_, &state_);
    }

} // namespace Dog8Dof