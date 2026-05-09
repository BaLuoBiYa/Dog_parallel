#include <chrono>
#include <dog_hardware/dm_motor.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace Dog8Dof;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::string can_iface = "can0"; // 根据实际情况修改CAN接口

    // 初始化电机参数 (以常用的参数为例)
    DM_Motor motor(can_iface, 0x00, 0x01, 12.5f, 30.0f, 10.0f, 20.0f);

    std::cout << "DM_Motor Initialized on " << can_iface << std::endl;

    std::cout << "Testing Output Setup..." << std::endl;
    // std::cout << "Enabling Motor..." << std::endl;
    motor.Motor_Enable();

    // 使用 rclcpp::ok() 代替 1，这样就能捕获 Ctrl+C (SIGINT)
    while (rclcpp::ok()) {
        motor.Output(
            5.0f, 5.0f, 0.1f, 0.0f, 5.0f); // 需要实际CAN设备支持，如未接硬件可能报错
        motor.Data_Process();              // 依赖底层接收逻辑，如未接硬件可能报错

        std::cout << motor.Rx_Data.Now_Angle << std::endl;
        std::cout << motor.Rx_Data.Now_Omega << std::endl;
        std::cout << motor.Rx_Data.Now_Torque << std::endl;
        std::cout << motor.Rx_Data.Now_MOS_Temperature << std::endl;
        std::cout << motor.Rx_Data.Now_Rotor_Temperature << std::endl;

        // 避免死循环跑满 CPU，增加少量延时
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Ctrl+C detected, exiting..." << std::endl;
    // motor.Motor_Disable(); // 退出前可以加上失能电机的代码

    // 简单测试数据处理
    std::cout << "Testing Data Process..." << std::endl;
    // motor.Data_Process(); // 依赖底层接收逻辑

    std::cout << "Test completed (Simulated run). " << std::endl;
    rclcpp::shutdown();
    return 0;
}
