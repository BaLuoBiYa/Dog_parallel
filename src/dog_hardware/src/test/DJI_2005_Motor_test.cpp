#include <chrono>
#include <dog_hardware/dji_motor.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace Dog8Dof;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::string can_iface = "can0"; // 根据实际情况修改CAN接口

    std::cout << "Initializing DJI_Motor_2006 on " << can_iface << "..." << std::endl;

    // 参数：接口, 接收ID(0x201), 控制模式(速度模式), 减速比(大疆2006默认约36),
    // 最大电流(10A) 根据实际情况修改
    DJI_Motor_2006 motor1(
        can_iface, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_OMEGA, 36.0f, 10.0f);
    motor1.Target_Omega = 2.0f; // 设置目标速度为5 rad/s，实际测试时可调整
    DJI_Motor_2006 motor2(
        can_iface, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_OMEGA, 36.0f, 10.0f);
    motor2.Target_Omega = 2.0f; // 设置目标速度为5 rad/s，实际测试时可调整
    DJI_Motor_2006 motor3(
        can_iface, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_OMEGA, 36.0f, 10.0f);
    motor3.Target_Omega = 2.0f; // 设置目标速度为5 rad/s，实际测试时可调整
    DJI_Motor_2006 motor4(
        can_iface, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_OMEGA, 36.0f, 10.0f);
    motor4.Target_Omega = 2.0f; // 设置目标速度为5 rad/s，实际测试时可调整

    std::cout << "Initialization completed." << std::endl;
    std::cout << "Running Motor Control Loop..." << std::endl;

    // 使用 rclcpp::ok() 以响应 Ctrl+C 退出
    while (rclcpp::ok()) {
        // 调用定时计算与输出，由于内部状态如 Rx_Data/Target_Omega 是 private，
        // 这里只是做流程上的循环调用，防止编译报错。后续若需控制，可在电机库提供 public
        // Set_Target() 等。
        motor1.TIM_Calculate_PeriodElapsedCallback();
        motor2.TIM_Calculate_PeriodElapsedCallback();
        motor3.TIM_Calculate_PeriodElapsedCallback();
        motor4.TIM_Calculate_PeriodElapsedCallback();
        // 避免满载 CPU

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "\nCtrl+C detected, exiting DJI Motor Test..." << std::endl;

    rclcpp::shutdown();
    return 0;
}
