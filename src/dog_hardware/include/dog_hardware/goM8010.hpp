#include "unitree_actuator_sdk/serialPort/SerialPort.h"
#include "unitree_actuator_sdk/unitreeMotor/unitreeMotor.h"
#include <cstdint>
#include <memory>
#include <string>

namespace Dog_Hardware {

    /**
     * @brief 封装了 Unitree M8010 电机的基本控制接口
     * @note 本类中所有的输入输出均为实际输出轴数据，电机默认为零力矩模式
     */
    class goM8010Interface {
      public:
        /**
         * @brief 构造函数，设定电机的 ID
         * @param ID 电机的唯一标识
         * @param port 串口实例的共享指针
         */
        goM8010Interface(uint8_t ID, std::shared_ptr<SerialPort> port);

        /**
         * @brief 混合控制模式
         * @param torque 发送的力矩
         * @param pos 目标位置
         * @param vel 目标速度
         * @param kp 位置环增益
         * @param kd 速度环增益
         */
        bool setHybrid(float torque, float pos, float vel, float kp, float kd);

        /**
         * @brief 位置闭环控制
         * @param target 目标位置
         * @param kp 位置环增益
         * @param kd 速度环增益
         */
        bool setPos(float target, float kp, float kd);

        /**
         * @brief 速度闭环控制
         * @param target 目标速度
         * @param kd 速度环增益
         */
        bool setVel(float target, float kd);

        /**
         * @brief 直接设定电机输出扭矩
         * @param target 目标扭矩
         */
        bool setTor(float target);

        /**
         * @brief 设定零位偏移
         * @note 设定当前位置为电机零位
         */
        void setZero();

        /**
         * @brief 设置电机为无摩擦模式
         */
        bool zeroTor();

        /**
         * @brief 设置电机为阻尼模式
         */
        bool damp(float kd);

        /**
         * @brief 锁定电机
         */
        bool stop();

        /**
         * @brief 获得当前电机位置
         */
        float getPos();

        /**
         * @brief 获得当前电机速度
         */
        float getVel();

        /**
         * @brief 获得当前电机扭矩
         */
        float getTor();

        /**
         * @brief 获得当前电机扭矩
         */
        uint8_t getID();

        /**
         * @brief 获得当前电机温度
         * @note 温度高于90度会触发保护无法控制
         */
        int8_t getTemp();

        /**
         * @brief 获得当前电机问题
         */
        std::string getError();

        /**
         * @brief 紧急停止所有电机
         */
        void emergencyStop();

      private:
        MotorCmd command_;
        MotorData state_;
        std::shared_ptr<SerialPort> port_;
        float posOffset_;
    };

} // namespace Dog_Hardware