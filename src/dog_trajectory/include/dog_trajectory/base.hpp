#pragma once

#include "dog_tools/const.hpp"
#include "dog_tools/type.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace Dog8Dof {

    class BaseTrajectory {
      public:
        /**
         * @brief 轨迹生成基类
         * @param config YAML格式配置文件路径，存储参数
         * @param period 控制时间周期
         */
        BaseTrajectory(const std::string &configPath, rclcpp::Duration period)
            : configPath_(configPath), controlDurationNanoSec_(period.nanoseconds()) {
        }
        virtual ~BaseTrajectory() = default;

        /**
         * @brief 获取当前时刻的期望轨迹点（每控制周期调用，内部在线重规划）
         * @param currentTime 当前 ROS 时间
         * @param currentTip  当前足端传感器反馈（可空，则用轨迹内推状态）
         * @return 本周期期望的足端状态（位置、速度、PD 增益）
         */
        virtual TrajectoryPoint get(const rclcpp::Time &currentTime,
                                    const Body *currentTip = nullptr);

        /**
         * @brief 启动轨迹生成器，记录起始时间
         * @param startTime 轨迹开始时间
         * @return 是否成功启动
         */
        void start(rclcpp::Time startTime) {
            startTimeNanoSec_ = startTime.nanoseconds();
            finished_ = false;
        }

        /**
         * @brief 从配置文件重新读出参数
         * @return 是否成功更新
         */
        virtual bool reloadParameters() = 0;

        /**
         * @brief 检查轨迹是否已自行完成
         * @return 如果轨迹已完成，则返回 true
         */
        bool finished() {
            return finished_;
        }

      protected:
        const std::string configPath_;
        YAML::Node configFile_;

        long startTimeNanoSec_;
        const long controlDurationNanoSec_;

        bool finished_ = false;
    };

} // namespace Dog8Dof