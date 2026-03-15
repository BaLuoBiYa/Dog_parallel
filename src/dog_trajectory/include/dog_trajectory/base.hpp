#pragma once
#include "dog_tools/type.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace Dog8Dof {

    class BaseTrajectory {
      public:
        BaseTrajectory(const std::string &configPath)
            : configPath_(configPath), duration_(rclcpp::Duration::from_seconds(0.001)) {}
        virtual ~BaseTrajectory() = default;

        /**
         * @brief 规划下一时刻足尖状态
         * @param currentTip 当前足尖状态
         */
        virtual TrajectoryPoint generate(const rclcpp::Time currentTime, const Body &currentTip) = 0;

        /**
         * @brief 当进入一个新的轨迹状态时调用的虚函数
         * @param currentTip 包含当前关节状态，用于初始化
         * @param currentTime 轨迹起始时间
         * @details 派生类可以重写此函数以实现特定的状态进入逻辑，
         *          例如重置内部状态机或记录初始位置。
         */
        virtual bool start(const rclcpp::Time currentTime, const Body &currentTip) = 0;

        /**
         * @brief 从配置文件重新读出参数
         * @return 是否成功更新
         */
        virtual bool reloadParameters() = 0;

        /**
         * @brief 检查轨迹是否已自行完成
         * @return 如果轨迹已完成，则返回 true
         */
        bool finished() { return finished_; }

      protected:
        const std::string configPath_;
        YAML::Node configFile_;
        // std::map<std::string, float> parameters;
        // std::vector<std::string> parameterNames_;

        rclcpp::Time startTime_;
        rclcpp::Duration duration_;

        bool finished_ = false;
        TrajectoryPoint target_ = {};
    };

} // namespace Dog8Dof