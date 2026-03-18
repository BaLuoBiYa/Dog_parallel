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
            : configPath_(configPath), durationNanoSec_(period.nanoseconds()) {
        }
        virtual ~BaseTrajectory() = default;

        /**
         * @brief 以当前状态生成下一个步态周期内的步态序列
         * @param currentTip 包含当前关节状态，用于初始化
         */
        virtual bool generate(const Body &currentTip) = 0;

        /**
         * @brief 初始化轨迹开始时间
         * @param startTime 轨迹开始执行时间
         * @warning 调用该方法后，在轨迹结束前不应该当使用耗时较长的函数，避免拖慢控制回路
         */
        virtual bool start(const rclcpp::Time &startTime) = 0;

        /**
         * @brief 获得下一时刻足尖状态
         * @param currentTime 当前时间
         */
        virtual TrajectoryPoint get(const rclcpp::Time &currentTime) = 0;

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

        size_t startTimeNanoSec_;
        const size_t durationNanoSec_;

        bool finished_ = false;
        int steps_;
        Trajectory trajectory_;
        TrajectoryPoint target_;

        /**
         * @brief   线性插值函数，赋值target的位置，速度和力属性
         * @param   currrentNanoSec 当前纳秒时间戳
         * @note    需要接下来手动实现增益的赋值
         */
        void interpolate(size_t currentNanoSec);

        /**
         * @brief   获得指向步高方向的单位向量
         * @param   tangentVec 起点坐标
         */
        Eigen::Vector2<float> getHeightVec(const Eigen::Vector2<float> &tangentVec);

        /**
         * @brief   根据步态周期生成步数
         * @param   durationNs 步态持续时长
         */
        size_t stepsFromDurationNs(size_t durationNs,
                                   size_t minSteps = Const::MIN_RESOLUTION,
                                   size_t maxSteps = Const::MAX_RESOLUTION) const;
    };

} // namespace Dog8Dof