#include "dog_trajectory/base.hpp"

namespace Dog8Dof {
    void BaseTrajectory::interpolate(size_t currentNanoSec) {
        const size_t n = trajectory_.time.size();
        if (n == 0)
            return;
        if (trajectory_.progress >= n)
            trajectory_.progress = n - 1;

        const auto searchBegin = trajectory_.time.begin() + trajectory_.progress;
        const auto upperIt =
            std::lower_bound(searchBegin, trajectory_.time.end(), currentNanoSec);

        const size_t upperIndex = static_cast<size_t>(upperIt - trajectory_.time.begin());
        if (upperIndex == 0) {
            target_ = trajectory_.points[0];
            trajectory_.progress = 0;
            return;
        }

        const size_t lowerIndex = upperIndex - 1;

        const size_t t0 = trajectory_.time[lowerIndex];
        const size_t t1 = trajectory_.time[upperIndex];

        float alpha = 0.0f;
        if (t1 > t0) {
            alpha = static_cast<float>(currentNanoSec - t0) / static_cast<float>(t1 - t0);
        }
        alpha = std::clamp(alpha, 0.0f, 1.0f);

        const TrajectoryPoint &p0 = trajectory_.points[lowerIndex];
        const TrajectoryPoint &p1 = trajectory_.points[upperIndex];

        target_.pos = p0.pos + (p1.pos - p0.pos) * alpha;
        target_.vel = p0.vel + (p1.vel - p0.vel) * alpha;
        target_.tau = p0.tau + (p1.tau - p0.tau) * alpha;

        trajectory_.progress = upperIndex;
    }

    Eigen::Vector2<float>
    BaseTrajectory::getHeightVec(const Eigen::Vector2<float> &tangentVec) {
        if (tangentVec.squaredNorm() <= 1e-10) {
            return Eigen::Vector2f(0.0f, -0.1f);
        }

        const Eigen::Vector2<float> normVec0(-tangentVec[1], tangentVec[0]);
        const Eigen::Vector2<float> normVec1(tangentVec[1], -tangentVec[0]);
        // 计算起始向量的两个法向量

        Eigen::Vector2<float> directionVec;
        if (normVec0[1] <= 0) {
            directionVec = normVec0.normalized();
        } else {
            directionVec = normVec1.normalized();
        }
        // 朝向Z负方向（朝向天空的向量作为运动方向）

        return directionVec;
    }

    size_t BaseTrajectory::stepsFromDurationNs(size_t durationNs,
                                               size_t minSteps,
                                               size_t maxSteps) const {
        if (controlDurationNanoSec_ == 0) {
            return minSteps;
        }

        // 四舍五入到最近控制周期
        size_t steps =
            (durationNs + controlDurationNanoSec_ / 2) / controlDurationNanoSec_;
        steps = std::max(steps, minSteps);
        steps = std::min(steps, maxSteps);
        return steps;
    }
} // namespace Dog8Dof