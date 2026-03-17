#include "dog_tools/type.hpp"
#include <Eigen/Dense>
namespace Dog8Dof {
    class CubicBezier {
        using Point2D = Eigen::Vector2<float>;

      public:
        static Trajectory generate(Point2D startPos,
                                   Point2D finishPos,
                                   Point2D startVel,
                                   Point2D finishVel,
                                   float time,
                                   size_t steps,
                                   float k = (1.0f / 3.0f)) {
            Trajectory result(steps);

            // 计算四个控制点
            const Point2D C0 = startPos;
            const Point2D C1 = startPos + startVel * (time * k);
            const Point2D C2 = finishPos - finishVel * (time * k);
            const Point2D C3 = finishPos;

            // 计算离散贝塞尔点
            auto bezierPoint = [C0, C1, C2, C3](double t) -> Point2D {
                const double t2 = t * t;
                const double t3 = t2 * t;
                const double u = 1.0 - t;
                const double u2 = u * u;
                const double u3 = u2 * u;

                return C0 * u3 + C1 * (3.0 * u2 * t) + C2 * (3.0 * u * t2) + C3 * t3;
            };

            // 计算离散贝塞尔速度
            auto bezierVel = [C0, C1, C2, C3](double t) -> Point2D {
                const double t2 = t * t;
                const double u = 1.0 - t;
                const double u2 = u * u;

                return (C1 - C0) * (3.0 * u2) + (C2 - C1) * (6.0 * u * t) + (C3 - C2) * (3.0 * t2);
            };

            for (size_t i = 0; i < steps; i++) {
                double t = static_cast<double>(i) / (steps - 1);
                Point2D pos = bezierPoint(t);
                result.points[i].pos[0] = pos[0];
                result.points[i].pos[1] = 0.0;
                result.points[i].pos[2] = pos[1];
                Point2D vel = bezierVel(t);
                result.points[i].vel[0] = vel[0] / time;
                result.points[i].vel[1] = 0.0;
                result.points[i].vel[2] = vel[1] / time;
            }
            return result;
        }
    };

} // namespace Dog8Dof