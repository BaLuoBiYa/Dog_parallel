#pragma once
#include "dog_tools/type.hpp"
#include <Eigen/Dense>
#include <cstddef>

namespace Dog8Dof {
    class QuinticPolynomial {
        using Point2D = Eigen::Vector2<float>;

      public:
        static Trajectory generate(const Point2D &startPos,
                                   const Point2D &finishPos,
                                   const Point2D &startVel,
                                   const Point2D &finishVel,
                                   const Point2D &startAcc,
                                   const Point2D &finishAcc,
                                   float time,
                                   size_t steps) {
            Trajectory result(steps);
            if (steps == 0 || time <= 0.0f) {
                return result;
            }

            const float T = time;
            const float T2 = T * T;
            const float T3 = T2 * T;
            const float T4 = T3 * T;
            const float T5 = T4 * T;

            Eigen::Matrix3f A;
            A << T3, T4, T5, 3.0f * T2, 4.0f * T3, 5.0f * T4, 6.0f * T, 12.0f * T2,
                20.0f * T3;

            auto solveOneAxis =
                [&](float p0, float v0, float acc0, float pT, float vT, float accT) {
                    const float a0 = p0;
                    const float a1 = v0;
                    const float a2 = 0.5f * acc0;

                    Eigen::Vector3f b;
                    b << pT - (a0 + a1 * T + a2 * T2), vT - (a1 + 2.0f * a2 * T),
                        accT - (2.0f * a2);

                    const Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);

                    Eigen::Matrix<float, 6, 1> coeff;
                    coeff << a0, a1, a2, x[0], x[1], x[2];
                    return coeff;
                };

            const Eigen::Matrix<float, 6, 1> cx = solveOneAxis(startPos[0],
                                                               startVel[0],
                                                               startAcc[0],
                                                               finishPos[0],
                                                               finishVel[0],
                                                               finishAcc[0]);
            const Eigen::Matrix<float, 6, 1> cz = solveOneAxis(startPos[1],
                                                               startVel[1],
                                                               startAcc[1],
                                                               finishPos[1],
                                                               finishVel[1],
                                                               finishAcc[1]);

            for (size_t i = 0; i < steps; ++i) {
                const float s =
                    (steps == 1) ? 0.0f
                                 : static_cast<float>(i) / static_cast<float>(steps - 1);
                const float t = s * T;
                const float t2 = t * t;
                const float t3 = t2 * t;
                const float t4 = t3 * t;
                const float t5 = t4 * t;

                const float px =
                    cx[0] + cx[1] * t + cx[2] * t2 + cx[3] * t3 + cx[4] * t4 + cx[5] * t5;
                const float pz =
                    cz[0] + cz[1] * t + cz[2] * t2 + cz[3] * t3 + cz[4] * t4 + cz[5] * t5;

                const float vx = cx[1] + 2.0f * cx[2] * t + 3.0f * cx[3] * t2 +
                                 4.0f * cx[4] * t3 + 5.0f * cx[5] * t4;
                const float vz = cz[1] + 2.0f * cz[2] * t + 3.0f * cz[3] * t2 +
                                 4.0f * cz[4] * t3 + 5.0f * cz[5] * t4;

                result.points[i].pos[0] = px;
                result.points[i].pos[1] = 0.0f;
                result.points[i].pos[2] = pz;

                result.points[i].vel[0] = vx;
                result.points[i].vel[1] = 0.0f;
                result.points[i].vel[2] = vz;

                result.points[i].tau.setZero();
            }

            return result;
        }

        static Eigen::Vector2f solveMidVel2D(const Eigen::Vector2f &startPos,
                                             const Eigen::Vector2f &midPos,
                                             const Eigen::Vector2f &finishPos,
                                             const Eigen::Vector2f &startVel,
                                             const Eigen::Vector2f &finishVel,
                                             float T1,
                                             float T2,
                                             float maxNorm = -1.0f) {
            constexpr float eps = 1e-6f;

            Eigen::Vector2f vm = 0.5f * (startVel + finishVel);

            if (T1 > eps && T2 > eps) {
                const Eigen::Vector2f d1 = midPos - startPos;
                const Eigen::Vector2f d2 = finishPos - midPos;

                vm = (6.0f * (d1 / (T1 * T1) + d2 / (T2 * T2)) -
                      2.0f * (startVel / T1 + finishVel / T2)) /
                     (4.0f * (1.0f / T1 + 1.0f / T2));
            }

            if (maxNorm > eps) {
                const float n = vm.norm();
                if (n > maxNorm) {
                    vm *= (maxNorm / n);
                }
            }

            return vm;
        }

        static Eigen::Vector2f solveMidAcc2D(const Eigen::Vector2f &startPos,
                                             const Eigen::Vector2f &startVel,
                                             const Eigen::Vector2f &startAcc,
                                             const Eigen::Vector2f &midPos,
                                             const Eigen::Vector2f &midVel,
                                             const Eigen::Vector2f &finishPos,
                                             const Eigen::Vector2f &finishVel,
                                             const Eigen::Vector2f &finishAcc,
                                             float T1,
                                             float T2) {

            Eigen::Vector2f am;
            am[0] = solveMidAcc1D(startPos[0],
                                  startVel[0],
                                  startAcc[0],
                                  midPos[0],
                                  midVel[0],
                                  finishPos[0],
                                  finishVel[0],
                                  finishAcc[0],
                                  T1,
                                  T2);
            am[1] = solveMidAcc1D(startPos[1],
                                  startVel[1],
                                  startAcc[1],
                                  midPos[1],
                                  midVel[1],
                                  finishPos[1],
                                  finishVel[1],
                                  finishAcc[1],
                                  T1,
                                  T2);
            return am;
        }

      private:
        static Eigen::Matrix<float, 6, 1> solveQuinticCoeff1D(
            float p0, float v0, float a0, float pT, float vT, float aT, float T) {
            const float T2 = T * T;
            const float T3 = T2 * T;
            const float T4 = T3 * T;
            const float T5 = T4 * T;

            Eigen::Matrix3f A;
            A << T3, T4, T5, 3.0f * T2, 4.0f * T3, 5.0f * T4, 6.0f * T, 12.0f * T2,
                20.0f * T3;

            const float c0 = p0;
            const float c1 = v0;
            const float c2 = 0.5f * a0;

            Eigen::Vector3f b;
            b << pT - (c0 + c1 * T + c2 * T2), vT - (c1 + 2.0f * c2 * T),
                aT - (2.0f * c2);

            const Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);

            Eigen::Matrix<float, 6, 1> c;
            c << c0, c1, c2, x[0], x[1], x[2];
            return c;
        }

        static float jerkAt(const Eigen::Matrix<float, 6, 1> &c, float t) {
            // p'''(t) = 6*c3 + 24*c4*t + 60*c5*t^2
            return 6.0f * c[3] + 24.0f * c[4] * t + 60.0f * c[5] * t * t;
        }

        static float solveMidAcc1D(float p0,
                                   float v0,
                                   float a0,
                                   float pm,
                                   float vm,
                                   float p2,
                                   float v2,
                                   float a2,
                                   float T1,
                                   float T2) {

            auto jerkGap = [&](float am) -> float {
                // 第一段: (p0,v0,a0) -> (pm,vm,am)
                const auto c1 = solveQuinticCoeff1D(p0, v0, a0, pm, vm, am, T1);
                const float j1_end = jerkAt(c1, T1);

                // 第二段: (pm,vm,am) -> (p2,v2,a2)
                const auto c2 = solveQuinticCoeff1D(pm, vm, am, p2, v2, a2, T2);
                const float j2_start = jerkAt(c2, 0.0f);

                return j1_end - j2_start; // 目标为 0
            };

            // f(am) 对 am 是线性的，用两点求解
            const float f0 = jerkGap(0.0f);
            const float f1 = jerkGap(1.0f);
            const float k = f1 - f0;

            if (std::fabs(k) < 1e-6f) {
                return 0.0f;
            }
            return -f0 / k;
        }
    };
} // namespace Dog8Dof