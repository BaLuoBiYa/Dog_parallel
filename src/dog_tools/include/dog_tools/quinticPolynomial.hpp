#pragma once
#include <Eigen/Dense>
#include <cstddef>

namespace Dog8Dof {
    class QuinticPolynomial {
        using Point2D = Eigen::Vector2<float>;

      public:
        /**
         * @brief 计算五次多项式系数
         * @param startState 起始状态 [位置, 速度, 加速度]
         * @param finishState 结束状态 [位置, 速度, 加速度]
         * @param T 运动总时间
         * @return 6x1 的系数向量 [c0, c1, c2, c3, c4, c5]，对应多项式 p(t) = c0 + c1*t +
         * c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
         */
        static Eigen::Matrix<float, 6, 1>
        computeCoefficients(const std::array<float, 3> &startState,
                            const std::array<float, 3> &finishState,
                            float T) {
            return solveQuinticCoeff1D(startState[0],
                                       startState[1],
                                       startState[2],
                                       finishState[0],
                                       finishState[1],
                                       finishState[2],
                                       T);
        }

        /**
         * @brief 采样五次多项式轨迹
         * @param coeff 五次多项式系数 [c0, c1, c2, c3, c4, c5]
         * @param tStart 采样起始时间
         * @param tEnd 采样结束时间
         * @param numPoints 采样点数量
         * @param positions 输出位置序列
         * @param velocities 输出速度序列
         */
        static void sampleTrajectory(const Eigen::Matrix<float, 6, 1> &coeff,
                                     float tStart,
                                     float tEnd,
                                     size_t numPoints,
                                     std::vector<float> &positions,
                                     std::vector<float> &velocities) {
            positions.clear();
            velocities.clear();
            if (numPoints == 0 || tEnd <= tStart) {
                return;
            }
            positions.reserve(numPoints);
            velocities.reserve(numPoints);

            for (size_t i = 0; i < numPoints; ++i) {
                const float s = (numPoints == 1) ? 0.0f
                                                 : static_cast<float>(i) /
                                                       static_cast<float>(numPoints - 1);
                const float t = tStart + s * (tEnd - tStart);
                const float t2 = t * t;
                const float t3 = t2 * t;
                const float t4 = t3 * t;
                const float t5 = t4 * t;

                const float p = coeff[0] + coeff[1] * t + coeff[2] * t2 + coeff[3] * t3 +
                                coeff[4] * t4 + coeff[5] * t5;
                const float v = coeff[1] + 2.0f * coeff[2] * t + 3.0f * coeff[3] * t2 +
                                4.0f * coeff[4] * t3 + 5.0f * coeff[5] * t4;

                positions.push_back(p);
                velocities.push_back(v);
            }
        }

        /**
         * @brief 计算通过中间点的五次多项式的中间速度
         * @param startState 起始状态 [位置, 速度]
         * @param finishState 结束状态 [位置, 速度]
         * @param midPos 中间点位置
         * @param T1 第一段运动时间 (p0 → pm)
         * @param T2 第二段运动时间 (pm → p2)
         * @param maxAbs 最大允许速度绝对值，负值表示不限制
         * @return 中间点速度
         */
        static float solveMidVel1D(std::array<float, 2> startState,
                                   std::array<float, 2> finishState,
                                   float midPos,
                                   float T1,
                                   float T2,
                                   float maxAbs = -1.0f) {
            constexpr float eps = 1e-6f;

            float vm = 0.5f * (startState[1] + finishState[1]);

            if (T1 > eps && T2 > eps) {
                const float d1 = midPos - startState[0];
                const float d2 = finishState[0] - midPos;

                vm = (6.0f * (d1 / (T1 * T1) + d2 / (T2 * T2)) -
                      2.0f * (startState[1] / T1 + finishState[0] / T2)) /
                     (4.0f * (1.0f / T1 + 1.0f / T2));
            }

            if (maxAbs > eps) {
                if (vm > maxAbs) {
                    vm = maxAbs;
                } else if (vm < -maxAbs) {
                    vm = -maxAbs;
                }
            }

            return vm;
        }

        /**
         * @brief 计算通过中间点的五次多项式的中间加速度（一维）
         *
         * 通过在中间点处 jerk 连续条件求解加速度 am：
         *   第一段 jerk(t=T1) == 第二段 jerk(t=0)
         * 由于 jerk 在端点的差距对 am 是线性的，用标准求根法计算。
         *
         * @param startState 起始状态 [位置, 速度, 加速度]
         * @param finishState 结束状态 [位置, 速度, 加速度]
         * @param midState 中点状态[位置, 速度]
         * @param p2  结束位置
         * @param v2  结束速度
         * @param a2  结束加速度
         * @param T1  第一段运动时间
         * @param T2  第二段运动时间
         * @return 中间点加速度
         */
        static float solveMidAcc1D(std::array<float, 3> startState,
                                   std::array<float, 3> finishState,
                                   std::array<float, 2> midState,
                                   float T1,
                                   float T2) {
            return solveMidAcc1DImpl(startState[0],
                                     startState[1],
                                     startState[2],
                                     midState[0],
                                     midState[1],
                                     finishState[0],
                                     finishState[1],
                                     finishState[2],
                                     T1,
                                     T2);
        }

        /**
         * @brief 计算五次多项式在时刻 t 的位置
         * @param coeff 系数向量 [c0..c5], p(t) = Σ cᵢ tⁱ
         * @param t     时间
         * @return p(t)
         */
        static float evaluatePos(const Eigen::Matrix<float, 6, 1> &coeff, float t) {
            const float t2 = t * t;
            const float t3 = t2 * t;
            const float t4 = t3 * t;
            const float t5 = t4 * t;
            return coeff[0] + coeff[1] * t + coeff[2] * t2 + coeff[3] * t3 +
                   coeff[4] * t4 + coeff[5] * t5;
        }

        /**
         * @brief 计算五次多项式在时刻 t 的速度
         * @param coeff 系数向量 [c0..c5]
         * @param t     时间
         * @return v(t) = p'(t)
         */
        static float evaluateVel(const Eigen::Matrix<float, 6, 1> &coeff, float t) {
            const float t2 = t * t;
            const float t3 = t2 * t;
            const float t4 = t3 * t;
            return coeff[1] + 2.0f * coeff[2] * t + 3.0f * coeff[3] * t2 +
                   4.0f * coeff[4] * t3 + 5.0f * coeff[5] * t4;
        }

        /**
         * @brief 计算五次多项式在时刻 t 的加速度
         * @param coeff 系数向量 [c0..c5]
         * @param t     时间
         * @return a(t) = p''(t)
         */
        static float evaluateAcc(const Eigen::Matrix<float, 6, 1> &coeff, float t) {
            const float t2 = t * t;
            const float t3 = t2 * t;
            return 2.0f * coeff[2] + 6.0f * coeff[3] * t + 12.0f * coeff[4] * t2 +
                   20.0f * coeff[5] * t3;
        }

      private:
        /**
         * @brief 解析求解 3×3 线性系统 A·x = b（Cramer 法则）
         *
         * 矩阵 A = [[T³, T⁴, T⁵],
         *           [3T², 4T³, 5T⁴],
         *           [6T, 12T², 20T³]]
         *
         * det(A) = 2·T⁹ (常量因子)，直接展开避免 Eigen QR 开销。
         */
        static Eigen::Matrix<float, 6, 1> solveQuinticCoeff1D(
            float p0, float v0, float a0, float pT, float vT, float aT, float T) {
            const float T2 = T * T;
            const float T3 = T2 * T;
            const float T4 = T3 * T;
            const float T5 = T4 * T;

            // 齐次项 c0, c1, c2
            const float c0 = p0;
            const float c1 = v0;
            const float c2 = 0.5f * a0;

            // 右端向量 b = [Δp, Δv, Δa]^T
            const float b0 = pT - (c0 + c1 * T + c2 * T2);
            const float b1 = vT - (c1 + 2.0f * c2 * T);
            const float b2 = aT - (2.0f * c2);

            // ---- Cramer 法则解析求解 A·[c3,c4,c5]^T = b ----
            // A = [[ T³,  T⁴,  T⁵ ],
            //      [3T², 4T³, 5T⁴],
            //      [ 6T,12T²,20T³]]
            //
            // det(A) = T³(4T³·20T³ - 5T⁴·12T²)
            //         - T⁴(3T²·20T³ - 5T⁴·6T)
            //         + T⁵(3T²·12T² - 4T³·6T)
            //        = T³(80T⁶-60T⁶) - T⁴(60T⁵-30T⁵) + T⁵(36T⁴-24T⁴)
            //        = 20T⁹ - 30T⁹ + 12T⁹ = 2T⁹

            const float T6 = T3 * T3;
            const float T9 = T6 * T3;
            const float invDet = 0.5f / T9; // 1/(2T⁹)

            // 伴随矩阵 × b（手动展开 3×3）
            // adj(A)[0,:] = [ (80-60)T⁶,  -(20-30)T⁵,  (48-60)T⁴ ] 的余子式转置
            // 实际用 Cramer 逐分量算更清晰：

            // c3 = det([b0, T⁴, T⁵; b1, 4T³, 5T⁴; b2, 12T², 20T³]) / det(A)
            const float num3 = b0 * (4.0f * T3 * 20.0f * T3 - 5.0f * T4 * 12.0f * T2) -
                               T4 * (b1 * 20.0f * T3 - 5.0f * T4 * b2) +
                               T5 * (b1 * 12.0f * T2 - 4.0f * T3 * b2);
            // = b0*(80T⁶-60T⁶) - T⁴*(20b1T³-5b2T⁴) + T⁵*(12b1T²-4b2T³)
            // = 20b0T⁶ - 20b1T⁷ + 5b2T⁸ + 12b1T⁷ - 4b2T⁸
            // = 20b0T⁶ - 8b1T⁷ + b2T⁸

            // c4 = det([T³, b0, T⁵; 3T², b1, 5T⁴; 6T, b2, 20T³]) / det(A)
            const float num4 = T3 * (b1 * 20.0f * T3 - 5.0f * T4 * b2) -
                               b0 * (3.0f * T2 * 20.0f * T3 - 5.0f * T4 * 6.0f * T) +
                               T5 * (3.0f * T2 * b2 - b1 * 6.0f * T);
            // = T³(20b1T³-5b2T⁴) - b0(60T⁵-30T⁵) + T⁵(3b2T²-6b1T)
            // = 20b1T⁶ - 5b2T⁷ - 30b0T⁵ + 3b2T⁷ - 6b1T⁶
            // = -30b0T⁵ + 14b1T⁶ - 2b2T⁷

            // c5 = det([T³, T⁴, b0; 3T², 4T³, b1; 6T, 12T², b2]) / det(A)
            const float num5 = T3 * (4.0f * T3 * b2 - b1 * 12.0f * T2) -
                               T4 * (3.0f * T2 * b2 - b1 * 6.0f * T) +
                               b0 * (3.0f * T2 * 12.0f * T2 - 4.0f * T3 * 6.0f * T);
            // = T³(4b2T³-12b1T²) - T⁴(3b2T²-6b1T) + b0(36T⁴-24T⁴)
            // = 4b2T⁶ - 12b1T⁵ - 3b2T⁶ + 6b1T⁵ + 12b0T⁴
            // = 12b0T⁴ - 6b1T⁵ + b2T⁶

            const float c3 = num3 * invDet;
            const float c4 = num4 * invDet;
            const float c5 = num5 * invDet;

            Eigen::Matrix<float, 6, 1> c;
            c << c0, c1, c2, c3, c4, c5;
            return c;
        }

        static float jerkAt(const Eigen::Matrix<float, 6, 1> &c, float t) {
            // p'''(t) = 6*c3 + 24*c4*t + 60*c5*t^2
            return 6.0f * c[3] + 24.0f * c[4] * t + 60.0f * c[5] * t * t;
        }

        static float solveMidAcc1DImpl(float p0,
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