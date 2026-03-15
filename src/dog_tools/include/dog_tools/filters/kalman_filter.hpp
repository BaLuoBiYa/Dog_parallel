/**
 * @file    kalman_filter.hpp
 * @author  Cline
 * @version 1.0
 * @date    2025-6-11
 * @brief   通用的卡尔曼滤波器库
 * @details 提供了模板化的卡尔曼滤波器类，支持不同维度的状态和测量。
 */
#pragma once

#include <Eigen/Dense> // 引入Eigen库

/**
 * @brief 通用卡尔曼滤波器类
 * @tparam StateDim 状态向量的维度 (n)
 * @tparam MeasureDim 测量向量的维度 (m)
 *
 * @details
 * 本类实现了一个通用的线性卡尔曼滤波器。卡尔曼滤波器是一种高效的递归滤波器，
 * 能够从一系列不完整的、包含噪声的测量中，估计动态系统的状态。
 * 它广泛应用于导航、控制、机器人等领域。
 *
 * **基本原理**:
 * 卡尔曼滤波器通过两个主要步骤进行操作：
 * 1.  **预测 (Predict)**：根据系统的动态模型，预测下一时刻的状态和协方差。
 *     - 状态预测: x_k = A * x_{k-1} + B * u_k
 *     - 协方差预测: P_k = A * P_{k-1} * A^T + Q
 * 2.  **更新 (Update)**：利用新的测量值来修正预测的状态和协方差。
 *     - 测量残差: y_k = z_k - H * x_k
 *     - 测量残差协方差: S_k = H * P_k * H^T + R
 *     - 卡尔曼增益: K_k = P_k * H^T * S_k^{-1}
 *     - 状态更新: x_k = x_k + K_k * y_k
 *     - 协方差更新: P_k = (I - K_k * H) * P_k
 *
 * **矩阵定义**:
 * - `x` (StateDim x 1): 状态向量，表示系统的当前估计状态。
 * - `P` (StateDim x StateDim): 状态协方差矩阵，表示状态估计的不确定性。
 * - `A` (StateDim x StateDim): 状态转移矩阵，描述系统状态如何从上一时刻演变到当前时刻。
 * - `B` (StateDim x StateDim): 控制输入矩阵，描述控制输入 `u` 如何影响系统状态。
 * - `H` (MeasureDim x StateDim): 测量矩阵，描述如何从状态中获得测量值。
 * - `Q` (StateDim x StateDim): 过程噪声协方差矩阵，表示系统模型的不确定性或未建模的扰动。
 * - `R` (MeasureDim x MeasureDim): 测量噪声协方差矩阵，表示测量传感器引入的噪声。
 *
 * **使用方法**:
 * 1.  **实例化**: 创建 `KalmanFilter` 对象，传入初始的系统矩阵和协方差矩阵。
 *     例如：`KalmanFilter<4, 2> kf(A, H, Q, R, P_initial, x_initial);`
 * 2.  **循环调用**: 在每个时间步或收到新测量时，依次调用 `predict()` 和 `update()` 方法。
 *     - `kf.predict(u);` // 如果有控制输入 `u`
 *     - `kf.update(z);` // 传入当前测量值 `z`
 * 3.  **获取状态**: 随时通过 `getState()` 获取当前的估计状态。
 *     - `Eigen::VectorXd estimated_state = kf.getState();`
 * 4.  **重置**: 如果需要重新初始化滤波器（例如，系统发生重大变化或需要重新定位），可以使用 `reset()` 方法。
 *
 * **参数调优**:
 * `Q` 和 `R` 矩阵的设置对卡尔曼滤波器的性能至关重要。
 * - `Q` 值越大，表示对模型预测的不确定性越大，滤波器会更倾向于相信测量值。
 * - `R` 值越大，表示测量噪声越大，滤波器会更倾向于相信模型预测。
 * 合理设置这两个矩阵是获得良好滤波效果的关键。
 */
template <int StateDim, int MeasureDim>
class KalmanFilter
{
public:
  // 状态向量 x_k: 估计的系统状态
  Eigen::Matrix<double, StateDim, 1> x;
  // 协方差矩阵 P_k: 状态估计的不确定性，对角线元素表示各状态分量的方差
  Eigen::Matrix<double, StateDim, StateDim> P;
  // 状态转移矩阵 A: 描述状态如何从 k-1 时刻转移到 k 时刻 (x_k = A * x_{k-1})
  Eigen::Matrix<double, StateDim, StateDim> A;
  // 控制输入矩阵 B: 描述控制输入 u 如何影响状态 (x_k = ... + B * u_k)
  Eigen::Matrix<double, StateDim, StateDim> B; // 假设控制输入维度与状态维度相同，如果不同，需要调整为 StateDim x ControlDim
  // 测量矩阵 H: 描述如何从状态中获得测量值 (z_k = H * x_k)
  Eigen::Matrix<double, MeasureDim, StateDim> H;
  // 过程噪声协方差矩阵 Q: 描述模型不确定性或未建模的扰动，通常为对角矩阵
  Eigen::Matrix<double, StateDim, StateDim> Q;
  // 测量噪声协方差矩阵 R: 描述测量传感器噪声，通常为对角矩阵
  Eigen::Matrix<double, MeasureDim, MeasureDim> R;

  /**
   * @brief 构造函数，用于初始化卡尔曼滤波器
   * @param A_in 状态转移矩阵 (A)
   * @param H_in 测量矩阵 (H)
   * @param Q_in 过程噪声协方差矩阵 (Q)
   * @param R_in 测量噪声协方差矩阵 (R)
   * @param P_in 初始状态协方差矩阵 (P_0)，表示初始状态估计的不确定性
   * @param x_in 初始状态向量 (x_0)，表示系统的初始估计状态
   * @param B_in 控制输入矩阵 (B)，可选参数，默认为零矩阵，表示没有控制输入或控制输入不影响状态
   */
  KalmanFilter(const Eigen::Matrix<double, StateDim, StateDim> &A_in,
               const Eigen::Matrix<double, MeasureDim, StateDim> &H_in,
               const Eigen::Matrix<double, StateDim, StateDim> &Q_in,
               const Eigen::Matrix<double, MeasureDim, MeasureDim> &R_in,
               const Eigen::Matrix<double, StateDim, StateDim> &P_in,
               const Eigen::Matrix<double, StateDim, 1> &x_in,
               const Eigen::Matrix<double, StateDim, StateDim> &B_in = Eigen::Matrix<double, StateDim, StateDim>::Zero())
      : A(A_in), H(H_in), Q(Q_in), R(R_in), P(P_in), x(x_in), B(B_in)
  {
  }

  /**
   * @brief 预测步骤 (时间更新)
   * @details
   * 根据系统的动态模型，预测下一时刻的状态和协方差。
   * 预测公式：
   * 1. 状态预测: x_k = A * x_{k-1} + B * u_k
   * 2. 协方差预测: P_k = A * P_{k-1} * A^T + Q
   * @param u 控制向量 (可选，默认为零向量)。如果系统没有控制输入，可以忽略此参数。
   */
  void predict(const Eigen::Matrix<double, StateDim, 1> &u = Eigen::Matrix<double, StateDim, 1>::Zero())
  {
    // 预测状态：根据上一时刻的状态和控制输入，通过状态转移矩阵 A 和控制输入矩阵 B 预测当前时刻的状态。
    x = A * x + B * u;
    // 预测协方差：根据上一时刻的协方差 P 和过程噪声协方差 Q，预测当前时刻的协方差。
    // A * P * A.transpose() 表示状态不确定性经过系统动态模型传播后的结果。
    // Q 表示模型本身的不确定性或未建模的扰动。
    P = A * P * A.transpose() + Q;
  }

  /**
   * @brief 更新步骤 (测量更新)
   * @details
   * 利用新的测量值来修正预测的状态和协方差，融合预测和测量信息以获得更准确的估计。
   * 更新公式：
   * 1. 测量残差 (Innovation): y_k = z_k - H * x_k
   * 2. 测量残差协方差 (Innovation Covariance): S_k = H * P_k * H^T + R
   * 3. 卡尔曼增益 (Kalman Gain): K_k = P_k * H^T * S_k^{-1}
   * 4. 状态更新: x_k = x_k + K_k * y_k
   * 5. 协方差更新: P_k = (I - K_k * H) * P_k
   * @param z 当前的测量向量。
   */
  void update(const Eigen::Matrix<double, MeasureDim, 1> &z)
  {
    // 计算测量残差协方差 S：表示测量预测与实际测量之间的不确定性。
    // H * P * H.transpose() 表示预测状态的不确定性在测量空间中的投影。
    // R 表示测量噪声。
    Eigen::Matrix<double, MeasureDim, MeasureDim> S = H * P * H.transpose() + R;
    // 计算卡尔曼增益 K：决定了测量值对状态估计的修正程度。
    // K 越大，表示越相信测量值；K 越小，表示越相信预测值。
    // 使用LDLT分解求解K，提高数值稳定性和效率，避免直接求逆。
    Eigen::Matrix<double, StateDim, MeasureDim> K = P * H.transpose() * S.ldlt().solve(Eigen::Matrix<double, MeasureDim, MeasureDim>::Identity());

    // 更新状态：将预测状态与测量残差通过卡尔曼增益 K 进行加权融合。
    x = x + K * (z - H * x);
    // 更新协方差：修正状态估计的不确定性。
    // (I - K * H) * P 是卡尔曼滤波的标准协方差更新公式。
    P = (Eigen::Matrix<double, StateDim, StateDim>::Identity() - K * H) * P;
    // 强制协方差矩阵对称：由于浮点运算误差，P 可能会失去对称性。
    // 强制对称操作 (P + P.transpose()) / 2.0 有助于保持 P 的有效性，提高数值稳定性。
    P = (P + P.transpose()) / 2.0;
  }

  /**
   * @brief 获取当前估计的状态向量
   * @return 当前状态向量 x
   */
  Eigen::Matrix<double, StateDim, 1> getState() const
  {
    return x;
  }

  /**
   * @brief 重置滤波器状态
   * @details
   * 当系统发生重大变化或需要重新初始化滤波器时，可以使用此方法。
   * 例如，在机器人重新定位或传感器校准后。
   * @param x_new 新的初始状态向量
   * @param P_new 新的初始协方差矩阵
   */
  void reset(const Eigen::Matrix<double, StateDim, 1> &x_new,
             const Eigen::Matrix<double, StateDim, StateDim> &P_new)
  {
    x = x_new;
    P = P_new;
  }
};
