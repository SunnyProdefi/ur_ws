#include "forward_kinematics.h"
#include "math_utils.h"
#include <iostream>

ForwardKinematics::ForwardKinematics() {
  // 构造函数，可以在此进行初始化
}

Eigen::Matrix4d ForwardKinematics::compute(const std::vector<double>& theta) {
  if (theta.size() != 6) {
    std::cerr << "关节角度数量必须为6。" << std::endl;
    return Eigen::Matrix4d::Identity();
  }

  // 初始化变换矩阵为单位矩阵
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  for (size_t i = 0; i < 6; ++i) {
    // 各个变换矩阵
    Eigen::Matrix4d T_rotZ = MathUtils::rotZ(theta[i]);
    Eigen::Matrix4d T_transZ = MathUtils::trans(0, 0, dhParams.d[i]);
    Eigen::Matrix4d T_rotX = MathUtils::rotX(dhParams.alpha[i]);
    Eigen::Matrix4d T_transX = MathUtils::trans(dhParams.a[i], 0, 0);

    // 累乘变换矩阵
    T = T * T_rotZ * T_transZ * T_rotX * T_transX;
  }

  // 将小于1e-10的数设为0，消除计算误差
  for (int i = 0; i < T.rows(); ++i) {
    for (int j = 0; j < T.cols(); ++j) {
      if (std::abs(T(i, j)) < 1e-10) {
        T(i, j) = 0;
      }
    }
  }

  return T;
}
