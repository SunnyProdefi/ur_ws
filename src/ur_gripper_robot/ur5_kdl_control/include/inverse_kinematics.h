#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <Eigen/Dense>
#include <vector>
#include "ur5_dh_params.h"

class InverseKinematics {
 public:
  InverseKinematics();
  // 计算逆运动学，返回8组解
  std::vector<std::vector<double>> compute(const Eigen::Matrix4d& T);

  void printIKResult(std::vector<std::vector<double>> solutions);
  std::vector<double> selectBestSolution(
      const std::vector<double>& currentPos,
      const std::vector<std::vector<double>>& solutions,
      const std::vector<double>& weights = std::vector<double>(6, 1.0));

 private:
  UR5DHParams dhParams;
  void controlAngles(std::vector<double>& angles);
  // 手动实现 clamp 函数
  template <typename T>
  T clamp(T val, T min_val, T max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
  }
};

#endif  // INVERSEKINEMATICS_H
