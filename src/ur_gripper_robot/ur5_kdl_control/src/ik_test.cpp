#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include <iostream>

int main() {
  // 示例关节角度
  std::vector<double> theta = {0, 0, 0, 0, 0, 0};

  // 正向运动学
  ForwardKinematics fk;
  Eigen::Matrix4d T = fk.compute(theta);
  std::cout << "正向运动学结果:\n" << T << std::endl;

  // 逆向运动学
  InverseKinematics ik;
  auto solutions = ik.compute(T);

  // 检查是否有解
  if (solutions.empty()) {
    std::cerr << "没有可用的逆运动学解。" << std::endl;
    return -1;
  }

  ik.printIKResult(solutions);

  auto bestSolution = ik.selectBestSolution(theta, solutions);

  // 输出最佳解
  std::cout << "最佳逆运动学解:\n";
  for (const auto& angle : bestSolution) {
    std::cout << angle << " ";
  }
  std::cout << std::endl;

  return 0;
}
