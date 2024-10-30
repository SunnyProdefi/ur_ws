#include "inverse_kinematics.h"
#include <cmath>
#include <iostream>
#include <algorithm>  // for std::clamp

InverseKinematics::InverseKinematics() {
  // 构造函数，可以在此进行初始化
}

void InverseKinematics::controlAngles(std::vector<double>& angles) {
  for (auto& angle : angles) {
    while (angle < -M_PI) angle += 2 * M_PI;
    while (angle > M_PI) angle -= 2 * M_PI;
  }
}

std::vector<std::vector<double>> InverseKinematics::compute(
    const Eigen::Matrix4d& T) {
  // 提取矩阵元素
  double nx = T(0, 0);
  double ny = T(1, 0);
  double nz = T(2, 0);
  double ox = T(0, 1);
  double oy = T(1, 1);
  double oz = T(2, 1);
  double ax = T(0, 2);
  double ay = T(1, 2);
  double az = T(2, 2);
  double px = T(0, 3);
  double py = T(1, 3);
  double pz = T(2, 3);

  std::vector<double> theta1(8, 0.0), theta2(8, 0.0), theta3(8, 0.0);
  std::vector<double> theta4(8, 0.0), theta5(8, 0.0), theta6(8, 0.0);
  std::vector<double> m(8, 0.0), n(8, 0.0), s2(8, 0.0), c2(8, 0.0);

  // 计算theta1，使用 dhParams.d[5]
  double mx = dhParams.d[5] * ay - py;
  double nx_theta1 = ax * dhParams.d[5] - px;

  for (int i = 0; i < 4; i++) {
    theta1[i] = atan2(mx, nx_theta1) -
                atan2(dhParams.d[3], sqrt(mx * mx + nx_theta1 * nx_theta1 -
                                          dhParams.d[3] * dhParams.d[3]));
    theta1[i + 4] = atan2(mx, nx_theta1) -
                    atan2(dhParams.d[3], -sqrt(mx * mx + nx_theta1 * nx_theta1 -
                                               dhParams.d[3] * dhParams.d[3]));
  }

  // 计算theta5
  theta5[0] = acos(ax * sin(theta1[0]) - ay * cos(theta1[0]));
  theta5[1] = theta5[0];
  theta5[2] = -theta5[0];
  theta5[3] = theta5[2];
  theta5[4] = acos(ax * sin(theta1[4]) - ay * cos(theta1[4]));
  theta5[5] = theta5[4];
  theta5[6] = -theta5[4];
  theta5[7] = theta5[6];

  // 计算theta6
  for (int i = 0; i < 8; i++) {
    double mx6 = nx * sin(theta1[i]) - ny * cos(theta1[i]);
    double nx6 = ox * sin(theta1[i]) - oy * cos(theta1[i]);
    theta6[i] = atan2(mx6, nx6) - atan2(sin(theta5[i]), 0);
  }

  for (int i = 0; i < 8; i++) {
    m[i] = dhParams.d[4] *
               (sin(theta6[i]) * (nx * cos(theta1[i]) + ny * sin(theta1[i])) +
                cos(theta6[i]) * (ox * cos(theta1[i]) + oy * sin(theta1[i]))) -
           dhParams.d[5] * (ax * cos(theta1[i]) + ay * sin(theta1[i])) +
           px * cos(theta1[i]) + py * sin(theta1[i]);

    n[i] = pz - dhParams.d[0] - az * dhParams.d[5] +
           dhParams.d[4] * (oz * cos(theta6[i]) + nz * sin(theta6[i]));

    double acos_input =
        (m[i] * m[i] + n[i] * n[i] - dhParams.a[1] * dhParams.a[1] -
         dhParams.a[2] * dhParams.a[2]) /
        (2 * dhParams.a[1] * dhParams.a[2]);

    // 限制 acos_input 的范围在 [-1, 1]
    acos_input = clamp(acos_input, -1.0, 1.0);

    if ((i % 2) == 0) {
      theta3[i] = acos(acos_input);
    } else {
      theta3[i] = -acos(acos_input);
    }
  }

  // 计算theta2
  for (int i = 0; i < 8; i++) {
    s2[i] = ((dhParams.a[2] * cos(theta3[i]) + dhParams.a[1]) * n[i] -
             dhParams.a[2] * sin(theta3[i]) * m[i]) /
            (dhParams.a[1] * dhParams.a[1] + dhParams.a[2] * dhParams.a[2] +
             2 * dhParams.a[1] * dhParams.a[2] * cos(theta3[i]));
    c2[i] = (m[i] + dhParams.a[2] * sin(theta3[i]) * s2[i]) /
            (dhParams.a[2] * cos(theta3[i]) + dhParams.a[1]);
    theta2[i] = atan2(s2[i], c2[i]);
  }

  // 计算theta4
  for (int i = 0; i < 8; i++) {
    theta4[i] =
        atan2(-sin(theta6[i]) * (nx * cos(theta1[i]) + ny * sin(theta1[i])) -
                  cos(theta6[i]) * (ox * cos(theta1[i]) + oy * sin(theta1[i])),
              oz * cos(theta6[i]) + nz * sin(theta6[i])) -
        theta2[i] - theta3[i];
  }

  // 将结果返回
  std::vector<std::vector<double>> solutions;
  for (int i = 0; i < 8; i++) {
    std::vector<double> solution = {theta1[i], theta2[i], theta3[i],
                                    theta4[i], theta5[i], theta6[i]};
    controlAngles(solution);
    solutions.push_back(solution);
  }

  return solutions;
}

void InverseKinematics::printIKResult(
    std::vector<std::vector<double>> solutions) {
  if (solutions.empty()) {
    std::cout << "无法求解逆运动学。" << std::endl;
    return;
  }

  std::cout << "逆运动学解：" << std::endl;
  for (size_t i = 0; i < solutions.size(); ++i) {
    std::cout << "解" << i + 1 << ": ";
    for (size_t j = 0; j < solutions[i].size(); ++j) {
      std::cout << solutions[i][j] << " ";
    }
    std::cout << std::endl;
  }
}

std::vector<double> InverseKinematics::selectBestSolution(
    const std::vector<double>& currentPos,
    const std::vector<std::vector<double>>& solutions,
    const std::vector<double>& weights) {
  if (solutions.empty()) {
    return {};
  }

  double minScore = std::numeric_limits<double>::max();
  std::vector<double> bestSolution;

  for (const auto& solution : solutions) {
    double score = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      double diff = std::abs(solution[i] - currentPos[i]);
      score += weights[i] * diff;
    }
    if (score < minScore) {
      minScore = score;
      bestSolution = solution;
    }
  }

  return bestSolution;
}