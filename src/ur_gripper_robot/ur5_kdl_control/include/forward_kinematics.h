#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <Eigen/Dense>
#include <vector>
#include "ur5_dh_params.h"

class ForwardKinematics {
 public:
  ForwardKinematics();
  // 计算正向运动学
  Eigen::Matrix4d compute(const std::vector<double>& theta);

 private:
  UR5DHParams dhParams;
};

#endif  // FORWARD_KINEMATICS_H
