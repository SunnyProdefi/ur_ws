#include "math_utils.h"
Eigen::Matrix4d MathUtils::rotX(double angle) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(1, 1) = cos(angle);
  T(2, 1) = sin(angle);
  T(1, 2) = -sin(angle);
  T(2, 2) = cos(angle);
  return T;
}

Eigen::Matrix4d MathUtils::rotY(double angle) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 0) = cos(angle);
  T(2, 0) = -sin(angle);
  T(0, 2) = sin(angle);
  T(2, 2) = cos(angle);
  return T;
}

Eigen::Matrix4d MathUtils::rotZ(double angle) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 0) = cos(angle);
  T(1, 0) = sin(angle);
  T(0, 1) = -sin(angle);
  T(1, 1) = cos(angle);
  return T;
}

Eigen::Matrix4d MathUtils::trans(double x, double y, double z) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = x;
  T(1, 3) = y;
  T(2, 3) = z;
  return T;
}