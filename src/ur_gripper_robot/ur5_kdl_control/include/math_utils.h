#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <Eigen/Dense>

namespace MathUtils {
Eigen::Matrix4d rotX(double angle);
Eigen::Matrix4d rotY(double angle);
Eigen::Matrix4d rotZ(double angle);
Eigen::Matrix4d trans(double x, double y, double z);
}  // namespace MathUtils

#endif  // MATH_UTILS_H