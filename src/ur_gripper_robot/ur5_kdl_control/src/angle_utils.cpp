#include "angle_utils.h"

void AngleUtils::controlAngles(std::vector<double>& angles) {
  for (auto& angle : angles) {
    while (angle < -M_PI) angle += 2 * M_PI;
    while (angle > M_PI) angle -= 2 * M_PI;
  }
}
