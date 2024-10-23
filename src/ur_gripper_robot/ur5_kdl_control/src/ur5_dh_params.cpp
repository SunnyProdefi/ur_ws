#include "ur5_dh_params.h"

UR5DHParams::UR5DHParams() {
  a = {0, -0.42500, -0.39225, 0, 0, 0};
  alpha = {M_PI_2, 0, 0, M_PI_2, -M_PI_2, 0};
  d = {0.089159, 0, 0, 0.10915, 0.09465, 0.0823};
}