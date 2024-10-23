#ifndef UR5_DH_PARAMS_H
#define UR5_DH_PARAMS_H

#include <vector>
#include <cmath>

class UR5DHParams {
 public:
  // 连杆长度
  std::vector<double> a;
  // 连杆扭转角
  std::vector<double> alpha;
  // 连杆偏移量
  std::vector<double> d;

  // 构造函数
  UR5DHParams();
};

#endif  // UR5_DH_PARAMS_H
