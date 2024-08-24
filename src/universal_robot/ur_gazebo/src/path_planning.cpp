#include "rrt.h"
#include <iostream>

// int main(int argc, char** argv) {
//     // 定义起点和终点的关节配置
//     Config start({0.0, -1.57, 1.57, 0.0, 1.57, 0.0});  // UR5的初始配置
//     Config goal({1.57, -1.0, 1.0, 0.0, 1.0, 0.0});    // 目标配置

//     // 创建RRT对象
//     RRT rrt(start, goal, 0.1, 0.5);  // 步长0.1，接近目标的阈值为0.5

//     // 加载UR5模型（假设urdf文件路径已知）
//     std::string urdf_file_path = "/home/ywy/pino_ws/src/ur5.urdf";
//     try {
//         rrt.loadUR5Model(urdf_file_path);
//     } catch (const std::exception& e) {
//         std::cerr << "Error loading UR5 model: " << e.what() << std::endl;
//         return -1;
//     }

//     // 加载环境模型
//     rrt.loadEnvironmentModel();  // 环境中有一个简单的盒子障碍物

//     // 生成路径
//     std::vector<Config> path;
//     try {
//         path = rrt.generatePath();
//     } catch (const std::exception& e) {
//         std::cerr << "Error generating path: " << e.what() << std::endl;
//         return -1;
//     }

//     // 输出路径
//     std::cout << "Generated path:" << std::endl;
//     for (const auto& config : path) {
//         for (double joint_angle : config.joints) {
//             std::cout << joint_angle << " ";
//         }
//         std::cout << std::endl;
//     }

//     return 0;
// }
