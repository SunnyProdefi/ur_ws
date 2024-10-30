#ifndef RRT_H
#define RRT_H

#include <vector>
#include <memory>
#include <fcl/fcl.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

// 定义机械臂的配置，包含6个关节角度
struct Config {
    std::vector<double> joints;
    Config() : joints(6, 0.0) {}
    Config(std::vector<double> joints) : joints(joints) {}
};

class RRT {
private:
    std::vector<Config> nodes;  // 存储路径中的所有节点
    Config start, goal;         // 起点和终点配置
    double step_size;           // RRT中的步长
    double max_dist;            // 终点的最大接近距离

    std::vector<std::shared_ptr<fcl::CollisionObject<double>>> robot_links;  // 机械臂的碰撞对象
    std::vector<std::shared_ptr<fcl::CollisionObject<double>>> obstacles;    // 环境中的障碍物

    KDL::Chain kdl_chain;       // KDL链，用于正向运动学计算
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;  // 正向运动学求解器

    void updateRobotModel(const Config& config);  // 更新机械臂模型的位姿
    bool checkCollision(const Config& config);    // 检查是否发生碰撞
    Config steer(const Config& nearest, const Config& randomConfig);  // 扩展树
    Config getRandomConfig();  // 生成随机配置
    Config nearestNeighbor(const Config& randomConfig);  // 找到树中最近的节点
    bool closeToGoal(const Config& config);  // 检查是否接近终点

public:
    RRT(const Config& start, const Config& goal, double step_size = 0.1, double max_dist = 0.5);
    void loadUR5Model(const std::string& urdf_file_path);  // 加载UR5的URDF模型
    void loadEnvironmentModel();  // 加载环境中的障碍物
    std::vector<Config> generatePath();  // 生成路径
};

#endif // RRT_H
