#include "rrt.h"

#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <urdf/model.h>  // 包含用于URDF模型的头文件

#include <fstream>
#include <iostream>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>  // 包含用于KDL解析的头文件
#include <kdl_parser/kdl_parser.hpp>
#include <random>
#include <streambuf>

RRT::RRT(const Config& start, const Config& goal, double step_size,
         double max_dist)
    : start(start), goal(goal), step_size(step_size), max_dist(max_dist) {
  nodes.push_back(start);
}

void RRT::loadUR5Model(const std::string& urdf_file_path) {
  // 加载URDF文件
  std::string urdf_xml;
  std::ifstream urdf_file(urdf_file_path);
  if (!urdf_file) {
    throw std::runtime_error("Failed to open URDF file.");
  }
  urdf_xml.assign(std::istreambuf_iterator<char>(urdf_file),
                  std::istreambuf_iterator<char>());

  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    throw std::runtime_error("Failed to parse URDF model.");
  }

  // 使用KDL解析URDF并生成运动链
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
    throw std::runtime_error("Failed to parse KDL tree from URDF.");
  }

  if (!kdl_tree.getChain("base_link", "ee_link", kdl_chain)) {
    throw std::runtime_error("Failed to get KDL chain.");
  }

  // 创建正向运动学求解器
  fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain);

  // 根据URDF文件中的碰撞几何体生成FCL碰撞对象
  for (const auto& link_pair : model.links_) {
    const auto& link = link_pair.second;
    if (!link->collision || !link->collision->geometry) continue;

    std::shared_ptr<fcl::CollisionGeometry<double>> fcl_geometry;
    const auto& geom = link->collision->geometry;

    if (geom->type == urdf::Geometry::CYLINDER) {
      auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(geom);
      fcl_geometry = std::make_shared<fcl::Cylinder<double>>(cylinder->radius,
                                                             cylinder->length);
    } else if (geom->type == urdf::Geometry::BOX) {
      auto box = std::dynamic_pointer_cast<urdf::Box>(geom);
      fcl_geometry = std::make_shared<fcl::Box<double>>(box->dim.x, box->dim.y,
                                                        box->dim.z);
    }

    if (fcl_geometry) {
      auto collision_object =
          std::make_shared<fcl::CollisionObject<double>>(fcl_geometry);
      robot_links.push_back(collision_object);
    }
  }
}

void RRT::loadEnvironmentModel() {
  // 创建一个简单的盒子作为障碍物
  auto box_geom =
      std::make_shared<fcl::Box<double>>(1.0, 1.0, 1.0);  // 1m x 1m x 1m 的盒子
  auto box_obj = std::make_shared<fcl::CollisionObject<double>>(box_geom);

  // 将盒子放置在环境中的某个位置
  fcl::Transform3<double> box_transform;
  box_transform.translation() =
      fcl::Vector3<double>(1.0, 0.0, 0.5);  // 将盒子放置在 (1, 0, 0.5) 位置
  box_obj->setTransform(box_transform);

  // 添加到障碍物列表
  obstacles.push_back(box_obj);
}

void RRT::updateRobotModel(const Config& config) {
  // 使用KDL进行正向运动学计算
  KDL::JntArray joint_positions(6);
  for (size_t i = 0; i < config.joints.size(); ++i) {
    joint_positions(i) = config.joints[i];
  }

  std::vector<KDL::Frame> link_frames(robot_links.size());
  for (size_t i = 0; i < kdl_chain.getNrOfSegments(); ++i) {
    fk_solver->JntToCart(joint_positions, link_frames[i], i + 1);
  }

  // 更新FCL中的每个链接的位置
  for (size_t i = 0; i < robot_links.size(); ++i) {
    KDL::Frame frame = link_frames[i];
    fcl::Transform3<double> transform;

    transform.translation() =
        fcl::Vector3<double>(frame.p.x(), frame.p.y(), frame.p.z());
    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);
    transform.linear() = fcl::Quaternion<double>(w, x, y, z).toRotationMatrix();

    robot_links[i]->setTransform(transform);
    robot_links[i]->computeAABB();  // 更新轴对齐包围盒
  }
}

bool RRT::checkCollision(const Config& config) {
  // 更新机器人模型的配置
  updateRobotModel(config);

  // 检查每个链接与障碍物之间的碰撞
  for (const auto& link : robot_links) {
    for (const auto& obstacle : obstacles) {
      fcl::CollisionRequest<double> request;
      fcl::CollisionResult<double> result;
      if (fcl::collide(link.get(), obstacle.get(), request, result)) {
        return true;  // 碰撞发生
      }
    }
  }
  return false;  // 无碰撞
}

Config RRT::steer(const Config& nearest, const Config& randomConfig) {
  Config newConfig = nearest;
  for (size_t i = 0; i < nearest.joints.size(); ++i) {
    double diff = randomConfig.joints[i] - nearest.joints[i];
    if (std::abs(diff) > step_size) {
      newConfig.joints[i] = nearest.joints[i] + std::copysign(step_size, diff);
    } else {
      newConfig.joints[i] = randomConfig.joints[i];
    }
  }
  return newConfig;
}

Config RRT::getRandomConfig() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(
      -M_PI, M_PI);  // 假设关节角度范围在 [-π, π] 之间

  Config randConfig;
  for (auto& joint : randConfig.joints) {
    joint = dis(gen);
  }
  return randConfig;
}

Config RRT::nearestNeighbor(const Config& randomConfig) {
  // 找到与随机配置最近的节点
  return *std::min_element(
      nodes.begin(), nodes.end(), [&](const Config& a, const Config& b) {
        double dist_a = 0.0, dist_b = 0.0;
        for (size_t i = 0; i < a.joints.size(); ++i) {
          dist_a += std::pow(randomConfig.joints[i] - a.joints[i], 2);
          dist_b += std::pow(randomConfig.joints[i] - b.joints[i], 2);
        }
        return dist_a < dist_b;
      });
}

bool RRT::closeToGoal(const Config& config) {
  double dist = 0.0;
  for (size_t i = 0; i < config.joints.size(); ++i) {
    dist += std::pow(config.joints[i] - goal.joints[i], 2);
  }
  return std::sqrt(dist) < max_dist;
}

std::vector<Config> RRT::generatePath() {
  while (true) {
    Config randConfig = getRandomConfig();
    Config nearest = nearestNeighbor(randConfig);
    Config newConfig = steer(nearest, randConfig);

    // 检查新配置是否碰撞
    if (!checkCollision(newConfig)) {
      nodes.push_back(newConfig);

      // 如果新节点接近终点，则完成路径生成
      if (closeToGoal(newConfig)) {
        nodes.push_back(goal);
        break;
      }
    }
  }
  return nodes;  // 返回生成的路径
}

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
