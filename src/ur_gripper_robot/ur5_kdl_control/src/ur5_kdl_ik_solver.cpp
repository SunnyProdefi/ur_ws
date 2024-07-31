#include <geometry_msgs/PoseStamped.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>

class UR5IKSolver {
private:
  KDL::Chain chain;
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  boost::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver;
  ros::NodeHandle nh;

public:
  UR5IKSolver() {
    KDL::Tree tree;
    std::string robot_desc_string;

    nh.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, tree)) {
      ROS_ERROR("Failed to construct KDL Tree");
    }

    tree.getChain("base_link", "tool0", chain);
    fk_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
    ik_solver.reset(new KDL::ChainIkSolverPos_LMA(chain));
  }
  // 新方法获取链中的关节数
  int getNumberOfJoints() const { return chain.getNrOfJoints(); }

  bool solveIK(const geometry_msgs::PoseStamped &goal_pose,
               KDL::JntArray &result) {
    KDL::Frame goal_frame;
    // Convert geometry_msgs::Pose to KDL::Frame
    tf::poseMsgToKDL(goal_pose.pose, goal_frame);

    return ik_solver->CartToJnt(KDL::JntArray(chain.getNrOfJoints()),
                                goal_frame, result) >= 0;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "ur5_kdl_ik_solver");
  ROS_INFO("Starting UR5 KDL IK Solver Node");
  UR5IKSolver solver;
  ROS_INFO("UR5 KDL IK Solver Node Started");
  // Create a goal pose
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "base_link";
  goal_pose.header.stamp = ros::Time::now();
  goal_pose.pose.position.x = 0.3;
  goal_pose.pose.position.y = -0.2;
  goal_pose.pose.position.z = 0.5;
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
      0, 0, 1.57); // Orientation in radians

  KDL::JntArray result(solver.getNumberOfJoints());
  if (solver.solveIK(goal_pose, result)) {
    ROS_INFO("IK solution successful");
    for (unsigned int i = 0; i < result.data.size(); i++) {
      ROS_INFO("Joint %d: %f", i, result(i));
    }
  } else {
    ROS_INFO("Failed to solve IK");
  }
  ros::spin();
  return 0;
}