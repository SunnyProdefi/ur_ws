#ifndef DUALARM_IK_H
#define DUALARM_IK_H
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>

class UR5IKSolver {
 private:
  KDL::Chain chain_left;
  KDL::Chain chain_right;
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_left;
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_right;
  boost::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_left;
  boost::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_right;
  boost::shared_ptr<TRAC_IK::TRAC_IK> trac_ik_solver_left;
  boost::shared_ptr<TRAC_IK::TRAC_IK> trac_ik_solver_right;
  ros::NodeHandle nh;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

  KDL::JntArray current_joint_positions_left;
  KDL::JntArray current_joint_positions_right;
  ros::Subscriber joint_state_sub;
  interactive_markers::MenuHandler menu_handler;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  bool use_trac_ik = true;

 public:
  ros::Publisher joint_traj_pub;
  UR5IKSolver();
  ~UR5IKSolver() {};

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

  int getNumberOfJoints() const;

  bool solveIK(const geometry_msgs::PoseStamped &goal_pose,
               KDL::JntArray &result_left, KDL::JntArray &result_right);

  void feedbackCb(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void make6DofMarker();
};

#endif  // DUALARM_IK_H