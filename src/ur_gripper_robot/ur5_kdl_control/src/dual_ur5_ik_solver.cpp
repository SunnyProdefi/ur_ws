#include "dualarm_ik.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "ur5_kdl_ik_solver");
  ROS_INFO("Starting UR5 KDL IK Solver Node");
  UR5IKSolver solver;
  ROS_INFO("UR5 KDL IK Solver Node Started");
  solver.make6DofMarker();
  trajectory_msgs::JointTrajectory traj;
  trajectory_msgs::JointTrajectoryPoint point;

  traj.joint_names.push_back("ur5_1_shoulder_pan_joint");
  traj.joint_names.push_back("ur5_1_shoulder_lift_joint");
  traj.joint_names.push_back("ur5_1_elbow_joint");
  traj.joint_names.push_back("ur5_1_wrist_1_joint");
  traj.joint_names.push_back("ur5_1_wrist_2_joint");
  traj.joint_names.push_back("ur5_1_wrist_3_joint");
  traj.joint_names.push_back("ur5_2_shoulder_pan_joint");
  traj.joint_names.push_back("ur5_2_shoulder_lift_joint");
  traj.joint_names.push_back("ur5_2_elbow_joint");
  traj.joint_names.push_back("ur5_2_wrist_1_joint");
  traj.joint_names.push_back("ur5_2_wrist_2_joint");
  traj.joint_names.push_back("ur5_2_wrist_3_joint");

  // 设置关节位置
  for (int i = 0; i < 6; ++i) {
    point.positions.push_back(0.0);
    std::cout << "result_left.data[i]: " << 0.0 << std::endl;
  }
  for (int i = 0; i < 6; ++i) {
    point.positions.push_back(0.0);
    std::cout << "result_right.data[i]: " << 0.0 << std::endl;
  }

  point.time_from_start = ros::Duration(1.0);
  traj.points.push_back(point);

  solver.joint_traj_pub.publish(traj);
  ros::spin();
  return 0;
}