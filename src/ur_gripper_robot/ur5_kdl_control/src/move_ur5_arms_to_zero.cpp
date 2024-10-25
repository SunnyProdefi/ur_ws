#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_ur5_arms_to_zero");
  ros::NodeHandle nh;

  // 创建动作客户端，连接到控制器的FollowJointTrajectory动作接口
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      client("/eff_joint_traj_controller/follow_joint_trajectory", true);

  ROS_INFO("等待动作服务器启动...");
  client.waitForServer();
  ROS_INFO("动作服务器已启动，发送目标。");

  // 定义所有关节的名称
  std::vector<std::string> joint_names = {
      "ur5_1_shoulder_pan_joint", "ur5_1_shoulder_lift_joint",
      "ur5_1_elbow_joint",        "ur5_1_wrist_1_joint",
      "ur5_1_wrist_2_joint",      "ur5_1_wrist_3_joint",
      "ur5_2_shoulder_pan_joint", "ur5_2_shoulder_lift_joint",
      "ur5_2_elbow_joint",        "ur5_2_wrist_1_joint",
      "ur5_2_wrist_2_joint",      "ur5_2_wrist_3_joint"};

  // 创建目标轨迹
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = joint_names;

  // 创建轨迹点，目标是所有关节角为0
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.resize(joint_names.size(), 0.0);  // 所有关节目标位置为0
  point.velocities.resize(joint_names.size(),
                          0.0);  // 速度可以设为0，表示使用默认速度
  point.time_from_start = ros::Duration(5.0);  // 在5秒内完成运动

  trajectory.points.push_back(point);

  // 创建动作目标
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;

  // 发送目标到动作服务器
  client.sendGoal(goal);

  // 等待结果
  client.waitForResult();

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The movement is complete.");
  else
    ROS_WARN("The movement was not completed successfully.");

  return 0;
}
