#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <trac_ik/trac_ik.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

class UR5IKSolver {
private:
  KDL::Chain chain;
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  boost::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver;
  boost::shared_ptr<TRAC_IK::TRAC_IK> trac_ik_solver;
  ros::NodeHandle nh;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  ros::Publisher joint_traj_pub;
  KDL::JntArray current_joint_positions;
  ros::Subscriber joint_state_sub;
  interactive_markers::MenuHandler menu_handler;
  bool use_trac_ik = true;

public:
  UR5IKSolver() {
    KDL::Tree tree;
    std::string robot_desc_string;

    nh.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, tree)) {
      ROS_ERROR("Failed to construct KDL Tree");
    }

    std::string base_link = "base_link";
    std::string tip_link = "tool0";
    double timeout = 0.005;
    double error = 1e-5;

    tree.getChain(base_link, tip_link, chain);

    trac_ik_solver.reset(new TRAC_IK::TRAC_IK(
        base_link, tip_link, "/robot_description", timeout, error,
        TRAC_IK::Speed)); // 使用TRAC-IK求解器

    fk_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
    ik_solver.reset(new KDL::ChainIkSolverPos_LMA(chain));
    server.reset(new interactive_markers::InteractiveMarkerServer(
        "ur5_tool0_control", "", false));
    joint_traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/eff_joint_traj_controller/command", 10);
    current_joint_positions.resize(chain.getNrOfJoints());
    joint_state_sub = nh.subscribe("/joint_states", 10,
                                   &UR5IKSolver::jointStateCallback, this);
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    // 创建一个映射表，用于快速查找关节索引
    std::map<std::string, int> joint_index_map;
    for (int i = 0; i < chain.getNrOfJoints(); ++i) {
      joint_index_map[chain.getSegment(i).getJoint().getName()] = i;
    }

    // 遍历接收到的关节状态
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] !=
          "robotiq_85_left_knuckle_joint") { // 忽略这个特定的关节
        if (joint_index_map.find(msg->name[i]) !=
            joint_index_map.end()) { // 如果这个关节在KDL链中
          int idx = joint_index_map[msg->name[i]]; // 获取在KDL链中的索引
          current_joint_positions(idx) = msg->position[i]; // 更新对应关节的位置
        }
      }
    }
  }

  // 新方法获取链中的关节数
  int getNumberOfJoints() const { return chain.getNrOfJoints(); }

  bool solveIK(const geometry_msgs::PoseStamped &goal_pose,
               KDL::JntArray &result) {
    // 转换目标姿态
    KDL::Frame goal_frame;
    tf::poseMsgToKDL(goal_pose.pose, goal_frame);
    if (use_trac_ik) {

      // 使用TRAC-IK求解器进行逆运动学求解
      KDL::Twist tolerances = KDL::Twist::Zero();
      return trac_ik_solver->CartToJnt(current_joint_positions, goal_frame,
                                       result, tolerances) >= 0;

    } else {

      return ik_solver->CartToJnt(current_joint_positions, goal_frame,
                                  result) >= 0;
    }
  }

  void feedbackCb(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    if (feedback->event_type ==
        visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
      geometry_msgs::PoseStamped goal_pose;
      goal_pose.header.frame_id = "base_link";
      goal_pose.pose = feedback->pose;

      KDL::JntArray result(getNumberOfJoints());
      if (solveIK(goal_pose, result)) {
        ROS_INFO("Updated IK solution from Interactive Marker");

        trajectory_msgs::JointTrajectory traj;
        trajectory_msgs::JointTrajectoryPoint point;

        // 设置关节名称
        traj.joint_names.push_back("shoulder_pan_joint");
        traj.joint_names.push_back("shoulder_lift_joint");
        traj.joint_names.push_back("elbow_joint");
        traj.joint_names.push_back("wrist_1_joint");
        traj.joint_names.push_back("wrist_2_joint");
        traj.joint_names.push_back("wrist_3_joint");

        // 设置关节位置
        for (int i = 0; i < result.data.size(); ++i) {
          point.positions.push_back(result.data[i]);
        }

        point.time_from_start = ros::Duration(1.0); // 设置这个点的持续时间
        traj.points.push_back(point);

        joint_traj_pub.publish(traj); // 发布轨迹
      } else {
        ROS_ERROR("Failed to update IK from Interactive Marker");
      }
    }
  }

  void make6DofMarker() {
    float norm = sqrt(1 * 1 + 1 * 1 + 0 * 0 + 0 * 0);
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.pose.position.x = 0.3;
    int_marker.pose.position.y = -0.2;
    int_marker.pose.position.z = 0.5;
    int_marker.scale = 0.2; // 设置整体尺度，这也会影响球体大小

    int_marker.name = "tool0_6dof";
    int_marker.description = "6-DOF Control for tool0";

    // 添加一个球形Marker作为视觉元素
    visualization_msgs::InteractiveMarkerControl sphere_control;
    sphere_control.always_visible = true;
    sphere_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::NONE;

    visualization_msgs::Marker sphere_marker;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.scale.x = 0.2; // 球体的直径
    sphere_marker.scale.y = 0.2;
    sphere_marker.scale.z = 0.2;
    sphere_marker.color.r = 0.0;
    sphere_marker.color.g = 1.0;
    sphere_marker.color.b = 0.0;
    sphere_marker.color.a = 0.5; // 半透明

    sphere_control.markers.push_back(sphere_marker);
    int_marker.controls.push_back(sphere_control);

    // 添加六自由度控制
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1 / norm;
    control.orientation.x = 1 / norm;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    // 添加绕Y轴旋转的控制
    visualization_msgs::InteractiveMarkerControl rotate_y_control;
    rotate_y_control.orientation.w = 1 / norm;
    rotate_y_control.orientation.x = 0;
    rotate_y_control.orientation.y = 1 / norm;
    rotate_y_control.orientation.z = 0;
    rotate_y_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(rotate_y_control);

    // 添加绕Z轴旋转的控制
    visualization_msgs::InteractiveMarkerControl rotate_z_control;
    rotate_z_control.orientation.w = 1 / norm;
    rotate_z_control.orientation.x = 0;
    rotate_z_control.orientation.y = 0;
    rotate_z_control.orientation.z = 1 / norm;
    rotate_z_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(rotate_z_control);

    // 添加沿Y轴移动的控制
    visualization_msgs::InteractiveMarkerControl move_y_control;
    move_y_control.orientation.w = 1 / norm;
    move_y_control.orientation.x = 0;
    move_y_control.orientation.y = 1 / norm;
    move_y_control.orientation.z = 0;
    move_y_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(move_y_control);

    // 添加沿Z轴移动的控制
    visualization_msgs::InteractiveMarkerControl move_z_control;
    move_z_control.orientation.w = 1 / norm;
    move_z_control.orientation.x = 0;
    move_z_control.orientation.y = 0;
    move_z_control.orientation.z = 1 / norm;
    move_z_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(move_z_control);

    visualization_msgs::InteractiveMarkerControl menu_control;
    menu_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MENU;
    menu_control.name = "Menu";
    int_marker.controls.push_back(menu_control);

    // 创建菜单条目
    interactive_markers::MenuHandler::EntryHandle entry = menu_handler.insert(
        "Solve IK", boost::bind(&UR5IKSolver::feedbackCb, this, _1));
    // 将菜单与Marker关联
    server->insert(int_marker);
    server->setCallback(int_marker.name,
                        boost::bind(&UR5IKSolver::feedbackCb, this, _1));
    menu_handler.apply(*server, int_marker.name);
    server->applyChanges();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "ur5_kdl_ik_solver");
  ROS_INFO("Starting UR5 KDL IK Solver Node");
  UR5IKSolver solver;
  ROS_INFO("UR5 KDL IK Solver Node Started");
  solver.make6DofMarker();
  ros::spin();
  return 0;
}