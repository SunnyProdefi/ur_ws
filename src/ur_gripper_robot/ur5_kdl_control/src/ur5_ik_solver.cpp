#include "ur5_ik_solver.h"

UR5IKSolver::UR5IKSolver() {
  KDL::Tree tree;
  std::string robot_desc_string;

  nh.param("robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, tree)) {
    ROS_ERROR("Failed to construct KDL Tree");
  }

  std::string base_link_left = "ur5_1_base_link";
  std::string tip_link_left = "tool0_controller_1";
  std::string base_link_right = "ur5_2_base_link";
  std::string tip_link_right = "tool0_controller_2";

  double timeout = 5.0;
  double error = 1e-5;

  tree.getChain(base_link_left, tip_link_left, chain_left);
  tree.getChain(base_link_right, tip_link_right, chain_right);

  trac_ik_solver_left.reset(new TRAC_IK::TRAC_IK(base_link_left, tip_link_left,
                                                 "/robot_description", timeout,
                                                 error, TRAC_IK::Speed));
  trac_ik_solver_right.reset(new TRAC_IK::TRAC_IK(
      base_link_right, tip_link_right, "/robot_description", timeout, error,
      TRAC_IK::Speed));

  fk_solver_left.reset(new KDL::ChainFkSolverPos_recursive(chain_left));
  fk_solver_right.reset(new KDL::ChainFkSolverPos_recursive(chain_right));

  ik_solver_left.reset(new KDL::ChainIkSolverPos_LMA(chain_left));
  ik_solver_right.reset(new KDL::ChainIkSolverPos_LMA(chain_right));

  server.reset(new interactive_markers::InteractiveMarkerServer(
      "dual_ur5_tool0_controller", "", false));

  joint_traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/eff_joint_traj_controller/command", 10);

  current_joint_positions_left.resize(chain_left.getNrOfJoints());
  current_joint_positions_right.resize(chain_right.getNrOfJoints());

  joint_state_sub =
      nh.subscribe("/joint_states", 10, &UR5IKSolver::jointStateCallback, this);

  listener.waitForTransform("ur5_1_base_link", "ur5_2_base_link", ros::Time(0),
                            ros::Duration(3.0));
  listener.lookupTransform("ur5_1_base_link", "ur5_2_base_link", ros::Time(0),
                           transform);
}

void UR5IKSolver::jointStateCallback(
    const sensor_msgs::JointState::ConstPtr &msg) {
  std::map<std::string, int> joint_index_map;
  for (int i = 0; i < chain_left.getNrOfJoints(); ++i) {
    joint_index_map[chain_left.getSegment(i).getJoint().getName()] = i;
  }

  for (int i = 0; i < chain_right.getNrOfJoints(); ++i) {
    joint_index_map[chain_right.getSegment(i).getJoint().getName()] =
        i + chain_left.getNrOfJoints();
  }

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] != "ur5_1_robotiq_85_left_knuckle_joint" &&
        msg->name[i] != "ur5_2_robotiq_85_left_knuckle_joint") {
      if (joint_index_map.find(msg->name[i]) != joint_index_map.end()) {
        int idx = joint_index_map[msg->name[i]];
        if (idx < chain_left.getNrOfJoints()) {
          current_joint_positions_left(idx) = msg->position[i];
        } else {
          current_joint_positions_right(idx - chain_left.getNrOfJoints()) =
              msg->position[i];
        }
      }
    }
  }
}

bool UR5IKSolver::solveIK(const geometry_msgs::PoseStamped &goal_pose,
                          KDL::JntArray &result_left,
                          KDL::JntArray &result_right) {
  KDL::Frame goal_frame_left;
  KDL::Frame goal_frame_right;
  geometry_msgs::PoseStamped goal_pose_left = goal_pose;
  geometry_msgs::PoseStamped goal_pose_right = goal_pose;
  goal_pose_right.header.frame_id = "ur5_2_base_link";
  listener.transformPose("ur5_2_base_link", goal_pose, goal_pose_right);
  goal_pose_left.pose.orientation.x = -0.7071068;
  goal_pose_left.pose.orientation.y = 0;
  goal_pose_left.pose.orientation.z = 0;
  goal_pose_left.pose.orientation.w = 0.7071068;
  goal_pose_right.pose.orientation.x = 0;
  goal_pose_right.pose.orientation.y = 0.7071068;
  goal_pose_right.pose.orientation.z = 0.7071068;
  goal_pose_right.pose.orientation.w = 0;
  std::cout << "goal_pose_left: " << goal_pose_left.pose.position.x << " "
            << goal_pose_left.pose.position.y << " "
            << goal_pose_left.pose.position.z << std::endl;
  std::cout << "goal_pose_left: " << goal_pose_left.pose.orientation.x << " "
            << goal_pose_left.pose.orientation.y << " "
            << goal_pose_left.pose.orientation.z << " "
            << goal_pose_left.pose.orientation.w << std::endl;
  std::cout << "goal_pose_right: " << goal_pose_right.pose.position.x << " "
            << goal_pose_right.pose.position.y << " "
            << goal_pose_right.pose.position.z << std::endl;
  std::cout << "goal_pose_right: " << goal_pose_right.pose.orientation.x << " "
            << goal_pose_right.pose.orientation.y << " "
            << goal_pose_right.pose.orientation.z << " "
            << goal_pose_right.pose.orientation.w << std::endl;

  tf::poseMsgToKDL(goal_pose_left.pose, goal_frame_left);
  tf::poseMsgToKDL(goal_pose_right.pose, goal_frame_right);

  if (use_trac_ik) {
    KDL::Twist tolerances = KDL::Twist::Zero();
    bool left_success = trac_ik_solver_left->CartToJnt(
        current_joint_positions_left, goal_frame_left, result_left, tolerances);
    bool right_success = trac_ik_solver_right->CartToJnt(
        current_joint_positions_right, goal_frame_right, result_right,
        tolerances);
    return left_success && right_success;
    // return right_success;
  } else {
    bool left_success = ik_solver_left->CartToJnt(current_joint_positions_left,
                                                  goal_frame_left, result_left);
    bool right_success = ik_solver_right->CartToJnt(
        current_joint_positions_right, goal_frame_right, result_right);
    return left_success && right_success;
  }
}

void UR5IKSolver::feedbackCb(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "ur5_1_base_link";
    goal_pose.pose = feedback->pose;

    KDL::JntArray result_left(chain_left.getNrOfJoints());
    KDL::JntArray result_right(chain_right.getNrOfJoints());

    if (solveIK(goal_pose, result_left, result_right)) {
      ROS_INFO("Updated IK solution from Interactive Marker");

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
      for (int i = 0; i < result_left.data.size(); ++i) {
        point.positions.push_back(result_left.data[i]);
        std::cout << "result_left.data[i]: " << result_left.data[i]
                  << std::endl;
        // point.positions.push_back(0.0);
        // std::cout << "result_left.data[i]: " << 0.0 << std::endl;
      }
      for (int i = 0; i < result_right.data.size(); ++i) {
        point.positions.push_back(result_right.data[i]);
        std::cout << "result_right.data[i]: " << result_right.data[i]
                  << std::endl;
        // point.positions.push_back(0.0);
        // std::cout << "result_right.data[i]: " << 0.0 << std::endl;
      }

      point.time_from_start = ros::Duration(1.0);
      traj.points.push_back(point);

      joint_traj_pub.publish(traj);
    } else {
      ROS_ERROR("Failed to update IK from Interactive Marker");
    }
  }
}

void UR5IKSolver::make6DofMarker() {
  float norm = sqrt(1 * 1 + 1 * 1 + 0 * 0 + 0 * 0);
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "ur5_1_base_link";
  int_marker.pose.position.x = 0.0;
  int_marker.pose.position.y = 0.5;
  int_marker.pose.position.z = 0.5;
  int_marker.scale = 0.2;

  int_marker.name = "tool0_6dof";
  int_marker.description = "6-DOF Control for tool0";

  visualization_msgs::InteractiveMarkerControl sphere_control;
  sphere_control.always_visible = true;
  sphere_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  visualization_msgs::Marker sphere_marker;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.scale.x = 0.2;
  sphere_marker.scale.y = 0.2;
  sphere_marker.scale.z = 0.2;
  sphere_marker.color.r = 0.5;
  sphere_marker.color.g = 0.5;
  sphere_marker.color.b = 0.5;
  sphere_marker.color.a = 0.8;

  sphere_control.markers.push_back(sphere_marker);
  int_marker.controls.push_back(sphere_control);

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

  visualization_msgs::InteractiveMarkerControl rotate_y_control;
  rotate_y_control.orientation.w = 1 / norm;
  rotate_y_control.orientation.x = 0;
  rotate_y_control.orientation.y = 1 / norm;
  rotate_y_control.orientation.z = 0;
  rotate_y_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(rotate_y_control);

  visualization_msgs::InteractiveMarkerControl rotate_z_control;
  rotate_z_control.orientation.w = 1 / norm;
  rotate_z_control.orientation.x = 0;
  rotate_z_control.orientation.y = 0;
  rotate_z_control.orientation.z = 1 / norm;
  rotate_z_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(rotate_z_control);

  visualization_msgs::InteractiveMarkerControl move_y_control;
  move_y_control.orientation.w = 1 / norm;
  move_y_control.orientation.x = 0;
  move_y_control.orientation.y = 1 / norm;
  move_y_control.orientation.z = 0;
  move_y_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(move_y_control);

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

  interactive_markers::MenuHandler::EntryHandle entry = menu_handler.insert(
      "Solve IK", boost::bind(&UR5IKSolver::feedbackCb, this, _1));

  server->insert(int_marker);
  server->setCallback(int_marker.name,
                      boost::bind(&UR5IKSolver::feedbackCb, this, _1));
  menu_handler.apply(*server, int_marker.name);
  server->applyChanges();
}