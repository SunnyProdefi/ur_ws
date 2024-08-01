# ROS Workspace

This is a ROS workspace that contains your ROS packages and code. It is organized using the recommended ROS package structure.

## Installation

1. Clone this repository to your local machine:

    ```bash
    git clone <repository_url>
    ```

2. Build the workspace:

    ```bash
    cd <workspace_directory>
    catkin_make
    ```

## Usage

1. Source the workspace:

    ```bash
    source <workspace_directory>/devel/setup.bash
    ```

2. Launch your ROS nodes:

    ```bash
    roslaunch <package_name> <launch_file>
    ```

### Using KDL/TRAC-IK for UR5 Inverse Kinematics (IK)

To demonstrate solving the inverse kinematics for the UR5 robot arm using KDL/TRAC-IK, follow these steps:

1. Start RVIZ and GAZEBO:

    ```bash
    roslaunch ur_gripper_gazebo ur5_bringup_with_rviz.launch 
    ```

2. Launch the control node:

    ```bash
    rosrun ur5_kdl_control ur5_kdl_ik_solver
    ```

3. In RVIZ, right-click on the MARK coordinates and choose 'Solve IK' from the context menu to solve the inverse kinematics. This will allow you to control the robot arm's end effector to move to the target pose.

## Contributing

Contributions are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request.

## License

This project is licensed under the [MIT License](LICENSE).
