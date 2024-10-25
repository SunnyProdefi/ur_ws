#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()) {
    tf::StampedTransform transform1;
    tf::StampedTransform transform2;

    try {
      // 获取第一个变换：ur5_1_base_link -> ur5_1_flange
      listener.lookupTransform("ur5_1_flange", "ur5_1_base_link", ros::Time(0),
                               transform1);
      // 使用transform1进行计算
    } catch (tf::TransformException& ex) {
      ROS_ERROR(
          "Failed to get transform between ur5_1_base_link and ur5_1_flange: "
          "%s",
          ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    try {
      // 获取第二个变换：ur5_2_base_link -> ur5_2_flange
      listener.lookupTransform("ur5_2_flange", "ur5_2_base_link", ros::Time(0),
                               transform2);
      // 使用transform2进行计算
    } catch (tf::TransformException& ex) {
      ROS_ERROR(
          "Failed to get transform between ur5_2_base_link and ur5_2_flange: "
          "%s",
          ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // 在此处添加使用transform1和transform2的代码

    rate.sleep();
  }
  return 0;
}
