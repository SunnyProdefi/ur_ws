#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()) {
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("target_frame", "source_frame", ros::Time(0),
                               transform);
      // 使用transform进行计算
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    rate.sleep();
  }
  return 0;
}
