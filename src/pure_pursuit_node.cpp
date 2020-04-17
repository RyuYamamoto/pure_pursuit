#include <ros/ros.h>
#include <pure_pursuit/pure_pursuit.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pure_pursuit_node");
  ros::NodeHandle nh;
  PurePursuit instance(nh);
  ros::spin();
  return 0;
}