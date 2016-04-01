#include <ros/ros.h>
#include "ekf/ekf.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ekf_node");
  ros::NodeHandle nh;

  ekf::EKF Thing;

  ros::spin();

  return 0;
}
