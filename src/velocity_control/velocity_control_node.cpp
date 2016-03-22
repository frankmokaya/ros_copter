#include <ros/ros.h>
#include "velocity_control/velocity_control.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_control");
  ros::NodeHandle nh;

  velocity_control::monoVO Thing;

  ros::spin();

  return 0;
}
