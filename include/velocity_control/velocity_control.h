#ifndef VELOCITY_CONTROL_H
#define VELOCITY_CONTROL_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <fcu_common/simple_pid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

using namespace std;

namespace velocity_control
{

class velocityControl
{

public:

  velocityControl();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber camera_sub_;
  ros::Subscriber estimate_sub_;
  ros::Publisher velocity_pub_;

  nav_msgs::Odometry current_state_;
  geometry_msgs::Vector3 velocity_measurement_;

  // Functions
  void velocityCommandCallback(const geometry_msgs::Vector3ConstPtr& msg);
  void estimateCallback(const nav_msgs::OdometryConstPtr& msg);
  void publishAttitudeCommand();
};

} // namespace ekf

#endif // VELOCITY_CONTROL_H
