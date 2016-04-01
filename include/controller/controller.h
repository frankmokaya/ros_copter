#ifndef VELOCITY_CONTROL_H
#define VELOCITY_CONTROL_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <fcu_common/simple_pid.h>
#include <fcu_common/MR_Controller_Commands.h>
#include <fcu_common/Command.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <ros_copter/gainsConfig.h>
#include <dynamic_reconfigure/server.h>

using namespace std;

namespace controller
{

class Controller
{

public:

  Controller();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber camera_sub_;
  ros::Subscriber estimate_sub_;
  ros::Publisher velocity_pub_;

  // PID Controllers
  fcu_common::SimplePID roll_PID_;
  fcu_common::SimplePID pitch_PID_;
  fcu_common::SimplePID yaw_PID_;
  fcu_common::SimplePID thrust_PID_;

  // Dynamic Reconfigure Stuff
  dynamic_reconfigure::Server<ros_copter::gainsConfig> server_;
  dynamic_reconfigure::Server<ros_copter::gainsConfig>::CallbackType func_;

  nav_msgs::Odometry current_state_;
  fcu_common::Command command_;

  double thrust_to_hover_bias_;

  // Functions
  void velocityCommandCallback(const fcu_common::MR_Controller_CommandsConstPtr &msg);
  void estimateCallback(const nav_msgs::Odometry msg);
  void publishAttitudeCommand();

  void gainCallback(ros_copter::gainsConfig &config, uint32_t level);
};

} // namespace ekf

#endif // VELOCITY_CONTROL_H
