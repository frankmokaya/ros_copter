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
#include <tf/tf.h>

#define GRAVITY 9.80665

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
  ros::Subscriber velocity_sub_;
  ros::Subscriber estimate_sub_;
  ros::Publisher attitude_command_pub_;
  geometry_msgs::Vector3 desired_acceleration_;
  double desired_yaw_rate_;

  // PID Controllers
  fcu_common::SimplePID u_PID_;
  fcu_common::SimplePID v_PID_;
  fcu_common::SimplePID yaw_PID_;
  fcu_common::SimplePID z_PID_;

  // Dynamic Reconfigure Stuff
  dynamic_reconfigure::Server<ros_copter::gainsConfig> server_;
  dynamic_reconfigure::Server<ros_copter::gainsConfig>::CallbackType func_;

  nav_msgs::Odometry current_state_;
  fcu_common::Command command_;

  double thrust_to_hover_bias_;
  double mass_;
  double max_roll_, max_pitch_, max_yaw_rate_;

  // Functions
  void velocityCommandCallback(const fcu_common::MR_Controller_CommandsConstPtr &msg);
  void estimateCallback(const nav_msgs::Odometry msg);

  void gainCallback(ros_copter::gainsConfig &config, uint32_t level);
  void mixOutput(geometry_msgs::Vector3 desired_acceleration, double desired_yaw_rate, fcu_common::Command & output_command);
  double saturate(double x, double min, double max);
};

} // namespace ekf

#endif // VELOCITY_CONTROL_H
