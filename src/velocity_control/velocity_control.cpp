#include "velocity_control/velocity_control.h"

namespace velocity_control
{

velocityControl::velocityControl() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  // Get Parameters from Server


  // Initialize PIDs
  roll_PID_.setGains(0.1, 0.0, 0.2, 0.15);
  pitch_PID_.setGains(0.1, 0.0, 0.2, 0.15);
  yaw_PID_.setGains(0.1, 0.0, 0.2, 0.15);
  thrust_PID_.setGains(0.1, 0.0, 0.2, 0.15);

  // Setup publishers and subscribers
  camera_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &velocityControl::velocityCommandCallback, this);
  estimate_sub_ = nh_.subscribe("estimate", 1, &velocityControl::estimateCallback, this);
  velocity_pub_ = nh_.advertise<geometry_msgs::Vector3>("velocity", 1);
  return;
}


void velocityControl::velocityCommandCallback(const fcu_common::MR_Controller_CommandsConstPtr& msg)
{

  return;
}

void velocityControl::estimateCallback(const nav_msgs::Odometry msg)
{
  current_state_ = msg;
  return;
}

void velocityControl::publishAttitudeCommand()
{

}



void velocityControl::gainCallback(ros_copter::gainsConfig &config, uint32_t level)
{
  // Convert bool to double
  double xIntegrator = config.xIntegrator?1.0:0.0;
  double uIntegrator = config.uIntegrator?1.0:0.0;

  ROS_INFO("New n(PID):   %0.4f,%0.4f,%0.4f", config.nP,xIntegrator*config.nI,config.nD);
  ROS_INFO("New e(PID):   %0.4f,%0.4f,%0.4f", config.eP,xIntegrator*config.eI,config.eD);
  ROS_INFO("New d(PID):   %0.4f,%0.4f,%0.4f", config.dP,xIntegrator*config.dI,config.dD);
  ROS_INFO("New u(PID):   %0.4f,%0.4f,%0.4f", config.uP,uIntegrator*config.uI,config.uD);
  ROS_INFO("New v(PID):   %0.4f,%0.4f,%0.4f", config.vP,uIntegrator*config.vI,config.vD);
  ROS_INFO("New YAW(PID): %0.4f,%0.4f,%0.4f", config.yawP,config.yawI,config.yawD);

  thrust_PID_.setGains( config.dP,   xIntegrator*config.dI,   config.dD,   config.nedtau);
  yaw_PID_.setGains(   config.yawP, config.yawI,             config.yawD, config.yawtau);
  roll_PID_.setGains(     config.uP,   uIntegrator*config.uI,   config.uD,   config.uvtau);
  pitch_PID_.setGains(     config.vP,   uIntegrator*config.vI,   config.vD,   config.uvtau);

  thrust_to_hover_bias_ = config.thrust_to_hover_bias;
}

} // namespace ekf


