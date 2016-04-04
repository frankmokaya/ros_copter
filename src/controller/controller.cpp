#include "controller/controller.h"

namespace controller
{

Controller::Controller() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  // Get Parameters from Server
  nh_.param("mass", mass_, 0.519);

  // Setup publishers and subscribers
  velocity_sub_ = nh_.subscribe("/velocity_command", 1, &Controller::velocityCommandCallback, this);
  estimate_sub_ = nh_.subscribe("estimate", 1, &Controller::estimateCallback, this);
  attitude_command_pub_ = nh_.advertise<fcu_common::Command>("command", 1);

  // intialize stuff
  desired_acceleration_.x = 0.0;
  desired_acceleration_.y = 0.0;
  desired_acceleration_.z = 0.0;
  desired_yaw_rate_ = 0.0;

  // connect dynamic reconfigure
  func_ = boost::bind(&Controller::gainCallback, this, _1, _2);
  server_.setCallback(func_);

  return;
}


void Controller::velocityCommandCallback(const fcu_common::MR_Controller_CommandsConstPtr& msg)
{
  // calculate the time
  static double prev_time(ros::Time::now().toSec());
  double now = ros::Time::now().toSec();
  double dt = now - prev_time;

  // variables
  double yaw_rate_command;

  if(dt > 0.0001){

    if(msg->position_valid){
      // here we will have to convert the global goal into body-fixed commands, wrap controllers
      // around the position commands to provide velocity controls to the lower-level loops

    }else{
      desired_yaw_rate_ = msg->omega.z; // yaw rate is passed to the FCU
    }
    if(msg->velocity_valid){
      // First, convert velocity commands from vehicle-1 to body
      double roll, pitch, yaw;
      tf::Quaternion q;
      tf::quaternionMsgToTF(current_state_.pose.pose.orientation,q);
      tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
      tf::Transform q_b2v1(tf::createQuaternionFromRPY(roll, pitch, 0.0));

      tf::Vector3 v_b, v_v1;
      tf::vector3MsgToTF(current_state_.twist.twist.linear,v_b);
      tf::Vector3 v_v1_d(msg->velocity.x, msg->velocity.y, msg->velocity.z);  // desired state in v1 frame
      v_v1 = q_b2v1*v_b; // curren state in v1 frame

      desired_acceleration_.x = u_PID_.computePID(v_v1_d.x(), v_v1.x(), dt);
      desired_acceleration_.y = v_PID_.computePID(v_v1_d.y(), v_v1.y(), dt);
    }
    desired_acceleration_.z = z_PID_.computePID( msg->position.z, current_state_.pose.pose.position.z,dt); // for now, always trust the desired state in z
  }

  fcu_common::Command outgoing_command;
  mixOutput(desired_acceleration_, desired_yaw_rate_, outgoing_command);
  attitude_command_pub_.publish(outgoing_command);

  return;
}

void Controller::estimateCallback(const nav_msgs::Odometry msg)
{
  current_state_ = msg;
  return;
}

void Controller::mixOutput(geometry_msgs::Vector3 desired_acceleration, double desired_yaw_rate, fcu_common::Command & output_command){
  double thrust = mass_*sqrt(desired_acceleration.x*desired_acceleration.x
                             + desired_acceleration.y*desired_acceleration.y
                             + (desired_acceleration.z-GRAVITY)*(desired_acceleration.z-GRAVITY));
  double phi_desired = asin(mass_*desired_acceleration.y/thrust);
  double theta_desired = atan2(desired_acceleration.x,(desired_acceleration.z-GRAVITY));

  output_command.normalized_roll     = saturate(phi_desired,-1.0, 1.0);
  output_command.normalized_pitch    = saturate(theta_desired,-1.0, 1.0);
  output_command.normalized_yaw      = saturate(desired_yaw_rate,-1.0, 1.0);
  output_command.normalized_throttle = saturate(thrust,0.0,1.0);
}


void Controller::gainCallback(ros_copter::gainsConfig &config, uint32_t level)
{
  // Convert bool to double
  double xIntegrator = config.xIntegrator?1.0:0.0;
  double uIntegrator = config.uIntegrator?1.0:0.0;

  ROS_INFO("New d(PID):   %0.4f,%0.4f,%0.4f", config.dP,xIntegrator*config.dI,config.dD);
  ROS_INFO("New u(PID):   %0.4f,%0.4f,%0.4f", config.uP,uIntegrator*config.uI,config.uD);
  ROS_INFO("New v(PID):   %0.4f,%0.4f,%0.4f", config.vP,uIntegrator*config.vI,config.vD);
  ROS_INFO("New YAW(PID): %0.4f,%0.4f,%0.4f", config.yawP,config.yawI,config.yawD);

  z_PID_.setGains( config.dP,   xIntegrator*config.dI,   config.dD,   config.nedtau);
  yaw_PID_.setGains(   config.yawP, config.yawI,             config.yawD, config.yawtau);
  u_PID_.setGains(     config.uP,   uIntegrator*config.uI,   config.uD,   config.uvtau);
  v_PID_.setGains(     config.vP,   uIntegrator*config.vI,   config.vD,   config.uvtau);

  thrust_to_hover_bias_ = config.thrust_to_hover_bias;
}


double Controller::saturate(double x, double min, double max){
  if(max <= min){
    ROS_ERROR("saturate function min is greater than max");
  }
  if(x > max){
    x = max;
  }else if(x < min){
    x = min;
  }
  return x;
}


} // namespace ekf
