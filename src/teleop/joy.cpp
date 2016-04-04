#include "teleop/joy.h"

// #include <rotor_gazebo/default_topics.h>


namespace ros_copter {
  RCJoy::RCJoy(){
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("command_topic", command_topic_, "command");

  pnh.param<int>("axis_roll_", axes_.roll, 2);
  pnh.param<int>("axis_pitch_", axes_.pitch, 3);
  pnh.param<int>("axis_thrust_", axes_.thrust, 1);
  pnh.param<int>("axis_yaw_", axes_.yaw, 0);

  pnh.param<double>("max_u", max_.u, 1.0);
  pnh.param<double>("max_v", max_.v, 1.0);
  pnh.param<double>("max_yaw_rate", max_.yaw_rate, 1.0);
  pnh.param<double>("max_dz", max_.dz, 0.1);

  pnh.param<int>("axis_direction_roll", axes_.roll_direction, -1);
  pnh.param<int>("axis_direction_pitch", axes_.pitch_direction, -1);
  pnh.param<int>("axis_direction_thrust", axes_.thrust_direction, 1);
  pnh.param<int>("axis_direction_yaw", axes_.yaw_direction, -1);

  pnh.param<double>("thrust_to_mass_ratio", thrust_to_mass_ratio_, 3.81);  // [N]

  pnh.param<int>("button_takeoff", buttons_.fly.index, 0);

  // define the control type
  int control;
  pnh.param<int>("control_type", control, 0);
  switch(control){
    case 0:
      ROS_WARN("joy_teleop is sending attitude commands");
      control_type_ = ATTITUDE;
      break;
    case 1:
      ROS_WARN("joy_teleop is sending velocity commands");
      control_type_ = VELOCITY;
      break;
    default:
      ROS_ERROR("Unknown joy_teleop command type");
      break;
  }


  // set up the appropriate publisher
  if(control_type_ == ATTITUDE){
    command_pub_ = nh_.advertise<fcu_common::Command>(command_topic_,10);

    ROS_ERROR_STREAM("thrust to mass ratio" << thrust_to_mass_ratio_);

    att_command_msg_.normalized_roll = 0;
    att_command_msg_.normalized_pitch = 0;
    att_command_msg_.normalized_yaw = 0;
    att_command_msg_.normalized_throttle = 0;
  }else if(control_type_ == VELOCITY){
    command_pub_ = nh_.advertise<fcu_common::MR_Controller_Commands>(command_topic_,10);

    vel_command_msg_.acceleration_valid = false;
    vel_command_msg_.velocity_valid = true;
    vel_command_msg_.position_valid = false;
  }

  joy_sub_ = nh_.subscribe("joy", 10, &RCJoy::JoyCallback, this);
  fly_mav_ = false;
}


void RCJoy::StopMav() {
  switch(control_type_){
    case ATTITUDE:
      // take care of attitude commands
      att_command_msg_.normalized_roll = 0;
      att_command_msg_.normalized_pitch = 0;
      att_command_msg_.normalized_yaw = 0;
      att_command_msg_.normalized_throttle = 0;
      break;


    case VELOCITY:
      // take care of velocity commands
      vel_command_msg_.velocity.x =  0;
      vel_command_msg_.velocity.y = 0;
      vel_command_msg_.omega.z = 0;
      break;


    default:
      ROS_ERROR("joy_teleop unknown control type");
  }

}

void RCJoy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  if(fly_mav_){
    // save off joy message
    current_joy_ = *msg;

    switch(control_type_){
      case ATTITUDE:
        // take care of attitude commands
        att_command_msg_.normalized_roll = msg->axes[axes_.roll] * axes_.roll_direction;
        att_command_msg_.normalized_pitch = msg->axes[axes_.pitch] * axes_.pitch_direction;
        att_command_msg_.normalized_yaw = msg->axes[axes_.yaw] * axes_.yaw_direction;
        if(msg->axes[axes_.thrust]*axes_.thrust_direction < 0){
          // some fraction of the mass
          att_command_msg_.normalized_throttle = (msg->axes[axes_.thrust]+1)*1.0/thrust_to_mass_ratio_;
        }else{
          // some fraction of remaining thrust
          att_command_msg_.normalized_throttle = msg->axes[axes_.thrust]*(1.0-(1.0/thrust_to_mass_ratio_))+1.0/thrust_to_mass_ratio_;
        }
        break;


      case VELOCITY:
        // take care of velocity commands
        vel_command_msg_.velocity.x =  max_.u * (msg->axes[axes_.pitch] * axes_.pitch_direction);
        vel_command_msg_.velocity.y = max_.v * (msg->axes[axes_.roll] * axes_.roll_direction);
        vel_command_msg_.omega.z = max_.yaw_rate * (msg->axes[axes_.yaw] * axes_.yaw_direction);
        vel_command_msg_.position.z += max_.dz * (msg->axes[axes_.thrust]*axes_.thrust_direction);
        break;


      default:
        ROS_ERROR("joy_teleop unknown control type");
    }

  }else{
    StopMav();
  }


  if(msg->buttons[buttons_.fly.index]==0 && buttons_.fly.prev_value==1){ // button release
    fly_mav_ = !fly_mav_;
  }
  buttons_.fly.prev_value = msg->buttons[buttons_.fly.index];

  ros::Time update_time = ros::Time::now();
  Publish();
}

void RCJoy::Publish() {
  switch(control_type_){
    case ATTITUDE:
      // take care of attitude commands
      command_pub_.publish(att_command_msg_);
      break;

    case VELOCITY:
      // take care of velocity commands
      command_pub_.publish(vel_command_msg_);
      break;

    default:
      ROS_ERROR("joy_teleop unknown control type");
  }

}

} // end namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotor_sim_joy");
  ros_copter::RCJoy joy;

  ros::spin();

  return 0;
}
