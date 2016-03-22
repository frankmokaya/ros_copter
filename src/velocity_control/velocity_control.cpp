#include "velocity_control/velocity_control.h"

namespace velocity_control
{

velocityControl::velocityControl() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~/velocity_control"))
{
  // Get Parameters from Server
  string camera_parameter_filename;
  nh_private_.param<string>("camera_parameter_filename", camera_parameter_filename, "camera.yaml");

//  // Initialize Camera
//  FileStorage file(camera_parameter_filename, FileStorage::WRITE);
//  file["I"] >> I_;
//  file["D"] >> D_;

  // Setup publishers and subscribers
  camera_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &velocityControl::cameraCallback, this);
  estimate_sub_ = nh_.subscribe("estimate", 1, &velocityControl::estimateCallback, this);
  velocity_pub_ = nh_.advertise<geometry_msgs::Vector3>("velocity", 1);
  return;
}


void velocityControl::cameraCallback(const sensor_msgs::ImageConstPtr msg)
{
  // Convert ROS message to opencv Mat
  cv_bridge::CvImageConstPtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  Mat src = cv_ptr->image;

  // Show image to show it's working
  imshow("image", src);
  waitKey(33);

  return;
}

void velocityControl::estimateCallback(const nav_msgs::Odometry)
{
  // Save off estimate for vision processing
  return;
}

void velocityControl::publishVelocity()
{
  velocity_pub_.publish(velocity_measurement_);
}

} // namespace ekf


