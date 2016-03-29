#include "mono_vo/mono_vo.h"
#include <iostream>

namespace mono_vo
{

monoVO::monoVO() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~/mono_vo"))
{
  // Get Parameters from Server
  // arguments are "name", "variable to put the value into", "default value"
  nh_private_.param<int>("GFTT_maxCorners", GFTT_params_.max_corners, 200);
  nh_private_.param<double>("GFTT_qualityLevel", GFTT_params_.quality_level, 0.01);
  nh_private_.param<double>("GFTT_minDist", GFTT_params_.min_dist, 5);
  nh_private_.param<int>("GFTT_blockSize", GFTT_params_.block_size, 3);
  nh_private_.param<int>("LK_winSize", LK_params_.win_size, 31);
  nh_private_.param<int>("LK_maxLevel", LK_params_.win_level, 3);
  nh_private_.param<int>("LK_iterations", LK_params_.iters, 30);
  nh_private_.param<double>("LK_accuracy", LK_params_.accuracy, 0.01);
  nh_private_.param<int>("LC_radius", LC_params_.radius, 5);
  nh_private_.param<int>("LC_thickness", LC_params_.thickness, 3);
  nh_private_.param<double>("FH_ransacReprojThreshold", FH_params_.rancsace_reproj_threshold, 3);
  nh_private_.param<int>("FH_maxIters", FH_params_.max_iters, 2000);
  nh_private_.param<double>("FH_confidence", FH_params_.confidence, 0.995);
  nh_private_.param<bool>("no_normal_estimate", no_normal_estimate_, false);

  // Setup publishers and subscribers
  camera_sub_ = nh_.subscribe("/image_mono", 1, &monoVO::cameraCallback, this);
  estimate_sub_ = nh_.subscribe("/shredder/ground_truth/odometry", 1, &monoVO::estimateCallback, this);
  velocity_pub_ = nh_.advertise<geometry_msgs::Vector3>("velocity", 1);
  flow_image_pub_ = nh_.advertise<sensor_msgs::Image>("optical_flow", 1);

  // Initialize Filters and other class variables
  optical_flow_velocity_ = (Mat_<double>(3,1) << 0, 0, 0);

  optical_center_ = Point(320,240);
  focal_length_ = Point(205.46963709898583,  205.46963709898583);

  I_ = (Mat_ <double>(3,3) <<
        205.46963709898583, 0.0000000000,   320,
        0.0000000000,   205.46963709898583, 240,
        0.0000000000,   0.0000000000,   1.0000000000);
  D_ = (Mat_ <double>(5,1) <<
        0.0000000000,
        0.0000000000,
        0.0000000000,
        0.0000000000,
        0.0000000000);
  return;
}


void monoVO::cameraCallback(const sensor_msgs::ImageConstPtr msg)
{
  static bool initializing(true);
  static double prev_time(0);

  // Convert ROS message to opencv Mat
  cv_bridge::CvImageConstPtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  Mat src = cv_ptr->image;

  // Pull Out Current State
  // Remember that when using Gazebo, current state will be in NWU,
  // but when actually flying, it will be NED
  double pd = current_state_.pose.pose.position.z;
  double vel_x = current_state_.twist.twist.linear.x;
  double vel_y = -1.0*current_state_.twist.twist.linear.y;
  double phi = current_state_.pose.pose.orientation.x;
  double theta = -1.0*current_state_.pose.pose.orientation.y;
  double psi = -1.0*current_state_.pose.pose.orientation.z;
  double p = current_state_.twist.twist.angular.x;
  double q = -1.0*current_state_.twist.twist.angular.y;
  double r = -1.0*current_state_.twist.twist.angular.z;

  // Calculate Inertial Normal Vector
  Mat N_inertial = (Mat_<double>(3,1) <<  0, 0, -1);
  Mat N_c = inertialToCamera(N_inertial, phi, theta);

  // Initialize output Mat
  Mat dst;

  // points_[0] are the points from the previous frame
  // points_[1] are the points from the current frame

  if(initializing){
    goodFeaturesToTrack(src, points_[1], GFTT_params_.max_corners, 0.01, 10, Mat(), 3, 0, 0.04);
    cornerSubPix(src, points_[1], Size(11,11), Size(-1,-1),
        TermCriteria(TermCriteria::COUNT|TermCriteria::EPS,20,0.03));
    undistortPoints(points_[1], undistorted_points_[1], I_, D_);
    unNormalize(undistorted_points_[1], Size(640,480), optical_center_);
    initializing = false;
    prev_time = msg->header.stamp.toSec();
    cout << "initialized" << endl;
  }else if(!points_[0].empty()){
    vector<uchar> status;
    vector<float> err;
    if(prev_src_.empty()){
      src.copyTo(prev_src_);
    }
    calcOpticalFlowPyrLK(prev_src_, src, points_[0], points_[1], status, err,
        Size(31,31), 3, TermCriteria(TermCriteria::COUNT|TermCriteria::EPS,20,0.03),
        0, 0.001);

    vector<uchar> inliers;
    Mat F = findFundamentalMat(points_[0], points_[1], inliers);

    // Copy previous image onto the output in color.  Color is for plotting motion
    cvtColor(prev_src_, dst, COLOR_GRAY2BGR);

    // Go through points, and draw correspondences.  both points_ vectors will
    // reduce in size, and only have points that found correspondences in both
    // images we can use this to track points across multiple images in the future
    int j, k;
    for( j = k = 0; j < points_[1].size(); j++ ){
      if( status[j] && inliers[j] ){
        stringstream text;
        text << k;
        points_[0][k] = points_[0][j];
        points_[1][k] = points_[1][j];
//        stringstream ss;
//        ss << j;
//        putText(dst, ss.str().c_str(), points_[1][j], FONT_HERSHEY_COMPLEX, 1.0, Scalar(255,255,0));
        circle( dst, points_[1][k], 2, Scalar(0,0,255), -1, 1);
        circle( dst, points_[0][k], 1, Scalar(0,255,0), -1, 1);
        //putText( dst, text.str().c_str(), points_[0][k], FONT_HERSHEY_PLAIN, 1.5, Scalar(255, 0, 255));
        line(dst, points_[1][j], points_[0][j], Scalar(0,0,255));
        k++;
      }
    }
    circle( dst, optical_center_, 5, Scalar(0,255,255), -1, 1);
    points_[0].resize(k);
    points_[1].resize(k);

    // publish the image
    cv_bridge::CvImage out_msg;
    out_msg.header = cv_ptr->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = dst;
    flow_image_pub_.publish(out_msg.toImageMsg());

    // Undistort Points using Camera Parameters
    undistortPoints(points_[1], undistorted_points_[1], I_, D_);
    unNormalize(undistorted_points_[1], Size(640,480), optical_center_);

    // Calculate Essential Matrix
    double focal = (focal_length_.x + focal_length_.y)/2.0;
    Mat E = findEssentialMat(points_[0], points_[1], focal, optical_center_);

    Mat R, T;
    recoverPose(E, points_[0], points_[1], R, T, focal, optical_center_);
    cout << "R = \n" << R << endl;
    cout << "T = \n" << T << endl;


    // get more corners to make up for lost corners
   goodFeaturesToTrack(src, points_[1], GFTT_params_.max_corners, 0.01, 10, Mat(), 3, 0, 0.04);
   cornerSubPix(src, points_[1], Size(11,11), Size(-1,-1),
          TermCriteria(TermCriteria::COUNT|TermCriteria::EPS,20,0.03));
  }else{
    cout << "empty" << endl;
  }
  // save off points for next loop
  std::swap(points_[1], points_[0]);
  std::swap(undistorted_points_[1], undistorted_points_[0]);
  cv::swap(prev_src_, src);

  //Store the resulting measurement in the geometry_msgs::Vector3 velocity_measurement.
  velocity_measurement_.x = optical_flow_velocity_.at<double>(0,0);
  velocity_measurement_.y = optical_flow_velocity_.at<double>(1,0);
  velocity_measurement_.z = optical_flow_velocity_.at<double>(2,0);

  // publish the velocity measurement whenever you're finished processing
  publishVelocity();
  return;
}

void monoVO::estimateCallback(const nav_msgs::Odometry msg)
{
  current_state_ = msg;
  return;
}


void monoVO::publishVelocity()
{
  velocity_pub_.publish(velocity_measurement_);
}


Mat monoVO::skewSymmetric(Mat m){
  double x = m.at<double>(0);
  double y = m.at<double>(1);
  double z = m.at<double>(2);

  Mat out = (Mat_ <double>(3,3)
             << 0, -z, y,
                z, 0, -x,
                -y, x, 0);
  return out;
}


Mat monoVO::inertialToCamera(Mat v, double phi, double theta){
   Mat R_v1_to_v2 = (Mat_<double>(3,3) <<
                     cos(theta),  0, -sin(theta),
                     0,           1, 0,
                     sin(theta),  0, cos(theta) );
   Mat R_v2_to_b = (Mat_<double>(3,3) <<
                    1,  0,         0,
                    0,  cos(phi),  sin(phi),
                    0, -sin(phi),  cos(phi) );
   Mat R_b_to_c = (Mat_<double>(3,3) <<
                   0,  1,  0,
                   -1, 0,  0,
                   0,  0,  1 );
   return R_b_to_c*R_v2_to_b*R_v1_to_v2*v;
}

void monoVO::unNormalize(vector<Point2d> & points, Size size, Point2d center){
  for(int i = 0; i<points.size(); i++){
    points[i].x = center.x + points[i].x*size.width/2.0;
    points[i].y = center.y + points[i].y*size.width/2.0;
  }
}

} // namespace mono_vo





