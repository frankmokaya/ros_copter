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

  optical_center_ = Point(320.5,240.5);
  focal_length_ = Point(205.46963709898583,  205.46963709898583);

  I_ = (Mat_ <double>(3,3) <<
           205.46963709898583, 0.0000000000,   320.5,
           0.0000000000,   205.46963709898583, 240.5,
           0.0000000000,   0.0000000000,   1.0000000);
  D_ = (Mat_ <double>(5,1) <<
            0,
            0,
            0,
            0,
            0);

  // Initialize Feature Matcher
  detector_ = ORB::create();
  detector_->setMaxFeatures(1000);
  matcher_ = DescriptorMatcher::create("BruteForce-Hamming");
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

  // Initialize output Mat
  Mat dst;
  cvtColor(src, dst, COLOR_GRAY2BGR);

  // points_[0] are the points from the previous frame
  // points_[1] are the points from the current frame

  if(initializing){
    detector_->detectAndCompute(src, noArray(), keypoints_[1], descriptors_[1]);
    cout << "initialized, kp size = " << (int)keypoints_[1].size() << endl;
    initializing = false;
  }
  else if(!keypoints_[0].empty()){
    detector_->detectAndCompute(src, noArray(), keypoints_[1], descriptors_[1]);
    vector<vector<DMatch>> matches;
    matcher_->knnMatch(descriptors_[0], descriptors_[1], matches, 2);
    matched_keypoints_[0].clear();
    matched_keypoints_[1].clear();
    for(int i = 0; i < matches.size(); i++)
    {
      if(matches[i][0].distance < nn_match_ratio_ * matches[i][1].distance) {
        matched_keypoints_[0].push_back(keypoints_[0][matches[i][0].queryIdx]);
        matched_keypoints_[1].push_back(keypoints_[1][matches[i][0].trainIdx]);
      }
    }
    // convert keypoints to point2d for findFundamentalMat
    points_[0].clear();
    points_[1].clear();
    for(int i = 0; i<keypoints_[0].size(); i++){
      points_[0].push_back(keypoints_[0][i].pt);
      points_[1].push_back(keypoints_[1][i].pt);
    }

    vector<uchar> inliers;
    Mat F = findFundamentalMat(points_[0], points_[1], inliers, FM_RANSAC, 3, 0.99);

    for(int j = 0; j < points_[1].size(); j++ ){
      if(!inliers[j]){
      }else{
        circle(dst, points_[1][j], 2, Scalar(0,0,255), -1, 1);
        circle(dst, points_[0][j], 2, Scalar(0,255,0 -1, 1));
        line(dst, points_[1][j], points_[0][j], Scalar(0,0,255));
      }
    }

    // publish the image
    cv_bridge::CvImage out_msg;
    out_msg.header = cv_ptr->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = dst;
    flow_image_pub_.publish(out_msg.toImageMsg());
  }else{
    cout << "empty" << endl;
  }
  // save off points for next loop
  std::swap(keypoints_[1], keypoints_[0]);
  cv::swap(descriptors_[1], descriptors_[0]);
  cv::swap(prev_src_, src);

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


void monoVO::unNormalize(vector<Point> & points, Size size, Point2d center){
  for(int i = 0; i<points.size(); i++){
    points[i].x = center.x + points[i].x*size.width/2.0;
    points[i].y = center.y + points[i].y*size.width/2.0;
  }
}

} // namespace mono_vo





