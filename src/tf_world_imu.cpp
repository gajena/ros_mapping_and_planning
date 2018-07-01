#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>


nav_msgs::Odometry odo_;
bool odo_update=false;

void odomCb(const nav_msgs::Odometry msg)
{
  odo_=msg;
  odo_update=true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf2_broadcaster");

  ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/rovio/pose_with_covariance_stamped",10,odomCb);
  ros::Publisher map_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/mavros/vision_pose/pose_cov", 10);


  ros::spin();
  return 0;
}