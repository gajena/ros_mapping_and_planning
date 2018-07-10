#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#define PI 3.14159265


using namespace std;

int mission_reset_flag = 0,odom_detected_flag = 0;
float x = 0.0, y = 0.0;

void mission_reset_flagcb(const std_msgs::Int32::ConstPtr &msg)
{
    mission_reset_flag = msg->data;
}

void odomcb(const nav_msgs::Odometry::ConstPtr &msg)
{
    x = (msg->pose.pose.position.x);
    y = (msg->pose.pose.position.y);
    odom_detected_flag = 1;
}

geometry_msgs::PoseArray waypoints_;
geometry_msgs::Pose pose_;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_waypoint");
    ros::NodeHandle n;

    ros::Subscriber mission_reset_flag_sub = n.subscribe<std_msgs::Int32>("/mission_reset_flag", 10, mission_reset_flagcb);
	ros::Subscriber odom_sub = n.subscribe("/rovio/odometry", 10, odomcb);

    ros::Publisher setpoint_pub = n.advertise<geometry_msgs::PoseArray>("/waypoints", 10);

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
    	waypoints_.header.stamp = ros::Time::now();
        waypoints_.header.frame_id = "world";
    	if(mission_reset_flag == 0)
    	{
                for (int i =0 ; i<11 ;i++)
                {
                        int t=-i;
        	        pose_.position.x = 2-2*(fabs(cos(PI*t/10))*cos(PI*t/10) + fabs(sin(PI*t/10))*sin(PI*t/10));
                        pose_.position.y = -2+2*(fabs(cos(PI*t/10))*cos(PI*t/10) - fabs(sin(PI*t/10))*sin(PI*t/10));
                        waypoints_.poses.push_back(pose_);
                }
	    	setpoint_pub.publish(waypoints_);
	    	waypoints_.poses.clear();
        }


		ros::spinOnce();
    	loop_rate.sleep();
    }


    return 0;
}
