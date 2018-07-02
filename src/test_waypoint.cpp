#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32.h>


using namespace std;

int mission_reset_flag = 0;

void mission_reset_flagcb(const std_msgs::Int32::ConstPtr &msg)
{
    mission_reset_flag = msg->data;
}

geometry_msgs::PoseArray waypoints_;
geometry_msgs::Pose pose_;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_waypoint");
    ros::NodeHandle n;

    ros::Subscriber mission_reset_flag_sub = n.subscribe<std_msgs::Int32>("/mission_reset_flag", 10, mission_reset_flagcb);

    ros::Publisher setpoint_pub = n.advertise<geometry_msgs::PoseArray>("/waypoints", 10);

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
    	waypoints_.header.stamp = ros::Time::now();
        waypoints_.header.frame_id = "world";
    	if(mission_reset_flag == 0)
    	{
    		pose_.position.x = 0;
	    	pose_.position.y = 0;
	    	waypoints_.poses.push_back(pose_);
	    	pose_.position.x = 0;
	    	pose_.position.y = -2;
	    	waypoints_.poses.push_back(pose_);

	    	setpoint_pub.publish(waypoints_);
	    	waypoints_.poses.clear();
		}
		else if(mission_reset_flag == 1)
    	{
    		pose_.position.x = 0;
	    	pose_.position.y = -2;
	    	waypoints_.poses.push_back(pose_);
	    	pose_.position.x = 0;
	    	pose_.position.y = 0;
	    	waypoints_.poses.push_back(pose_);

	    	setpoint_pub.publish(waypoints_);
	    	waypoints_.poses.clear();
		}
		ros::spinOnce();
    	loop_rate.sleep();
    }


    return 0;
}