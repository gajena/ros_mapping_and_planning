
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

cv::Mat depth_est=cv::Mat::zeros(160,752, CV_8UC1);;

void imageCb(const sensor_msgs::Image msg)
{    
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

  	for(int a=0;a<160;a++)
  	{
  		for(int b=0;b<752;b++)
  		{
  			depth_est.at<uchar>(a,b) = cv_ptr->image.at<float>(a,b)*15;
  		}
  	}
}

nav_msgs::Odometry odo_;
bool odo_update=false;

void odomCb(const nav_msgs::Odometry msg)
{
	odo_=msg;
	odo_update=true;
}


int main(int argc, char** argv)
{
  /*****************************************************************************
   ros node stuff : like nodehandle 
   publisher for image topic
  *****************************************************************************/
	ros::init(argc, argv, "depth_map");
	ros::NodeHandle nh;
  	ros::Rate loop_rate(10);
	ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/flame/depth_registered/image_rect",10,imageCb);
   	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/rovio/odometry",10,odomCb);
   	ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 10);

  	int count=0,trig=0,local_sum[160],sum_=0;
  	long long int k=0,kk=751;
  	float sum[752]={0};
  	std::vector<signed char> v;
	nav_msgs::OccupancyGrid map_;

  	while (nh.ok())
	{
		if(depth_est.size().width>1)
		{		
	  		for (int iii = 0; iii <752 ; iii++)
  			{
    			for(int jjj=0;jjj<160;jjj++)
    			{
    				if ((int)depth_est.at<uchar>(jjj, iii)>150)
    				{
        				sum_=sum_+0;
    				}
      				else if ((int)depth_est.at<uchar>(jjj, iii)<50)
      				{	
      					sum_=sum_+100;
        			}
        			else
        			{
        				sum_=sum_-((int)depth_est.at<uchar>(jjj, iii)-150);
        			}
    			}
    			std::cout<<kk<<" , "<<iii<<std::endl;
				sum[kk]=(sum_/160);
				kk=kk-1;
    			sum_=0;
  			}
			kk=751;
  			map_.header.stamp = ros::Time::now();
        	map_.header.frame_id = "/imu";
        	map_.info.resolution = 0.05f;
        	map_.info.height = 50;
        	map_.info.width = 47;
        	map_.info.origin.position.x=0;
        	map_.info.origin.position.y=1.175;
			map_.info.origin.orientation.w=0.707388;
			map_.info.origin.orientation.z=-0.706825;
			// map_.info.origin.orientation.y = 0.9999997;
			// map_.info.origin.orientation.w = 0.0007963;
			// map_.info.origin.orientation.y=0;
			// map_.info.origin.orientation.x=-0;
			// map_.info.origin.orientation.x = 0;
			// map_.info.origin.orientation.w = 1;
			// 0.707388, 0.706825, 0.0005629, 0.0005633
			// 0.707388, -0.706825, -0.0005629, 0.0005633
			for(int ii=0;ii<40;ii++)
        	{
        		for(int jj=0;jj<47;jj++)
        		{
        			if(ii==59 || jj==0 || jj==46)
        					v.push_back(0);
        			else if(sum[jj*16]<100)
        			{
        				if(ii<100-(sum[jj*16]))
        					v.push_back(0);
        				else
        					v.push_back(100);
        			}
        			else
        				v.push_back(sum[jj*16]);
				}
			}
		}

		if(count==0)
		{
			map_.data = v;
				count=count+1;
		}
		if(count!=0 && odo_update==true)
		{
			map_.data.resize(47*1*(count));
			// std::cout<<.size()<<std::endl;
			for(int i=0;i<v.size();i++)
			map_.data.push_back(v[i]);
			v.clear();
			count=count+1;
			if(count==11)
			{	
			map_pub.publish(map_);
			count=0;
			trig=1;
			}
			odo_update=false;
		}
		ros::spinOnce();
    	loop_rate.sleep();
	}
  return 0;
}
