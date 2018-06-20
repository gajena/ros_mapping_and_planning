
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/OccupancyGrid.h>

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
   	ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 10);

  	int i = 0,j=5,local_sum[160],sum_=0;
  	long long int k=0,kk=0;
  	float sum[752]={0};
	nav_msgs::OccupancyGrid map_;  
  	while (nh.ok())
	{
		if(depth_est.size().width>1)
		{		
	  		for (int iii = 0; iii <752 ; iii++)
  			{
    			for(int jjj=0;jjj<160;jjj++)
    			{
    				if ((int)depth_est.at<uchar>(jjj, iii)>100)
    				{
        				depth_est.at<uchar>(jjj,iii) = 255;
        				sum_=sum_+2;
    				}
      				else
      				{	
      					sum_=sum_-1;
        				depth_est.at<uchar>(jjj,iii) = 0;
        			}
    			}
    			if(sum_>0)
    			sum[iii]=0;
    			else
    			sum[iii]=99;
    			sum_=0;
  			}
  			map_.header.stamp = ros::Time::now();
        	map_.header.frame_id = "map";
        	map_.info.resolution = 0.05f;
        	map_.info.height = 60;
        	map_.info.width = 47;
        	map_.info.origin.position.x=-1.175;
        	map_.info.origin.position.y= 0;
        	for(int ii=0;ii<60;ii++)
        	{
        		int count=0;
        		for(int jj=0;jj<47;jj++)
        		{
        			map_.data.push_back(sum[jj*16]);
        			count++;
				}
			}
		}

		map_pub.publish(map_);
		map_.data.clear();

		ros::spinOnce();
    	loop_rate.sleep();
	}
  return 0;
}