#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <math.h>

using namespace cv;

#define PI 3.14159265
 
Mat depth_est=Mat::zeros(160,752, CV_8UC1);;

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
  			depth_est.at<uchar>(a,b) = cv_ptr->image.at<float>(a,b)*40;
  		}
  	}
}

nav_msgs::Odometry odo_;
bool odo_update=false;

Mat toMat(const nav_msgs::OccupancyGrid map);

const nav_msgs::OccupancyGrid toMap(Mat image);

void odomCb(const nav_msgs::Odometry msg)
{
	odo_=msg;
	odo_update=true;
}

Mat rotate(Mat src, double angle)
{
    Mat dst;
    Point2f pt(src.cols/2, src.rows/2);    
    Mat r = getRotationMatrix2D(pt, angle, 1.0);
    warpAffine(src, dst, r, Size(src.cols, src.rows));
    return dst;
}

int main(int argc, char**argv)
{
  
	ros::init(argc, argv, "global_map");
	ros::NodeHandle nh;
  	ros::Rate loop_rate(10);
	ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/flame/depth_registered/image_rect",10,imageCb);
   	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/rovio/odometry",10,odomCb);
   	ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 10);

  	int count=0,trig=0,local_sum[160],sum_=0;
  	long long int k=0,kk=751;
  	float sum[752]={0};
  	std::vector<signed char> v;
	nav_msgs::OccupancyGrid map_,map_temp;
	double yaw_init, yaw_set = 0;
			int yaw_reset = 1;         
	Mat  img_temp,img;

  	while (nh.ok())
	{

		// nh.getParam("/vin_mission_control/yaw_reset", yaw_reset);

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
				sum[kk]=(sum_/160);
				kk=kk-1;
    			sum_=0;
  			}
			
			kk=751;

			map_temp.header.stamp = ros::Time::now();
        	map_temp.header.frame_id = "/imu";
        	map_temp.info.resolution = 0.05f;
        	map_temp.info.height = 40;
        	map_temp.info.width = 47;
        	map_temp.info.origin.position.x=0;
        	map_temp.info.origin.position.y=1.175;
			map_temp.info.origin.orientation.w=0.707388;
			map_temp.info.origin.orientation.z=-0.706825;
			
			for(int ii=0;ii<40;ii++)
        	{
        		for(int jj=0;jj<47;jj++)
        		{
        			
        			if(sum[jj*16]<100)
        			{
        				if(ii<100-(sum[jj*16]))
        					map_temp.data.push_back(0);
        				else
        					map_temp.data.push_back(100);
        			}
        			else
        				map_temp.data.push_back(sum[jj*16]);
				}
			}

			img_temp = toMat(map_temp);
			map_temp.data.clear();
  		
			map_.header.stamp = ros::Time::now();
        	map_.header.frame_id = "world";
        	map_.info.resolution = 0.05f;
        	map_.info.height = 1000;
        	map_.info.width = 1000;
        	map_.info.origin.position.x = -(map_.info.height/40.0f);
        	map_.info.origin.position.y = -(map_.info.width/40.0f);
			float init_x = 20*(odo_.pose.pose.position.x - (map_.info.origin.position.x));
			float init_y = 20*(odo_.pose.pose.position.y - (map_.info.origin.position.y));
			tf::Quaternion q1(
            	odo_.pose.pose.orientation.x,
                odo_.pose.pose.orientation.y,
                odo_.pose.pose.orientation.z,
                odo_.pose.pose.orientation.w);
        
			tf::Matrix3x3 m1(q1);

			double r, p, yaw_odo;
			m1.getRPY(r, p, yaw_odo);
			
			if(yaw_reset == 1 && 0==std::isnan(yaw_odo))
			{
				yaw_init = yaw_odo;
				// nh.setParam("/vin_mission_control/yaw_reset", 0);
				yaw_reset = 0;
			}

			yaw_set = yaw_init - yaw_odo;
			if(std::isnan(yaw_set))
	      		 yaw_set=0;
			

			for(int ii=0;ii<1000 && count == 0;ii++)
        	{
        		for(int jj=0;jj<1000;jj++)
        		{
       				v.push_back(-1);
				}
			}

			map_.data = v;
			// v.clear();
			if(count == 0)
			{
				img = toMat(map_);
				img =  rotate(img,270);
				flip(img,img,1);
			}
			
			count = count + 1; 
			img_temp = rotate(img_temp,-yaw_set* 180 / PI);

		
			float img_temp_pos_x = odo_.pose.pose.position.x;
			float img_temp_pos_y = odo_.pose.pose.position.y;
		

			for(int i=0; i<img_temp.rows-6; i++) 
			{
				for(int j=0; j<img_temp.cols; j++) 
				{	
					if(img_temp.at<uchar>(i,j)>140.0)
						img.at<uchar>(479+i-(int)(odo_.pose.pose.position.x*20),474+j-(int)(odo_.pose.pose.position.y*20)) = img_temp.at<uchar>(i,j);
				}
			}

			
			for(int i=0; i<img.rows; i++) 
			{
				for(int j=0; j<img.cols; j++) 
				{
					if(img.at<uchar>(i,j)>160)
						map_.data.at((1000-1-i)+(1000-1-j)*(1000))=0;
					else if ( img.at<uchar>(i,j)<160 && img.at<uchar>(i,j)>140 )
						map_.data.at((1000-1-i)+(1000-1-j)*(1000))=100;
					else
						map_.data.at((1000-1-i)+(1000-1-j)*(1000))=-1;
				}
			}
								
			// v = toMap(img).data;
			map_pub.publish(map_);
		
			map_.data.clear();
			ros::spinOnce();
    		loop_rate.sleep();
		}
	}
  return 0;
}


Mat toMat(const nav_msgs::OccupancyGrid map)
{
  uint8_t *data = (uint8_t*) map.data.data(),
           testpoint = data[0];
  bool mapHasPoints = false;

  Mat im(map.info.height, map.info.width, CV_8UC1);

  // transform the map in the same way the map_saver component does
  for (size_t i=0; i<map.data.size(); i++)
  {
    if (data[i] == 0)        im.data[map.data.size()-i-1] = 254;
    else if (data[i] == 100) im.data[map.data.size()-i-1] = 150;
    else im.data[map.data.size()-i-1] = 50;

    // just check if there is actually something in the map
    if (i!=0) mapHasPoints = mapHasPoints || (data[i] != testpoint);
    testpoint = data[i];
  }

  // sanity check
  if (!mapHasPoints) { ROS_WARN("map is empty, ignoring update."); }

  return im;
}
