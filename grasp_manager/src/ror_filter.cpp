// This file implements a radius outlier removal, which will remove clusters of points from a pointcloud based on the number of neighbors they have. This ought to remove many miscellaneous clusters in the realsense data that could cause spurious ICP matches/features
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include "pcl_ros/filters/filter.h"

#include "pcl_to_ros.h"

ros::Publisher* pub;

void rorfilter_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) 
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = fromROSMsg(msg);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter;
	rorfilter.setInputCloud (cloud_in);
	rorfilter.setRadiusSearch (0.01);
	rorfilter.setMinNeighborsInRadius (3);
	rorfilter.filter (*cloud_out);

	sensor_msgs::PointCloud2::Ptr cloud = toROSMsg(*cloud_out);
	pub->publish(cloud);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "rorfilter");
	ros::NodeHandle nh("~");
	
	ros::Subscriber sub = nh.subscribe("in_topic", 1, rorfilter_cb);
	ros::Publisher _pub = nh.advertise<sensor_msgs::PointCloud2>("out_topic", 5);
	pub = &_pub;
	
	ros::spin();
	return EXIT_SUCCESS;
}
