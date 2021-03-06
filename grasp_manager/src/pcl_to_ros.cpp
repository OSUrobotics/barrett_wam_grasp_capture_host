#include "pcl_to_ros.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr fromROSMsg(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*msg, pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
	return cloud;	
}


sensor_msgs::PointCloud2::Ptr toROSMsg(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(cloud, *msg);
	return msg;	
}

sensor_msgs::PointCloud2::Ptr toROSMsg(pcl::PointCloud<pcl::PointNormal>& cloud)
{
	sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(cloud, *msg);
	return msg;	
}
