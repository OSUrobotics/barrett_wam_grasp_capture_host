#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr fromROSMsg(const sensor_msgs::PointCloud2::ConstPtr& msg);
sensor_msgs::PointCloud2::Ptr toROSMsg(pcl::PointCloud<pcl::PointXYZ>& cloud);
sensor_msgs::PointCloud2::Ptr toROSMsg(pcl::PointCloud<pcl::PointNormal>& cloud);
