#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include "pcl_to_ros.h"

using std::string;

ros::Publisher* pub;

void
poly_filter_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO("Polynomial Smoother got an input message.");

	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
	mls.setComputeNormals (true);

	// Set parameters
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = fromROSMsg(msg);
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.03);

	// Reconstruct
	mls.process (mls_points);

	// Output
	sensor_msgs::PointCloud2::Ptr out_cloud = toROSMsg(mls_points);
	pub->publish(out_cloud);
}
/*
string get_param_fatal(const char* param_name)
{
	if (!ros::param::has(param_name)) {
		ROS_ERROR_STREAM("Could not find parameter: " << param_name << "\nTerminating.");
		exit(1);
	}
	
	string out_str;
	if (!ros::param::get(param_name, out_str)) {
		ROS_ERROR_STREAM("Could not retrieve parameter: " << param_name << "\nTerminating.");
		exit(1);
	}
	
	return out_str;
}
*/

int
main (int argc, char** argv)
{
	// Initialize the ros interface
	ros::init(argc, argv, "polynomial_smoother");
	ros::NodeHandle nh("~");
	ROS_INFO("PCL Polynomial Smoother online!");

	//std::string in_topic = get_param_fatal("~in_topic");
	//std::string out_topic = get_param_fatal("~out_topic");

	ros::Subscriber sub = nh.subscribe("input", 1, poly_filter_cb);
	ros::Publisher _pub = nh.advertise<sensor_msgs::PointCloud2>("output", 5);
	pub = &_pub;

	ros::spin();
}
