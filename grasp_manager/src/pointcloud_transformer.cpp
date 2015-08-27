#include <ros/ros.h>
#include "grasp_manager/TransformCloud.h"
#include "pcl/ros/conversions.h"
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <tf/transform_broadcaster.h>

#include <string.h>

bool transform_cloud_handler(grasp_manager::TransformCloud::Request& req, grasp_manager::TransformCloud::Response& res)
{
	// Create transform from pose
	tf::Quaternion marker_quat(req.marker_pose.orientation.x, req.marker_pose.orientation.y, req.marker_pose.orientation.z, req.marker_pose.orientation.w);
	tf::Vector3 marker_pos(req.marker_pose.position.x, req.marker_pose.position.y, req.marker_pose.position.z);
	tf::Transform marker_transform(marker_quat, marker_pos);
	marker_transform = marker_transform.inverse();

	// Reorient pointcloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr intermediate_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::moveFromROSMsg(req.cloud, *intermediate_cloud);
	pcl_ros::transformPointCloud(*intermediate_cloud, *out_cloud, marker_transform);
	pcl::toROSMsg(*out_cloud, res.cloud);

	// Adjust the header appropriately
	res.cloud.header.frame_id = "/robot_marker";

	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "pointcloud_transform");
	ros::NodeHandle nh;
	ROS_INFO("Pointcloud transformer marker online.");

	// Establish simple pointcloud reorientation service
	ros::ServiceServer service = nh.advertiseService("transform_pointcloud", transform_cloud_handler);
	ROS_INFO("Pointcloud Transform online.");
	ros::spin();

	return EXIT_SUCCESS;
}
