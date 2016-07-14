// File: object_segmentation.cpp
// Description: This file will remove the plane of the table from 
// 	the pointcloud results, leaving only the objects to be aligned and a 
// 	couple of random pieces of the scene. This should lead to a much 
// 	faster and more accurate ICP computation.
// Assumptions: The cloud fed in is from the camera's depth optical frame
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include "pcl_to_ros.h"

#define ZERO_THRESHOLD 0.01 // If all of a points' coordinates are less than this value, the point is removed

ros::Publisher *pub;

void filter_zeroes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	unsigned int n = cloud->points.size();
	pcl::ExtractIndices<pcl::PointXYZ> eifilter (true);
	pcl::PointIndices::Ptr non_zero_idxs (new pcl::PointIndices);
	for (unsigned int i =0; i < n; ++i) {
		if ((*cloud)[i].x < ZERO_THRESHOLD &&
			(*cloud)[i].y < ZERO_THRESHOLD &&
			(*cloud)[i].z < ZERO_THRESHOLD) {
			non_zero_idxs->indices.push_back(i);
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	eifilter.setInputCloud(cloud);
	eifilter.setIndices(non_zero_idxs);
	eifilter.filter(*out_cloud);
	std::cout << "PCL object segmentation non_zero pt count: " << out_cloud->points.size() << " original cloud points: " << cloud->points.size() << std::endl;
	*cloud = *out_cloud;
}

void remove_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	
	// Find and remove small clusters (small points and points at improper resolutions)
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.03);
	ec.setMinClusterSize (1);
	ec.setMaxClusterSize (50);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	// Extract the small clusters
	pcl::ExtractIndices<pcl::PointXYZ> cluster_filter;
	cluster_filter.setNegative(true);
	ROS_INFO("Reached Actual Euclidean Extraction");
	ROS_INFO_STREAM("Found " << cluster_indices.size() << " clusters. REMOVING!");
	pcl::IndicesPtr idxs(new std::vector<int>);
	for (std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
		for (std::vector<int>::iterator nit = it->indices.begin(); nit != it->indices.end(); ++nit) {
			idxs->push_back(*nit);
		}
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
	ROS_INFO("Prior to setIndices.");
	cluster_filter.setIndices(idxs);
	cluster_filter.setInputCloud(cloud);
	ROS_INFO("Prior to filtering.");
	cluster_filter.filter(*temp);
	*cloud = *temp;
	ROS_INFO("Extraction successful.");
}

void object_seg_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	if (msg->header.frame_id.find("optical_frame") == std::string::npos) {
		ROS_ERROR("Object segmentation node received a frame not from the optical frame of reference. Filtering could be troublesome. Skipping.");
		return;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = fromROSMsg(msg);
	//filter_zeroes(cloud);
	remove_clusters(cloud);
	std::cout << "cloud size: " << cloud->points.size() << std::endl;

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.02);

	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return;
	}

	/*std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " " 
		<< coefficients->values[3] << std::endl;

	std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
	for (size_t i = 0; i < inliers->indices.size (); ++i)
		std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
			<< cloud->points[inliers->indices[i]].y << " "
			<< cloud->points[inliers->indices[i]].z << std::endl;
	*/

	// Remove the plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> eifilter (true);
	eifilter.setNegative(true);
	eifilter.setInputCloud(cloud);
	eifilter.setIndices(inliers);
	eifilter.filter(*out_cloud);
	
	sensor_msgs::PointCloud2::Ptr out_msg = toROSMsg(*out_cloud);
	pub->publish(out_msg);
}

int main (int argc, char** argv) {
	ros::init(argc, argv, "object_segmentation");
	ros::NodeHandle nh("~");

	ros::Subscriber sub = nh.subscribe("input", 1, object_seg_cb);
	ros::Publisher _pub = nh.advertise<sensor_msgs::PointCloud2>("output", 5);
	pub = &_pub;
	ros::spin();

	return (0);
}
