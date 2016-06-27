#include <ros/ros.h>
#include "grasp_data_msgs/GraspSnapshot.h"
#include "grasp_data_msgs/SaveCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/ros/conversions.h"
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <string.h>
#include <fstream>
#include <iostream>
#include <math.h>

using std::string;
using std::fstream;
using std::ofstream;
using std::endl;
using std::to_string;
using std::cout;

// Prototypes
void save_cloud(sensor_msgs::PointCloud2* cloud, string path);

bool save_cloud_handler(grasp_data_msgs::SaveCloud::Request& req,
			grasp_data_msgs::SaveCloud::Response& res)
{
	save_cloud(&(req.cloud), req.path);
	return true;
}

void save_cloud_cb(const grasp_data_msgs::GraspSnapshot::ConstPtr& msg)
{
	// Open pointcloud file
	string filename = string("obj") + to_string(msg->obj_num) + \
			  string("_sub") + to_string(msg->sub_num) + \
			  string("_grasp") + to_string(msg->grasp_num);
	if (msg->is_optimal) {
		filename += string("_optimal") + to_string(msg->optimal_num);
	} else {
		filename += string("_extreme") + to_string(msg->extreme_num);
	}
	filename += string("_pointcloud.csv");

	sensor_msgs::PointCloud2 c(msg->cloud_image);
	save_cloud(&c, filename);
}

void save_cloud(sensor_msgs::PointCloud2* cloud, string path)
{
	// Change pointcloud types
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::moveFromROSMsg(*cloud, *pcl_cloud);

	// Skip empty clouds (or really sparse clouds)
	if (pcl_cloud->points.size() < 100) {
		ROS_WARN("Pointcloud has insufficient points for saving (arbitrary limit). Skipping.");
		return;
	}
		

	ofstream pointcloud_file;
	pointcloud_file.open(path, fstream::out | fstream::trunc);
	if (pointcloud_file.fail() || !pointcloud_file.is_open()) {
		ROS_ERROR_STREAM("Cannot open pointcloud csv file for saving: " << path);
		return;
	}

	// Save points
	unsigned int num_pts = pcl_cloud->points.size();
	pcl::PointXYZRGB* pt;
	for (unsigned int i = 0; i < num_pts; ++i) {
		pt = &(pcl_cloud->at(i));
		if (isnan(pt->x) || isnan(pt->y) || isnan(pt->z)) {
			continue;
		}
		
		pointcloud_file << pt->x << "," << pt->y << "," << pt->z << "," 
				<< (unsigned int) pt->r << "," 
				<< (unsigned int) pt->g << "," 
				<< (unsigned int) pt->b << endl;
	}

	pointcloud_file.close();
	cout << "File saved." << endl;
	return;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "pointcloud_saver");
	ros::NodeHandle nh;
	cout << "Pointcloud saver node online!" << endl;

	// Establish simple pointcloud reorientation service
	//ros::Subscriber subby = nh.subscribe("/grasp_extremes", 1, save_cloud_cb);
	
	ros::ServiceServer service = nh.advertiseService("save_ptcloud_csv", save_cloud_handler);
	ros::spin();

	return EXIT_SUCCESS;
}
