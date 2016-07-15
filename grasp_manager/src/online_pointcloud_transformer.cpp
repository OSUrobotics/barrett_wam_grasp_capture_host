#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/Empty.h>

#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "pcl_to_ros.h"

// Globals required for data transfer
ros::Publisher *pub = NULL;
tf::TransformListener *listener;
sensor_msgs::PointCloud2::ConstPtr cloud_msg;
tf::StampedTransform cam_T;
bool cam_T_set = false;
std::string target_frame;


void new_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//ROS_INFO("Got cloud in online transformer.");
	cloud_msg = msg;
}

void transform_cloud_cb(const std_msgs::Empty::ConstPtr& msg)
{
	if (cloud_msg && cam_T_set) {
		ROS_INFO("Transforming cloud.");
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZRGB>), in_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		fromROSMsg(*cloud_msg, *in_cloud);
		pcl_ros::transformPointCloud(*in_cloud, *out_cloud, cam_T);
		out_cloud->header.frame_id = target_frame;
		pub->publish(out_cloud);
	} else if (!cloud_msg) {
		ROS_WARN("No cloud to transform.");
	} else {
		ROS_WARN("No transform to transform cloud.");
	}

}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "online_pointcloud_transformer");
	ros::NodeHandle nh("~");

	tf::TransformListener _listener;
	listener = &_listener;

	ros::Publisher _pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
	pub = &_pub;
	ros::Subscriber sub = nh.subscribe("input", 1, new_cloud_cb);
	ros::Subscriber trigger_sub = nh.subscribe("trigger", 1, transform_cloud_cb);
	if (!nh.getParam("target_frame", target_frame)) {
		ROS_ERROR("Could not get target_frame parameter.");
		return -1;
	}

	ros::AsyncSpinner spinny(1);
	spinny.start();
	ros::Rate rate(1.0);
	while (ros::ok()){
		try {
			if (cloud_msg) {
				listener->lookupTransform(target_frame.c_str(), cloud_msg->header.frame_id.c_str(), ros::Time(0), cam_T);
				cam_T_set = true;
			} else {
				ROS_WARN("No camera message to align.");
			}
		} catch (tf::TransformException ex) {
			ROS_WARN("Could not get cam transform.");
		}
		rate.sleep();
	}
}
