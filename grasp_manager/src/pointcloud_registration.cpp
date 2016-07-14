/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier
 * further adaptation Jackson Carter*/

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include "pcl_to_ros.h"

#include <tf/transform_listener.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables 
//our visualizer
pcl::visualization::PCLVisualizer *p;
//its left and right viewports
int vp_1, vp_2;

// Function Prototypes
void align();
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false);

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
	public:
	MyPointRepresentation ()
	{
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray (const PointNormalT &p, float * out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

// Globals required for data transfer
ros::Publisher *pub = NULL;
tf::TransformListener *listener;
pcl::PointCloud<pcl::PointXYZ>::Ptr cur_msg1;
pcl::PointCloud<pcl::PointXYZ>::Ptr cur_msg2;
tf::StampedTransform cam1_T;
bool cam1_T_set = false;
tf::StampedTransform cam2_T;
bool cam2_T_set = false;

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
 * \param argc the number of arguments (pass from main ())
 * \param argv the actual command line arguments (pass from main ())
 * \param models the resultant vector of point cloud datasets
 */
/*void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
  {
  std::string extension (".pcd");
// Suppose the first argument is the actual test model
for (int i = 1; i < argc; i++)
{
std::string fname = std::string (argv[i]);
// Needs to be at least 5: .plot
if (fname.size () <= extension.size ())
continue;

std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

//check that the argument is a pcd file
if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
{
// Load the cloud and saves it into the global list of models
PCD m;
m.f_name = argv[i];
pcl::io::loadPCDFile (argv[i], *m.cloud);
//remove NAN points from the cloud
std::vector<int> indices;
pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

models.push_back (m);
}
}
}
*/

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
	p->removePointCloud ("vp1_target");
	p->removePointCloud ("vp1_source");

	PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
	p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
	p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

	PCL_INFO ("Press q to begin the registration.\n");
	p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
	p->removePointCloud ("source");
	p->removePointCloud ("target");


	PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
	if (!tgt_color_handler.isCapable ())
		PCL_WARN ("Cannot create curvature color handler!");

	PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
	if (!src_color_handler.isCapable ())
		PCL_WARN ("Cannot create curvature color handler!");


	p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
	p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

	p->spinOnce();
}


bool align_is_possible()
{
	return (cur_msg1 != NULL && cur_msg2 != NULL && cam1_T_set && cam2_T_set);
}

void new_cloud_cb1(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO("Got cloud 1 in registrar.");
	cur_msg1 = fromROSMsg(msg);
	if (align_is_possible()) {
		align();
	}
}

void new_cloud_cb2(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO("Got cloud 2 in registrar.");
	cur_msg2 = fromROSMsg(msg);
	if (align_is_possible()) {
		align();
	}
}

void align()
{
	PointCloud::Ptr result (new PointCloud);
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

	// Add visualization data
	//showCloudsLeft(source, target);

	ROS_INFO("Before pointcloud transformations.");
	PointCloud::Ptr temp (new PointCloud);
	std::string reference_frame("/ar_marker_4");
	pcl_ros::transformPointCloud(*cur_msg1, *temp, cam1_T);
	*cur_msg1 = *temp;
	pcl_ros::transformPointCloud(*cur_msg2, *temp, cam2_T);
	*cur_msg2 = *temp;
	pairAlign (cur_msg1, cur_msg2, temp, pairTransform, true);
	ROS_INFO("After pointcloud transformations.");

	//transform current pair into the global transform
	pcl::transformPointCloud (*temp, *result, GlobalTransform);

	//update the global transform
	GlobalTransform = GlobalTransform * pairTransform;

	//save aligned pair, transformed into the first cloud's frame

	/*std::stringstream ss;
	  ss << i << ".pcd";
	  pcl::io::savePCDFile (ss.str (), *result, true);*/

}

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
 * \param cloud_src the source PointCloud
 * \param cloud_tgt the target PointCloud
 * \param output the resultant aligned source PointCloud
 * \param final_transform the resultant transform between source and target
 */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)
{
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	PointCloud::Ptr src (new PointCloud);
	PointCloud::Ptr tgt (new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	if (downsample)
	{
		grid.setLeafSize (0.03, 0.03, 0.03);
		grid.setInputCloud (cloud_src);
		grid.filter (*src);

		grid.setInputCloud (cloud_tgt);
		grid.filter (*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}


	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch (30);

	norm_est.setInputCloud (src);
	norm_est.compute (*points_with_normals_src);
	pcl::copyPointCloud (*src, *points_with_normals_src);

	norm_est.setInputCloud (tgt);
	norm_est.compute (*points_with_normals_tgt);
	pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues (alpha);

	//
	// Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	reg.setTransformationEpsilon (1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance (0.05);  
	// Set the point representation
	reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

	reg.setInputSource (points_with_normals_src);
	reg.setInputTarget (points_with_normals_tgt);



	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations (2);
	for (int i = 0; i < 30; ++i)
	{
		if (!ros::ok())
			break;
		PCL_INFO ("Iteration Nr. %d.\n", i);

		// save cloud for visualization purpose
		points_with_normals_src = reg_result;

		// Estimate
		reg.setInputSource (points_with_normals_src);
		reg.align (*reg_result);

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation () * Ti;
		std::cout << "Registrar. Current Transform: " << Ti << std::endl;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
			reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

		prev = reg.getLastIncrementalTransformation ();

		// visualize current state
		showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}

	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	//
	// Transform target back in source frame
	pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

	p->removePointCloud ("source");
	p->removePointCloud ("target");

	PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
	PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
	p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
	p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

	PCL_INFO ("Press q to continue the registration.\n");
	p->spin ();

	p->removePointCloud ("source"); 
	p->removePointCloud ("target");

	//add the source to the transformed target
	*output += *cloud_src;

	final_transform = targetToSource;
	sensor_msgs::PointCloud2::Ptr msg = toROSMsg(*output);
	pub->publish(msg);
	ROS_INFO("Finished Alignment.");
}


/* ---[ */
int main (int argc, char** argv)
{
	// Load data
	/*std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
	  loadData (argc, argv, data);*/
	ros::init(argc, argv, "pointcloud_registrar");
	ros::NodeHandle nh("~");

//	ros::Duration extrapolation_limit(2.0);
	tf::TransformListener _listener;
//	_listener.setExtrapolationLimit(extrapolation_limit);
	listener = &_listener;

	//TODO: Change this to a service interface, that's really what we need
	ros::Publisher _pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
	pub = &_pub;
	ros::Subscriber sub1 = nh.subscribe("input1", 1, new_cloud_cb1);
	ros::Subscriber sub2 = nh.subscribe("input2", 1, new_cloud_cb2);

	// Create a PCLVisualizer object
	p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
	p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);


	//ros::spin();
	ros::AsyncSpinner spinny(2);
	spinny.start();
	ros::Rate rate(2.0);
	while (ros::ok()){
		try {
			listener->lookupTransform("/ar_marker_4", "/camera1_depth_optical_frame", ros::Time(0), cam1_T);
			cam1_T_set = true;
		} catch (tf::TransformException ex){
			ROS_WARN("Could not acquire cam1 transform");
		} 

		try {
			listener->lookupTransform("/ar_marker_4", "/camera2_depth_optical_frame", ros::Time(0), cam2_T);
			cam2_T_set = true;
		} catch (tf::TransformException ex) {
			ROS_WARN("Could not get cam2 transform.");
		}
		rate.sleep();
	}
}
