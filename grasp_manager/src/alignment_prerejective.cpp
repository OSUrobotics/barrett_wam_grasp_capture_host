#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_to_ros.h"

#include <string.h>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <pcl/io/vtk_lib_io.h>

#include "mesh_sampler.h"

#define NUM_SAMPLE_POINTS 2000

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

PointCloudT::Ptr obj_cloud;
ros::Publisher *obj_pub;

void estimate_pose_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	ROS_INFO("Got a scene.");
	if (!obj_cloud) {
		ROS_INFO("Without a cloud... Cannot perform alignment.");
		return;
	}
	
	// Point clouds
	PointCloudT::Ptr object (new PointCloudT);
	PointCloudT::Ptr object_aligned (new PointCloudT);
	PointCloudT::Ptr scene (new PointCloudT);
	FeatureCloudT::Ptr object_features (new FeatureCloudT);
	FeatureCloudT::Ptr scene_features (new FeatureCloudT);
	
	// Estimate normals for object and scene
	pcl::NormalEstimation<pcl::PointXYZ, PointNT> ne;
	ne.setInputCloud (fromROSMsg(msg));
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (0.03);
	ne.compute (*scene);
	
	//ne.setInputCloud(obj_cloud);
	//ne.compute(*object);
	object = obj_cloud;
	ROS_INFO_STREAM("Scene size: " << scene->points.size() << " object size: " << object->points.size());
	
	// Downsample
	pcl::console::print_highlight ("Skipping downsampling... Voxel grid leaves only one point in the cloud\n");
	PointCloudT t1, t2;
	pcl::VoxelGrid<PointNT> grid;
	const float leaf = 0.005f;
	grid.setLeafSize (leaf, leaf, leaf);
	grid.setInputCloud (object);
	grid.filter (t1);
	*object = t1;
	grid.setInputCloud (scene);
	grid.filter (t2);
	*scene = t2;

	// Estimate normals for scene
	ROS_INFO_STREAM("Post Downsample: Scene size: " << scene->points.size() << " object size: " << object->points.size());
	pcl::console::print_highlight ("Estimating scene normals...\n");
	pcl::NormalEstimationOMP<PointNT,PointNT> nest;
	nest.setRadiusSearch (0.01);
	nest.setInputCloud (scene);
	nest.compute (*scene);

	// Estimate features
	ROS_INFO_STREAM("Post normals: Scene size: " << scene->points.size() << " object size: " << object->points.size());
	pcl::console::print_highlight ("Estimating features...\n");
	FeatureEstimationT fest;
	fest.setRadiusSearch (0.025);
	fest.setInputCloud (object);
	fest.setInputNormals (object);
	fest.compute (*object_features);
	fest.setInputCloud (scene);
	fest.setInputNormals (scene);
	fest.compute (*scene_features);

	// Perform alignment
	ROS_INFO_STREAM("Perform alignment: Scene size: " << scene->points.size() << " object size: " << object->points.size());
	pcl::console::print_highlight ("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
	align.setInputSource (object);
	align.setSourceFeatures (object_features);
	align.setInputTarget (scene);
	align.setTargetFeatures (scene_features);
	align.setMaximumIterations (50000); // Number of RANSAC iterations
	align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness (5); // Number of nearest features to use
	align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
	//align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
	align.setMaxCorrespondenceDistance (0.01); // Inlier threshold
	align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align.align (*object_aligned);
	}

	// Display object cloud in Rviz
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*object, *cloud);
	sensor_msgs::PointCloud2 obj = *toROSMsg(*cloud);
	obj.header.frame_id = "object";
	obj.header.stamp = ros::Time::now();
	obj_pub->publish(obj);

	if (align.hasConverged ())
	{
		// Print results
		printf ("\n");
		Eigen::Matrix4f transformation = align.getFinalTransformation ();
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
		pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

		// Show alignment
		pcl::visualization::PCLVisualizer visu("Alignment");
		visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
		visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
		visu.spin ();
	}
	else
	{
		pcl::console::print_error ("Alignment failed!\n");
		pcl::visualization::PCLVisualizer visu("Alignment Initials");
		visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
		visu.addPointCloud (object, ColorHandlerT (object, 0.0, 0.0, 255.0), "object");
		visu.spin ();
	}
	
}

pcl::PointCloud<pcl::PointXYZ>::Ptr subsample_mesh(vtkSmartPointer<vtkPolyData>& polydata1)
{
	//make sure that the polygons are triangles!
	vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New ();
#if VTK_MAJOR_VERSION < 6
	triangleFilter->SetInput (polydata1);
#else
	triangleFilter->SetInputData (polydata1);
#endif
	triangleFilter->Update ();
	vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
	triangleMapper->SetInputConnection (triangleFilter->GetOutputPort ());
	triangleMapper->Update();
	polydata1 = triangleMapper->GetInput();
	
	// Perform the sampling
	//pcl::visualization::PCLVisualizer vis;
	//vis.addModelFromPolyData (polydata1, "mesh1", 0);
	//vis.setRepresentationToSurfaceForAllActors ();
	//vis.spin();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
	uniform_sampling (polydata1, NUM_SAMPLE_POINTS, *cloud_1);
	
	// Downsample
	//VoxelGrid<PointXYZ> grid_;
	return cloud_1;
}

void set_object_cb(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO_STREAM("Got an object name" << msg->data << ".\nUSING HARDCODED OBJECT PATH");
	std::string path = std::string("/home/sonny/catkin_ws/src/valid_grasp_generator/models/stl_files/") + msg->data;

	/*pcl::PolygonMesh m;
	pcl::io::loadPolygonFileSTL(path, m);
	vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New ();
	pcl::io::mesh2vtk (m, polydata1);*/
	PointCloudT::Ptr cloud_1 (new PointCloudT);
	pcl::io::loadPCDFile<PointNT> (path.c_str(), *cloud_1);

	obj_cloud = cloud_1;
}

// Align a rigid object to a scene with clutter and occlusions
int main (int argc, char **argv){
	ros::init(argc, argv, "object_estimator");
	ros::NodeHandle nh("~");
	ros::Subscriber scene_sub = nh.subscribe("scene_input", 1, estimate_pose_cb);
	ros::Subscriber obj_sub = nh.subscribe("object_input", 1, set_object_cb);
	ros::Publisher _obj_pub = nh.advertise<sensor_msgs::PointCloud2>("object_output", 1);
	obj_pub = &_obj_pub;

	// Get input object and scene
/*	if (argc != 3)
	{
		pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
		return (1);
	}

	// Load object and scene
	pcl::console::print_highlight ("Loading point clouds...\n");
	if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||
			pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0)
	{
		pcl::console::print_error ("Error loading object/scene file!\n");
		return (1);
	}
*/
	
	ros::spin();

	return (0);
}
