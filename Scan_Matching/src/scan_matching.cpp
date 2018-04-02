#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <Eigen/Dense>


// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointXYZ,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "hw4_node");
	ros::NodeHandle nh;

	// Point Clouds
	const PointCloudNT::Ptr scene_normals (new PointCloudNT);
	const PointCloudNT::Ptr map_normals (new PointCloudNT);
	const PointCloudXYZ::Ptr map (new PointCloudXYZ);
	const PointCloudXYZ::Ptr scene (new PointCloudXYZ);
	const PointCloudXYZ::Ptr initial_aligned (new PointCloudXYZ);
	const PointCloudXYZ::Ptr final_registered (new PointCloudXYZ);
	const FeatureCloudT::Ptr map_features (new FeatureCloudT);
	const FeatureCloudT::Ptr scene_features (new FeatureCloudT);

	// ROS 
	ros::Publisher map_pub;
	ros::Publisher scene_pub;
	tf::Transform transform;
	tf::TransformBroadcaster br;
	tf::Vector3 trans;
	tf::Matrix3x3 rotation;
	tf::Quaternion quaterion;

	// Create ROS publisher for the output point cloud
	map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 10, true);
	scene_pub = nh.advertise<sensor_msgs::PointCloud2>("/scene", 10, true);


	// Load .pcd files
	pcl::io::loadPCDFile ("0310126_hw4/map.pcd", *map);
	pcl::io::loadPCDFile ("0310126_hw4/scene.pcd", *scene);


	// Estimate normals
	pcl::NormalEstimationOMP<PointXYZ, PointNT> ne;
	pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ> ());
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (0.05);
	ne.setInputCloud (scene);
	ne.compute (*scene_normals);
	ne.setInputCloud (map);
	ne.compute (*map_normals);


	// Estimate features
	FeatureEstimationT fest;
	fest.setRadiusSearch (0.05);
	fest.setInputCloud (map);
	fest.setInputNormals (map_normals);
	fest.compute (*map_features);
	fest.setInputCloud (scene);
	fest.setInputNormals (scene_normals);
	fest.compute (*scene_features);


	// Initial alignment
	pcl::SampleConsensusInitialAlignment<PointXYZ, PointXYZ, FeatureT> sac;
	sac.setMinSampleDistance (0.05f);
	sac.setMaxCorrespondenceDistance (0.2);
	sac.setMaximumIterations (10);
	sac.setInputSource (scene);
	sac.setSourceFeatures (scene_features);
	sac.setInputTarget (map);
	sac.setTargetFeatures (map_features);
	sac.align(*initial_aligned);


	// Iterative closest points
	pcl::IterativeClosestPoint<PointXYZ, PointXYZ> icp;
	icp.setInputSource(initial_aligned); // Set input source
	icp.setInputTarget(map); // Set input target
	icp.setMaxCorrespondenceDistance (0.05); // Set the max correspondence distance
	icp.setMaximumIterations (2000); // Set the maximum number of iterations (criterion 1)
	icp.setTransformationEpsilon(1e-8); // Set the transformation epsilon (criterion 2)
	icp.setEuclideanFitnessEpsilon(1); // Set the euclidean distance difference epsilon (criterion 3)
	icp.align(*final_registered); // Perform the alignment


	// Obtain the transformation that aligned scene_cloud to scene_cloud_registered
	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	std::cout << "Transformation matrix : " << "\n" << std::endl << transformation << "\n";


	// Publish tf
	trans.setValue(static_cast<double>(transformation(0,3)), static_cast<double>(transformation(1,3)), static_cast<double>(transformation(2,3)));
	rotation.setValue(static_cast<double>(transformation(0,0)), static_cast<double>(transformation(0,1)), static_cast<double>(transformation(0,2)), 
						static_cast<double>(transformation(1,0)), static_cast<double>(transformation(1,1)), static_cast<double>(transformation(1,2)), 
						static_cast<double>(transformation(2,0)), static_cast<double>(transformation(2,1)), static_cast<double>(transformation(2,2)));
	rotation.getRotation(quaterion);
	transform.setOrigin(trans);
	transform.setRotation(quaterion);
	br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "map", "scene"));


	// Convert pcd data type to ROS message type
	sensor_msgs::PointCloud2 map_cloud;
	sensor_msgs::PointCloud2 scene_cloud;
	pcl::toROSMsg(*map, map_cloud);
	pcl::toROSMsg(*scene, scene_cloud);


	// Set PointCloud2 parameters
	map_cloud.header.frame_id = "map";
	scene_cloud.header.frame_id = "scene";


	// Publish the point cloud message and tf
	map_pub.publish(map_cloud);
	scene_pub.publish(scene_cloud);


	// Spin
	ros::spin();

	return 0;
}