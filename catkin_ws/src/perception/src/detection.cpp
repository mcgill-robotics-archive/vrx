#include <ros/ros.h>
// PCL specifc includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// Include object detection service
#include "perception/DetectObjects.h"

std::string lidar_topic;

bool detection(perception::DetectObjects::Request &rew,
               perception::DetectObjects::Response &res) {

  // Obtain point cloud message
  sensor_msgs::PointCloud2ConstPtr pc = ros::topic::waitForMessage(lidar_topic,
                                                      ros::Duration(10));

  // convert to PCL point cloud
  pcl::PCLPointCloud2* cloud_in = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_in);
  pcl_conversions::toPCL(pc, *cloud_in);

  // Downsample cloud
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
  voxel_grid.setInputCloud(cloudPtr);
  voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
  pcl::PointCloud2 cloud_filtered;
  voxel_grid.filter(cloud_filtered);

  // Perform euclidian clustering
  // Create planar segmentation object to segment water from cluster
  pcl::SACSegmentation<pcl::PointCloud2> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
  pcl::PointCloud2* cloud_plane = new pcl::PCLPointCloud2;
  pcl::PCDWriter write;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);

  // TODO : Segment water surface from cloud






  return true;



}


int main(int argc, char **argv) {
  ros::init(argc, argv, "object_detection_server");
  ros::NodeHandle nh("~");

  nh.getParam("lidar_topic", lidar_topic);

  ros::ServiceServer service = nh.advertiseService("detect_objects", detection);
  ROS_INFO("Ready to detect objects in lidar view");

  ros::spin();

  return 0;
}


