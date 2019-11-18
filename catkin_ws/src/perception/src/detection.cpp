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
#include <pcl/common/centroid.h>
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
  pcl::PointCloud2 cloud_temp;
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

  int i=0, nr_points = (int) cloud_filtered->points.size();
  while (cloud_filtered->points.size() > 0.3 * nr_points) {
    // Segment planar cloud from remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coeff);
    if (inliers->indices.size() == 0) {
      ROS_INFO("Could not estimate planar model.");
      break;
    }

    // Extract planar inliers
    pcl::ExtractIndices<pcl::PointCloud2> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(cloud_plane);
    extract.setNegative(true);
    extract.filter(cloud_temp);
    cloud_filtered = cloud_temp;
  }

  // Create KDTree object for cluster extraction
  pcl::search::KdTree<pcl::PointIndices>::Ptr tree (new pcl::search::KdTree<pcl::PointCloud2>);
  tree->setInputCloud(cloud_filtered);
  std::vector<pcl::PointCloud2> cluster_indices;
  pcl::EuclidianClusterExtraction<pcl::PointCloud2> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end() ; it++) {
    pcl::PointCloud2::Ptr cloud_cluster (new pcl::PointCloud2);

    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      cloud_cluster->points.push_back(cloud_filtered.points[*pit];
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // Compute centroid of current cluster
    CentroidPoint<pcl::PointCloud2> centroid;
    // Convert Point Cloud into a XYZ cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud_cluster, *centroid_cloud);
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator cit = centroid_cloud.begin();
         cit != centroid_cloud.end() ; cit++) {
      centroid.add(*cit);
    }
    pcl::PointXYZ c1;
    centroid.get(c1);
    //TODO TF Transformation
  }
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


