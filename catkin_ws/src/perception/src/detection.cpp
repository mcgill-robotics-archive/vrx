#include <ros/ros.h>
// PCL specifc includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
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
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

std::string lidar_topic;

void downsample(pcl::PointCloud2ConstPtr input,
                pcl::PointCloud2ConstPtr filtered){
  // Use Voxel grid
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud(input);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*filtered);
}

void planar_segmentation(pcl::PointCloud2ConstPtr filtered){
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

  pcl::PointCloud2 cloud_temp;
  // Perform euclidian clustering
  int i=0, nr_points = (int) filtered->points.size();
  while (filtered->points.size() > 0.3 * nr_points) {
    // Segment planar cloud from remaining cloud
    seg.setInputCloud(filtered);
    seg.segment(*inliers, *coeff);
    if (inliers->indices.size() == 0) {
      ROS_INFO("Could not estimate planar model.");
      break;
    }

    // Extract planar inliers
    pcl::ExtractIndices<pcl::PointCloud2> extract;
    extract.setInputCloud(filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(cloud_plane);
    extract.setNegative(true);
    extract.filter(cloud_temp);
    *filtered = cloud_temp;
  }
}

std::vector<pcl::PointIndices> euclidian_cluster_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr pc){
  // Kd tree
  pcl::search::KdTree<pcl::PointIndices>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pc);
  // cluster extraction
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclidianClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pc);
  ec.extract(cluster_indices);
  // Return indices of clusters
  return cluster_indices
}

std::list<pcl::CentroidPoint<pcl::PointXYZ>>
    compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
                     std::vector<pcl::PointIndices> cluster_indices){
  int j = 0;
  std::list<pcl::CentroidPoint<pcl::PointXYZ>> l = {};
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end() ; it++) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      cloud_cluster->points.push_back(cloud.points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // Compute centroid of current cluster
    CentroidPoint<pcl::PointXYZ> centroid;
    // Add points in cluster to centroid point
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator cit = cloud.begin();
         cit != cloud.end() ; cit++) {
      centroid.add(*cit);
    }
    pcl::PointXYZ c1;
    centroid.get(c1);
    // Push to back of list
    l.push_back(c1);
    }
  return l;
 }


bool detection(perception::DetectObjects::Request &req,
               perception::DetectObjects::Response &res) {

  // Obtain point cloud message
  sensor_msgs::PointCloud2ConstPtr pc = ros::topic::waitForMessage(lidar_topic,
                                                      ros::Duration(10));
  sensor_msgs::PointCloud2ConstPtr transformed_pc;

  // Transform point cloud to base link frame
  std::string lidar_frame, base_frame;
  ros::param::get("base_frame", base_frame);
  ros::param::get("lidar_frame", lidar_frame);
  tf::TransformListener listener;
  pcl_ros::transformPointCloud(base_frame, pc, transformed_pc, listener);

  // convert to PCL point cloud
  pcl::PCLPointCloud2 cloud_in = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_in);
  pcl_conversions::toPCL(transformed_pc, cloudPtr);

  // Downsample cloud
  pcl::PointCloud2 cloud_filtered;
  pcl::PointCloud2ConstPtr filteredPtr(cloud_filtered);
  downsample(cloudPtr, filteredPtr);

  // Remove planar surfaces from point cloud using segmentation
  planar_segmentation(filteredPtr);

  // Extract clusters, by first convering PointCloud2 to type XYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCLoud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*filteredPtr,*xyz_cloud);
  std::vector<pcl::PointIndices> cluster_indices = euclidian_cluster_extraction(xyz_cloud);

  // For each cluster, compute centroid, and append position
  std::list<pcl::PointXYZ> centroids = compute_centroid(xyz_cloud, cluster_indices);

  // Construct response from centroids
  for (std::list<pcl::PointXYZ>::const_iterator it = centroids.begin();
       it != centroids.end(); it++) {
    geometry_msgs::Pose p;
    p.position.x = it->x;
    p.position.y = it->y;
    p.position.z = it->z;
    res.object_locations.poses.push_back(p);
  }
  res.header.frame_id = base_frame;
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


