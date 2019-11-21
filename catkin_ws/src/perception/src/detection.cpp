#include <ros/ros.h>
// PCL specifc includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
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
ros::Publisher debug;

void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered){
  // Use Voxel grid
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(input);
  float vg_size;
  ros::param::get("~vg_size", vg_size);
  vg.setLeafSize(vg_size,vg_size, vg_size);
  vg.filter(*filtered);
}

void planar_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr filtered){
  // Create planar segmentation object to segment water from cluster
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PCDWriter write;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);

  pcl::PointCloud<pcl::PointXYZ> cloud_temp;
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
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    extract.setNegative(true);
    extract.filter(cloud_temp);
    *filtered = cloud_temp;
  }
}

std::vector<pcl::PointIndices> euclidian_cluster_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr pc){
  // Kd tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pc);
  // cluster extraction
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  int cluster_tolerance, min_cluster_size;
  ros::param::get("~cluster_tolerance", cluster_tolerance);
  ros::param::get("~min_cluster_size", min_cluster_size);

  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pc);
  ec.extract(cluster_indices);
  // Return indices of clusters
  return cluster_indices;
}

std::list<pcl::PointXYZ>
    compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     std::vector<pcl::PointIndices> cluster_indices){

  std::list<pcl::PointXYZ> l = {};
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
      it != cluster_indices.end() ; it++) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      cloud_cluster->points.push_back(cloud->points[*pit]);
    }

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // Compute centroid of current cluster
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    // Add points in cluster to centroid point
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator cit = cloud_cluster->begin();
         cit != cloud_cluster->end() ; cit++) {
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
  sensor_msgs::PointCloud2ConstPtr pc = ros::topic::waitForMessage<sensor_msgs::PointCloud2>
                                                    (lidar_topic, ros::Duration(10));
  // Obtain lidar link frame
  std::string lidar_frame;
  ros::param::get("~lidar_frame", lidar_frame);

  // convert to PCL::PointCloud<pcl::PointXYZ>
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pc, pcl_pc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::fromPCLPointCloud2(pcl_pc, *cloud_xyz);

  // Downsample cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
  downsample(cloud_xyz, cloud_filtered);
  *cloud_xyz = *cloud_filtered;

  // Extract clusters, by first convering PointCloud2 to type XYZ
  std::vector<pcl::PointIndices> cluster_indices = euclidian_cluster_extraction(cloud_xyz);

  // For each cluster, compute centroid, and append position
  std::list<pcl::PointXYZ> centroids = compute_centroid(cloud_xyz, cluster_indices);

  // Construct response from centroids
  for (std::list<pcl::PointXYZ>::const_iterator it = centroids.begin();
       it != centroids.end(); it++) {
    geometry_msgs::Pose p;
    p.position.x = it->x;
    p.position.y = it->y;
    p.position.z = it->z;
    res.object_locations.poses.push_back(p);
  }
  res.object_locations.header.frame_id = lidar_frame;
  return true;
}

void debugCallback(sensor_msgs::PointCloud2ConstPtr input_pc) {

  sensor_msgs::PointCloud2 transformed_pc;
  std::string base_frame, lidar_frame;
  ros::param::get("~base_frame", base_frame);
  ros::param::get("~lidar_frame", lidar_frame);
  //tf::TransformListener listener;
  //pcl_ros::transformPointCloud(base_frame, *input_pc, transformed_pc, listener);

  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*input_pc, pcl_pc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::fromPCLPointCloud2(pcl_pc, *cloud_xyz);

  // Downsample cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
  downsample(cloud_xyz, cloud_filtered);
  *cloud_xyz = *cloud_filtered;

  //sensor_msgs::PointCloud2 debug_msg;
  //pcl::toROSMsg(*cloud_xyz, debug_msg);
  //debug.publish(debug_msg);

  std::vector<pcl::PointIndices> cluster_indices = euclidian_cluster_extraction(cloud_xyz);
  std::list<pcl::PointXYZ> centroids = compute_centroid(cloud_xyz, cluster_indices);

  geometry_msgs::PoseArray msg;
  for (std::list<pcl::PointXYZ>::const_iterator it = centroids.begin();
  it != centroids.end(); it++) {
    // if (it->x < 0.0) continue;
    geometry_msgs::Pose p;
    p.position.x = it->x;
    p.position.y = it->y;
    p.position.z = it->z;
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = 1;
    p.orientation.w = 0;
    msg.poses.push_back(p);
}
    msg.header.frame_id = lidar_frame;
    debug.publish(msg);

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "object_detection_server");
  ros::NodeHandle nh("~");

  ros::param::get("~lidar_topic", lidar_topic);

  ros::ServiceServer service = nh.advertiseService("detect_objects", detection);
  debug = nh.advertise<geometry_msgs::PoseArray>("lidar_debug", 10);
  ros::Subscriber sub = nh.subscribe(lidar_topic, 10, debugCallback);

  ROS_INFO("Ready to detect objects in lidar view");

  ros::spin();

  return 0;
}


