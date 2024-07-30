/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#include "ltm_pointcloud_filter/pointcloud_filter_node.hpp"

#include <vector>

#include <visualization_msgs/msg/marker.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
	
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace LTMPointcloudFilterNode;

PointCloudFilterNode::PointCloudFilterNode()
: Node("ltm_pointcloud_filter_node")
{
  // Determine if the node is running in simulation mode
  declare_parameter("in_simulation", true);
  bool in_simulation = this->get_parameter("in_simulation").as_bool();

  // Configure PCL parameters
  configurePCLParameters();

  // Configure ROS subscribers and publishers
  configureRosSubscribers(in_simulation);
  configureRosPublishers(in_simulation);

  RCLCPP_INFO(this->get_logger(), "LTM Pointcloud Filter Node has been initialized.");
}

PointCloudFilterNode::~PointCloudFilterNode()
{
  this->m_raw_pointcloud_sub.reset();
  this->m_filtered_pointcloud_pub.reset();
  RCLCPP_WARN(this->get_logger(), "LTM Pointcloud Filter Node has been destroyed.");
}

void PointCloudFilterNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert the ROS PointCloud2 message to a PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = convertPointCloud2ToPCL(msg);

  // Downsample the pointcloud using leaf size
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(m_voxel_grid_leaf_size, m_voxel_grid_leaf_size, m_voxel_grid_leaf_size);
  voxel_grid.filter(*cloud_downsampled);

  // Create the filtering object
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> sac_segmentation;
  sac_segmentation.setOptimizeCoefficients(true);

  // Set the segmentation parameters for the plane model
  sac_segmentation.setModelType(pcl::SACMODEL_PLANE);
  sac_segmentation.setMethodType(pcl::SAC_RANSAC);
  sac_segmentation.setDistanceThreshold(m_sac_segmentation_distance_threshold);

  sac_segmentation.setInputCloud(cloud_downsampled);
  sac_segmentation.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
    return;
  }

  // Create the filtering object for the inliers
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_downsampled);

  // Extract the inliers of the plane model from the input cloud
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_downsampled);

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_downsampled);

  // Create the Euclidean Cluster Extraction object
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.1);  // 10cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_downsampled);

  // Extract the clusters from the cloud
  ec.extract(cluster_indices);

  // Create a marker array for the bounding boxes
  visualization_msgs::msg::MarkerArray marker_array;

  // Iterate over the clusters and publish the bounding boxes
  RCLCPP_INFO(this->get_logger(), "Number of clusters: %lu", cluster_indices.size());
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      cloud_cluster->points.push_back(cloud_downsampled->points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // Compute the bounding box of the cluster
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

    // Create the bounding box message
    vision_msgs::msg::BoundingBox3D bounding_box;
    bounding_box.center.position.x = (min_pt.x + max_pt.x) / 2;
    bounding_box.center.position.y = (min_pt.y + max_pt.y) / 2;
    bounding_box.center.position.z = (min_pt.z + max_pt.z) / 2;
    bounding_box.size.x = max_pt.x - min_pt.x;
    bounding_box.size.y = max_pt.y - min_pt.y;
    bounding_box.size.z = max_pt.z - min_pt.z;

    // Publish the bounding box message
    m_bounding_box_pub->publish(bounding_box);

    // Create a marker array for the bounding box
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = msg->header.frame_id;
    marker.header.stamp = msg->header.stamp;
    marker.ns = "bounding_box";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = bounding_box.center.position.x;
    marker.pose.position.y = bounding_box.center.position.y;
    marker.pose.position.z = bounding_box.center.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = bounding_box.size.x;
    marker.scale.y = bounding_box.size.y;
    marker.scale.z = bounding_box.size.z;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;
    marker.lifetime = rclcpp::Duration(0.5);
    marker_array.markers.push_back(marker);
  }

  // Convert the PCL PointCloud back to a ROS PointCloud2 message
  sensor_msgs::msg::PointCloud2::SharedPtr filtered_msg = convertPCLToPointCloud2(cloud_downsampled);

  // Publish the filtered PointCloud2 message
  m_filtered_pointcloud_pub->publish(*filtered_msg);

  // Publish the bounding box marker array
  m_marker_array_pub->publish(marker_array);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudFilterNode::convertPointCloud2ToPCL(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  return cloud;
}

sensor_msgs::msg::PointCloud2::SharedPtr PointCloudFilterNode::convertPCLToPointCloud2(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const
{
  sensor_msgs::msg::PointCloud2::SharedPtr msg(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*cloud, *msg);
  return msg;
}

void PointCloudFilterNode::configurePCLParameters()
{
  // Declare the parameters for the PCL segmentation
  declare_parameter("sac_segmentation.distance_threshold", 0.01);
  declare_parameter("sac_segmentation.max_iterations", 1000);
  declare_parameter("sac_segmentation.probability", 0.99);
  declare_parameter("voxel_grid.leaf_size", 0.01);

  // Get the parameters for the PCL segmentation
  m_sac_segmentation_distance_threshold = this->get_parameter("sac_segmentation.distance_threshold").as_double();
  m_sac_segmentation_max_iterations = this->get_parameter("sac_segmentation.max_iterations").as_int();
  m_sac_segmentation_probability = this->get_parameter("sac_segmentation.probability").as_double();
  m_voxel_grid_leaf_size = this->get_parameter("voxel_grid.leaf_size").as_double();
}

void PointCloudFilterNode::configureRosSubscribers(bool in_simulation)
{
  // Get the pointcloud topic parameter from the parameter server
  std::string robot_type = in_simulation ? "sim" : "real";
  std::string raw_pointcloud_topic_param = "topics." + robot_type + ".raw_pointcloud_topic";
  RCLCPP_INFO(this->get_logger(), "Raw pointcloud topic parameter: %s", raw_pointcloud_topic_param.c_str());

  declare_parameter(raw_pointcloud_topic_param, "point_cloud");
  std::string raw_pointcloud_topic = this->get_parameter(raw_pointcloud_topic_param).as_string();
  RCLCPP_INFO(this->get_logger(), "Subscribe to raw pointcloud topic: %s", raw_pointcloud_topic.c_str());

  // Create the raw pointcloud subscriber
  m_raw_pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    raw_pointcloud_topic, 10, std::bind(&PointCloudFilterNode::pointcloudCallback, this, std::placeholders::_1));
}

void PointCloudFilterNode::configureRosPublishers(bool in_simulation)
{
  // Get the filtered pointcloud topic parameter from the parameter server
  std::string robot_type = in_simulation ? "sim" : "real";
  std::string filtered_pointcloud_topic_param = "topics." + robot_type + ".filtered_pointcloud_topic";
  RCLCPP_INFO(this->get_logger(), "Filtered pointcloud topic parameter: %s", filtered_pointcloud_topic_param.c_str());

  declare_parameter(filtered_pointcloud_topic_param, "filtered_point_cloud");
  std::string filtered_pointcloud_topic = this->get_parameter(filtered_pointcloud_topic_param).as_string();
  RCLCPP_INFO(this->get_logger(), "Publish filtered pointcloud topic: %s", filtered_pointcloud_topic.c_str());

  // Create the filtered pointcloud publisher
  m_filtered_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    filtered_pointcloud_topic, 1);

  // Create the bounding box publisher
  m_bounding_box_pub = this->create_publisher<vision_msgs::msg::BoundingBox3D>(
    "bounding_box", 1);
  m_marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "marker_array", 1);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
