/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#include "ltm_pointcloud_filter/pointcloud_filter_node.hpp"

#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
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

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Geometry>

using namespace LTM;

PointCloudFilterNode::PointCloudFilterNode()
: Node("ltm_pointcloud_filter_node")
{
  // Determine if the node is running in simulation mode
  declare_parameter("in_simulation", true);
  bool in_simulation = this->get_parameter("in_simulation").as_bool();

  // Initialize filtering objects
  initializeGroundPlaneSegmentation();
  initializeRobotClusterRemoval();
  initializeVoxelGridFilter();

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

// void PointCloudFilterNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
// {
//   // Convert the ROS PointCloud2 message to a PCL PointCloud
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = convertPointCloud2ToPCL(msg);

//   // Downsample the pointcloud using leaf size
//   pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
//   voxel_grid.setInputCloud(cloud);
//   voxel_grid.setLeafSize(m_voxel_grid_leaf_size, m_voxel_grid_leaf_size, m_voxel_grid_leaf_size);
//   voxel_grid.filter(*cloud_downsampled);

//   // Remove the ground plane from the pointcloud
//   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

//   if (!m_ground_plane_segmentation->segmentPlane(cloud_downsampled, inliers, coefficients)) {
//     RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
//     return;
//   }

//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_removed(new pcl::PointCloud<pcl::PointXYZ>);
//   m_ground_plane_segmentation->removePlane(cloud_downsampled, cloud_plane_removed, inliers);

//   // // Create the KdTree object for the search method of the extraction
//   // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//   // tree->setInputCloud(cloud_plane_removed);

//   // // Create the Euclidean Cluster Extraction object
//   // std::vector<pcl::PointIndices> cluster_indices;
//   // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//   // ec.setClusterTolerance(0.1);  // 10cm
//   // ec.setMinClusterSize(100);
//   // ec.setMaxClusterSize(25000);
//   // ec.setSearchMethod(tree);
//   // ec.setInputCloud(cloud_plane_removed);

//   // // Obtain the cluster indices from the input cloud
//   // ec.extract(cluster_indices);

//   // // Create a marker array for the bounding boxes
//   // visualization_msgs::msg::MarkerArray marker_array;

//   // // Iterate over the clusters and publish the bounding boxes
//   // RCLCPP_INFO(this->get_logger(), "Number of clusters: %lu", cluster_indices.size());
//   // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
//   //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
//   //   for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
//   //     cloud_cluster->points.push_back(cloud_plane_removed->points[*pit]);
//   //   }
//   //   cloud_cluster->width = cloud_cluster->points.size();
//   //   cloud_cluster->height = 1;
//   //   cloud_cluster->is_dense = true;

//   //   // Compute the bounding box of the cluster
//   //   pcl::PointXYZ min_pt, max_pt;
//   //   pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

//   //   // Create the bounding box message
//   //   vision_msgs::msg::BoundingBox3D bounding_box;
//   //   bounding_box.center.position.x = (min_pt.x + max_pt.x) / 2;
//   //   bounding_box.center.position.y = (min_pt.y + max_pt.y) / 2;
//   //   bounding_box.center.position.z = (min_pt.z + max_pt.z) / 2;
//   //   bounding_box.size.x = max_pt.x - min_pt.x;
//   //   bounding_box.size.y = max_pt.y - min_pt.y;
//   //   bounding_box.size.z = max_pt.z - min_pt.z;

//   //   // Publish the bounding box message
//   //   m_bounding_box_pub->publish(bounding_box);

//   //   // Create a marker array for the bounding box
//   //   visualization_msgs::msg::Marker marker;
//   //   marker.header.frame_id = msg->header.frame_id;
//   //   marker.header.stamp = msg->header.stamp;
//   //   marker.ns = "bounding_box";
//   //   marker.id = static_cast<int>(it - cluster_indices.begin());
//   //   marker.type = visualization_msgs::msg::Marker::CUBE;
//   //   marker.action = visualization_msgs::msg::Marker::ADD;
//   //   marker.pose.position.x = bounding_box.center.position.x;
//   //   marker.pose.position.y = bounding_box.center.position.y;
//   //   marker.pose.position.z = bounding_box.center.position.z;
//   //   marker.pose.orientation.x = 0.0;
//   //   marker.pose.orientation.y = 0.0;
//   //   marker.pose.orientation.z = 0.0;
//   //   marker.pose.orientation.w = 1.0;
//   //   marker.scale.x = bounding_box.size.x;
//   //   marker.scale.y = bounding_box.size.y;
//   //   marker.scale.z = bounding_box.size.z;
//   //   marker.color.r = 0.0f;
//   //   marker.color.g = 1.0f;
//   //   marker.color.b = 0.0f;
//   //   marker.color.a = 0.5;
//   //   marker.lifetime = rclcpp::Duration(0.5);
//   //   marker_array.markers.push_back(marker);
//   // }

//   // Convert the PCL PointCloud back to a ROS PointCloud2 message
//   sensor_msgs::msg::PointCloud2::SharedPtr filtered_msg = convertPCLToPointCloud2(cloud_plane_removed);

//   // Publish the filtered PointCloud2 message
//   m_filtered_pointcloud_pub->publish(*filtered_msg);

//   // // Publish the bounding box marker array
//   // m_marker_array_pub->publish(marker_array);
// }

void PointCloudFilterNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received pointcloud message with %d points", msg->width * msg->height);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input = convertPointCloud2ToPCL(msg);

  // Determine if the pointcloud is empty
  if (cloud_input->empty()) {
    RCLCPP_DEBUG(this->get_logger(), "Received empty pointcloud message.");
    return;
  }

  // Downsample the pointcloud using leaf size
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  m_voxel_grid_filter->filter(cloud_input, cloud_downsampled);

  // Remove the ground plane from the pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_removed(new pcl::PointCloud<pcl::PointXYZ>);
  removeGroundPlane(cloud_downsampled, cloud_plane_removed);

  // Remove the robot clusters from the pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_robot_removed(new pcl::PointCloud<pcl::PointXYZ>);
  m_robot_cluster_removal->removeRobotCluster(cloud_plane_removed, cloud_robot_removed);

  publishFilteredPointCloud(cloud_robot_removed, msg->header.frame_id, msg->header.stamp);
}

void PointCloudFilterNode::publishFilteredPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  const std::string& frame_id, const rclcpp::Time& stamp)
{
  // Convert the PCL PointCloud back to a ROS PointCloud2 message
  sensor_msgs::msg::PointCloud2::SharedPtr filtered_msg = convertPCLToPointCloud2(cloud);

  // Set the header information for the PointCloud2 message
  filtered_msg->header.frame_id = frame_id;
  filtered_msg->header.stamp = stamp;

  // Publish the filtered PointCloud2 message
  m_filtered_pointcloud_pub->publish(*filtered_msg);
}

void PointCloudFilterNode::removeGroundPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  if (!m_ground_plane_segmentation->segmentPlane(cloud_input, inliers, coefficients)) {
    RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model from input pointcloud.");
    return; // TODO: Return buffer instead?
  }
  m_ground_plane_segmentation->removePlane(cloud_input, cloud_filtered, inliers);
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

void PointCloudFilterNode::initializeGroundPlaneSegmentation()
{
  // Initialize the GroundPlaneSegmentation object
  m_ground_plane_segmentation = std::make_unique<LTM::GroundPlaneSegmentation>();

  // Declare parameters for the GroundPlaneSegmentation object
  declare_parameter("ground_plane_removal.sac_segmentation.distance_threshold", 0.01);
  declare_parameter("ground_plane_removal.sac_segmentation.max_iterations", 1000);
  declare_parameter("ground_plane_removal.sac_segmentation.probability", 0.99);

  // Set the parameters for the SAC segmentation
  double sac_segmentation_distance_threshold = this->get_parameter("ground_plane_removal.sac_segmentation.distance_threshold").as_double();
  int sac_segmentation_max_iterations = this->get_parameter("ground_plane_removal.sac_segmentation.max_iterations").as_int();
  double sac_segmentation_probability = this->get_parameter("ground_plane_removal.sac_segmentation.probability").as_double();

  m_ground_plane_segmentation->configureSACSegmentationParameters(
    sac_segmentation_distance_threshold, sac_segmentation_max_iterations, sac_segmentation_probability);
  RCLCPP_INFO(this->get_logger(), "Ground plane removal, SAC segmentation parameters configured: \n Distance threshold: %f m\n Max iterations: %d\n Probability: %f",
    sac_segmentation_distance_threshold, sac_segmentation_max_iterations, sac_segmentation_probability);
}

void PointCloudFilterNode::initializeRobotClusterRemoval()
{
  // Initialize the RobotClusterRemoval object
  m_robot_cluster_removal = std::make_unique<LTM::RobotClusterRemoval>(this->get_clock());

  // Declare parameters for the RobotClusterRemoval object
  declare_parameter("robot_cluster_removal.robot_description.package_name", "ltm_go2_description");
  declare_parameter("robot_cluster_removal.robot_description.directory_name", "urdf");
  declare_parameter("robot_cluster_removal.robot_description.file_name", "go2_description.urdf");
  declare_parameter("robot_cluster_removal.robot_mesh_resolution", 0.01);

  // Set the robot mesh resolution
  m_robot_cluster_removal->setRobotMeshResolution(
    this->get_parameter("robot_cluster_removal.robot_mesh_resolution").as_double());

  // Set the robot model from the URDF
  std::string urdf_package_name = this->get_parameter("robot_cluster_removal.robot_description.package_name").as_string();
  std::string urdf_directory_name = this->get_parameter("robot_cluster_removal.robot_description.directory_name").as_string();
  std::string urdf_file_name = this->get_parameter("robot_cluster_removal.robot_description.file_name").as_string();
  std::string urdf_filepath = ament_index_cpp::get_package_share_directory(
    this->get_parameter("robot_cluster_removal.robot_description.package_name").as_string()) + "/" + 
    this->get_parameter("robot_cluster_removal.robot_description.directory_name").as_string() + "/" + 
    this->get_parameter("robot_cluster_removal.robot_description.file_name").as_string();

  // Set the robot model from the URDF file path, exit if failed
  if (!m_robot_cluster_removal->setRobotModel(urdf_filepath)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set the robot model from URDF file: %s", urdf_filepath.c_str());
    exit(EXIT_FAILURE);
  } else {
    RCLCPP_INFO(this->get_logger(), "Robot model set from URDF file: %s", urdf_filepath.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Robot cluster removal configured.");
}

void PointCloudFilterNode::initializeVoxelGridFilter()
{
  // Initialize the VoxelGridFilter object
  m_voxel_grid_filter = std::make_unique<LTM::VoxelGridFilter>();

  // Declare parameters for the VoxelGridFilter object
  declare_parameter("voxel_grid_filter.leaf_size.x", 0.1);
  declare_parameter("voxel_grid_filter.leaf_size.y", 0.1);
  declare_parameter("voxel_grid_filter.leaf_size.z", 0.1);

  // Set the leaf size for the VoxelGridFilter object
  double leaf_size_x = this->get_parameter("voxel_grid_filter.leaf_size.x").as_double();
  double leaf_size_y = this->get_parameter("voxel_grid_filter.leaf_size.y").as_double();
  double leaf_size_z = this->get_parameter("voxel_grid_filter.leaf_size.z").as_double();
  m_voxel_grid_filter->setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

  RCLCPP_INFO(this->get_logger(), "Voxel grid filter leaf size configured: \n x: %f m\n y: %f m\n z: %f m",
    leaf_size_x, leaf_size_y, leaf_size_z);
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
    raw_pointcloud_topic, rclcpp::SensorDataQoS(), 
    std::bind(&PointCloudFilterNode::pointcloudCallback, this, std::placeholders::_1));
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
