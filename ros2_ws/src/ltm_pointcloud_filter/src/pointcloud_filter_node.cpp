/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#include "ltm_pointcloud_filter/pointcloud_filter_node.hpp"

#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using namespace LTMPointcloudFilterNode;

PointCloudFilterNode::PointCloudFilterNode()
: Node("ltm_pointcloud_filter_node")
{
  // Determine if the node is running in simulation mode
  declare_parameter("in_simulation", true);
  bool in_simulation = this->get_parameter("in_simulation").as_bool();

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
  RCLCPP_INFO(this->get_logger(), "Received a new PointCloud2 message.");

  // Convert the ROS PointCloud2 message to a PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = convertPointCloud2ToPCL(msg);

  // Create the filtering object
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> sac_segmentation;
  sac_segmentation.setOptimizeCoefficients(true);

  // Set the segmentation parameters
  sac_segmentation.setModelType(pcl::SACMODEL_PLANE);
  sac_segmentation.setMethodType(pcl::SAC_RANSAC);
  sac_segmentation.setDistanceThreshold(m_sac_segmentation_distance_threshold);

  sac_segmentation.setInputCloud(cloud);
  sac_segmentation.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
    return;
  }

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);

  // Extract the inliers
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud);

  // Convert the PCL PointCloud back to a ROS PointCloud2 message
  sensor_msgs::msg::PointCloud2::SharedPtr filtered_msg = convertPCLToPointCloud2(cloud);

  // Publish the filtered PointCloud2 message
  m_filtered_pointcloud_pub->publish(*filtered_msg);
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

void PointCloudFilterNode::configureRosSubscribers(bool in_simulation)
{
  // Get the pointcloud topic parameter from the parameter server
  std::string robot_type = in_simulation ? "sim" : "real";
  std::string raw_pointcloud_topic_param = "topics." + robot_type + ".raw_pointcloud_topic";
  RCLCPP_INFO(this->get_logger(), "Raw pointcloud topic parameter: %s", raw_pointcloud_topic_param.c_str());

  declare_parameter("raw_pointcloud_topic_param", "point_cloud");
  std::string raw_pointcloud_topic = this->get_parameter(topics.sim.raw_pointcloud_topic).as_string();
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
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
