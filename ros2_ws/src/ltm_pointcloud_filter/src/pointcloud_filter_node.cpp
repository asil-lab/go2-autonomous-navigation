/*
 * Project Lava Tube Mapping
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "ltm_pointcloud_filter/pointcloud_filter_node.hpp"

#include <string>
#include <vector>

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
  // Convert the ROS PointCloud2 message to a PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = convertPointCloud2ToPCL(msg);

  // Perform the pointcloud filtering operations
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
  std::string pointcloud_topic_param = "topics." + robot_type + ".pointcloud_topic";

  declare_parameter(pointcloud_topic_param, "point_cloud");
  std::string pointcloud_topic = this->get_parameter(pointcloud_topic_param).as_string();

  // Create the raw pointcloud subscriber
  m_raw_pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic, 10,
    std::bind(&PointCloudFilterNode::pointcloudCallback, this, std::placeholders::_1));
}

void PointCloudFilterNode::configureRosPublishers(bool in_simulation)
{
  // Get the filtered pointcloud topic parameter from the parameter server
  std::string robot_type = in_simulation ? "sim" : "real";
  std::string filtered_pointcloud_topic_param = "topics." + robot_type + ".filtered_pointcloud_topic";

  declare_parameter(filtered_pointcloud_topic_param, "filtered_point_cloud");
  std::string filtered_pointcloud_topic = this->get_parameter(filtered_pointcloud_topic_param).as_string();

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
