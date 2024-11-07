/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#include "ltm_pointcloud_filter/pointcloud_filter_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace LTM;

PointCloudFilterNode::PointCloudFilterNode(const std::string& node_name)
: Node(node_name)
{
  // Initialize ROS communication
  initializeInputPointcloudSubscriber();
  initializeOutputPointcloudPublisher();

  RCLCPP_INFO(get_logger(), "%s has been initialized", get_name());
}

PointCloudFilterNode::~PointCloudFilterNode()
{
  RCLCPP_WARN(get_logger(), "%s has been terminated", get_name());
}

void PointCloudFilterNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert ROS message to PCL point cloud as input point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud_in);

  // Filter the input point cloud to get the output point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
  filterPointCloud(cloud_in, cloud_out);

  // Publish the output point cloud
  publishFilteredPointCloud(cloud_out, msg->header);
}

void PointCloudFilterNode::publishFilteredPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  const std_msgs::msg::Header& header)
{
  // Convert PCL point cloud to ROS message as output point cloud
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);

  // Set the header of the output point cloud message
  msg.header = header;

  // Publish the output point cloud message
  m_output_pointcloud_pub->publish(msg);
}

void PointCloudFilterNode::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
  // Without overriding this function, the filter is merely a pass-through
  *cloud_out = *cloud_in;
}

void PointCloudFilterNode::initializeInputPointcloudSubscriber()
{
  this->declare_parameter("input_pointcloud_topic_name", "input_pointcloud");
  this->declare_parameter("input_pointcloud_topic_queue_size", 1);

  std::string input_pointcloud_topic = this->get_parameter("input_pointcloud_topic_name").as_string();
  int input_pointcloud_queue_size = this->get_parameter("input_pointcloud_topic_queue_size").as_int();
  RCLCPP_INFO(get_logger(), "Subscribing to %s with queue size %d",
    input_pointcloud_topic.c_str(), input_pointcloud_queue_size);

  m_input_pointcloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(input_pointcloud_topic, 
    input_pointcloud_queue_size, std::bind(&PointCloudFilterNode::pointcloudCallback, this, std::placeholders::_1));
}

void PointCloudFilterNode::initializeOutputPointcloudPublisher()
{
  this->declare_parameter("output_pointcloud_topic_name", "output_pointcloud");
  this->declare_parameter("output_pointcloud_topic_queue_size", 1);

  std::string output_pointcloud_topic = this->get_parameter("output_pointcloud_topic_name").as_string();
  int output_pointcloud_queue_size = this->get_parameter("output_pointcloud_topic_queue_size").as_int();
  RCLCPP_INFO(get_logger(), "Publishing to %s with queue size %d",
    output_pointcloud_topic.c_str(), output_pointcloud_queue_size);

  m_output_pointcloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_pointcloud_topic, output_pointcloud_queue_size);
}
