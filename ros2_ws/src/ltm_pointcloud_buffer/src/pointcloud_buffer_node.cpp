/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 01-08-2024.
 */

#include "ltm_pointcloud_buffer/pointcloud_buffer_node.hpp"

#include <chrono>

using namespace LTM;

PointCloudBufferNode::PointCloudBufferNode()
: Node("pointcloud_buffer_node")
{
  // Initialize the pointcloud buffer
  initializePointcloudBuffer();

  // Create a timer to call the callback function at a specified rate
  auto timer_callback = std::bind(&PointCloudBufferNode::timerCallback, this);
  m_timer = this->create_wall_timer(std::chrono::milliseconds(5000), timer_callback); // TODO: Parameterize this

  // Create a subscription to the pointcloud topic
  m_pointcloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "point_cloud/raw", 10, std::bind(&PointCloudBufferNode::pointcloudCallback, this, std::placeholders::_1));

  // Create a publisher to the pointcloud topic
  m_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "point_cloud/buffer", 10);

  // Create a buffer and listener for the tf2 transforms
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  RCLCPP_INFO(this->get_logger(), "Pointcloud Buffer Node has been initialized");
}

PointCloudBufferNode::~PointCloudBufferNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down PointCloudBufferNode");
}

void PointCloudBufferNode::timerCallback()
{
  publishPointcloudBuffer();
  clearPointcloudBuffer();
}

void PointCloudBufferNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received pointcloud message with %d points", msg->width * msg->height);
  updateInputPointcloudMsg(msg);
  bufferPointCloud(convertPointCloud2ToPCL(msg));
}

void PointCloudBufferNode::initializePointcloudBuffer()
{
  m_pointcloud_buffer = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  m_pointcloud_buffer->header.frame_id = "world";

  m_recent_input_pointcloud_msg = sensor_msgs::msg::PointCloud2::SharedPtr(new sensor_msgs::msg::PointCloud2);
}

void PointCloudBufferNode::updateInputPointcloudMsg(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  m_recent_input_pointcloud_msg = msg;
}

void PointCloudBufferNode::bufferPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  *m_pointcloud_buffer += *(transformPointCloud(cloud, "world"));
}

void PointCloudBufferNode::clearPointcloudBuffer()
{
  m_pointcloud_buffer->clear();
}

void PointCloudBufferNode::publishPointcloudBuffer()
{
  sensor_msgs::msg::PointCloud2::SharedPtr msg = convertPCLToPointCloud2(m_pointcloud_buffer);
  msg->header.stamp = this->now();
  m_pointcloud_publisher->publish(*msg);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBufferNode::transformPointCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string target_frame) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // while (!m_tf_buffer->canTransform(m_recent_input_pointcloud_msg->header.frame_id, 
  //   target_frame, m_recent_input_pointcloud_msg->header.stamp))
  // {
  //   RCLCPP_DEBUG(this->get_logger(), "Waiting for transform from %s to %s", 
  //     m_recent_input_pointcloud_msg->header.frame_id.c_str(), target_frame.c_str());
  // }
  pcl_ros::transformPointCloud(target_frame, *cloud, *transformed_cloud, *m_tf_buffer);
  return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBufferNode::convertPointCloud2ToPCL(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  return cloud;
}

sensor_msgs::msg::PointCloud2::SharedPtr PointCloudBufferNode::convertPCLToPointCloud2(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const
{
  sensor_msgs::msg::PointCloud2::SharedPtr msg(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*cloud, *msg);
  return msg;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudBufferNode>());
  rclcpp::shutdown();
  return 0;
}