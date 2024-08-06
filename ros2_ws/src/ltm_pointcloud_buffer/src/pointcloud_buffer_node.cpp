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
  initializePointcloudBuffer();
  initializeROSTopics();
  initializeTFListener();
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

void PointCloudBufferNode::updateInputPointcloudMsg(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  m_recent_input_pointcloud_msg = msg;
}

void PointCloudBufferNode::bufferPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  *m_pointcloud_buffer += *(transformPointCloud(cloud));
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
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ros::transformPointCloud(m_target_frame, rclcpp::Time(0), *cloud, m_source_frame, *transformed_cloud, *m_tf_buffer);
  return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBufferNode::convertPointCloud2ToPCL(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  return cloud;
}

void PointCloudBufferNode::initializeROSTopics()
{
  // Extract the parameters for topics
  declare_parameter("topics.raw_pointcloud_topic", "/go2_gazebo/point_cloud/raw");
  declare_parameter("topics.buffered_pointcloud_topic", "go2_gazebo/point_cloud/buffer");

  std::string raw_pointcloud_topic = 
    this->get_parameter("topics.raw_pointcloud_topic").as_string();
  std::string buffered_pointcloud_topic = 
    this->get_parameter("topics.buffered_pointcloud_topic").as_string();
  RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", raw_pointcloud_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing to topic: %s", buffered_pointcloud_topic.c_str());

  // Create a subscription to the raw pointcloud topic
  m_pointcloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    raw_pointcloud_topic, 10, 
    std::bind(&PointCloudBufferNode::pointcloudCallback, this, std::placeholders::_1));

  // Create a publisher to the buffered pointcloud topic
  m_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    buffered_pointcloud_topic, 10);
}

void PointCloudBufferNode::initializeTFListener()
{
  // Create a buffer and listener for the tf2 transforms
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}

void PointCloudBufferNode::initializePointcloudBuffer()
{
  // Extract the parameters
  declare_parameter("buffer_time", 0.5);
  declare_parameter("source_frame", "radar");
  declare_parameter("target_frame", "world");

  double buffer_time = this->get_parameter("buffer_time").as_double();
  m_source_frame = this->get_parameter("source_frame").as_string();
  m_target_frame = this->get_parameter("target_frame").as_string();

  RCLCPP_INFO(this->get_logger(), "Buffer time: %f s", buffer_time);
  RCLCPP_INFO(this->get_logger(), "Source frame: %s, target frame: %s",
    m_source_frame.c_str(), m_target_frame.c_str());

  // Initialize an empty pointcloud buffer
  m_pointcloud_buffer = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  m_pointcloud_buffer->header.frame_id = m_target_frame;

  // Initialize an empty Pointcloud2 msg
  m_recent_input_pointcloud_msg = 
    sensor_msgs::msg::PointCloud2::SharedPtr(new sensor_msgs::msg::PointCloud2);

  // Initialize a buffer timer
  unsigned int buffer_time_int = static_cast<unsigned int>(buffer_time * 1000);
  m_timer = this->create_wall_timer(std::chrono::milliseconds(buffer_time_int), 
    std::bind(&PointCloudBufferNode::timerCallback, this));
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