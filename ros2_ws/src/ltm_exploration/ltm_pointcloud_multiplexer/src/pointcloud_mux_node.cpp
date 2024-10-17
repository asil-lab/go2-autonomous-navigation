/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 17-10-2024.
 */

#include "ltm_pointcloud_multiplexer/pointcloud_mux_node.hpp"

using namespace LTM;

PointCloudMuxNode::PointCloudMuxNode()
: Node("pointcloud_mux_node")
{
  initializeLidarPointCloudSubscriber();
  initializeCameraPointCloudSubscriber();
  initializeMuxedPointCloudPublisher();
  initializeMuxTimer();
  RCLCPP_INFO(this->get_logger(), "Pointcloud Mux Node has been initialized.");
}

PointCloudMuxNode::~PointCloudMuxNode()
{
  RCLCPP_WARN(this->get_logger(), "Pointcloud Mux Node has been terminated.");
}

void PointCloudMuxNode::lidarPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  m_lidar_pointcloud_msg = msg;
}

void PointCloudMuxNode::cameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  m_camera_pointcloud_msg = msg;
}

void PointCloudMuxNode::publishMuxedPointCloud() const
{
  // Convert PointCloud2 messages to PCL PointClouds and merge them
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud = convertPointCloud2ToPCL(m_lidar_pointcloud_msg);
  pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud = convertPointCloud2ToPCL(m_camera_pointcloud_msg);
  pcl::PointCloud<pcl::PointXYZ>::Ptr muxed_cloud = mergePointClouds(lidar_cloud, camera_cloud);

  // Convert merged PCL PointCloud to PointCloud2 message and publish
  sensor_msgs::msg::PointCloud2::SharedPtr muxed_pointcloud_msg = convertPCLToPointCloud2(muxed_cloud);
  m_muxed_pointcloud_pub->publish(*muxed_pointcloud_msg);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudMuxNode::mergePointClouds(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud) const
{
  // Assuming that the two pointclouds are in the same frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr muxed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Merge the two pointclouds
  *muxed_cloud = *lidar_cloud + *camera_cloud;

  return muxed_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudMuxNode::convertPointCloud2ToPCL(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  return cloud;
}

sensor_msgs::msg::PointCloud2::SharedPtr PointCloudMuxNode::convertPCLToPointCloud2(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const
{
  sensor_msgs::msg::PointCloud2::SharedPtr msg(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*cloud, *msg);
  return msg;
}

void PointCloudMuxNode::initializeLidarPointCloudSubscriber()
{
  this->declare_parameter("input_lidar_pointcloud_topic_name", "/lidar/pointcloud");
  this->declare_parameter("input_lidar_pointcloud_queue_size", 10);

  std::string input_lidar_pointcloud_topic_name = this->get_parameter("input_lidar_pointcloud_topic_name").as_string();
  int input_lidar_pointcloud_queue_size = this->get_parameter("input_lidar_pointcloud_queue_size").as_int();

  m_lidar_pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_lidar_pointcloud_topic_name, input_lidar_pointcloud_queue_size,
    std::bind(&PointCloudMuxNode::lidarPointCloudCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Lidar Pointcloud Subscriber is listening to topic: %s with queue size %d", 
    input_lidar_pointcloud_topic_name.c_str(), input_lidar_pointcloud_queue_size);

  // Initialize empty pointcloud message
  m_lidar_pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
}

void PointCloudMuxNode::initializeCameraPointCloudSubscriber()
{
  this->declare_parameter("input_camera_pointcloud_topic_name", "/camera/pointcloud");
  this->declare_parameter("input_camera_pointcloud_queue_size", 10);

  std::string input_camera_pointcloud_topic_name = this->get_parameter("input_camera_pointcloud_topic_name").as_string();
  int input_camera_pointcloud_queue_size = this->get_parameter("input_camera_pointcloud_queue_size").as_int();

  m_camera_pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_camera_pointcloud_topic_name, input_camera_pointcloud_queue_size,
    std::bind(&PointCloudMuxNode::cameraPointCloudCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Camera Pointcloud Subscriber is listening to topic: %s with queue size %d", 
    input_camera_pointcloud_topic_name.c_str(), input_camera_pointcloud_queue_size);

  // Initialize empty pointcloud message
  m_camera_pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
}

void PointCloudMuxNode::initializeMuxedPointCloudPublisher()
{
  this->declare_parameter("output_muxed_pointcloud_topic_name", "/muxed/pointcloud");
  this->declare_parameter("output_muxed_pointcloud_queue_size", 10);

  std::string output_muxed_pointcloud_topic_name = this->get_parameter("output_muxed_pointcloud_topic_name").as_string();
  int output_muxed_pointcloud_queue_size = this->get_parameter("output_muxed_pointcloud_queue_size").as_int();

  m_muxed_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_muxed_pointcloud_topic_name, output_muxed_pointcloud_queue_size);
  RCLCPP_INFO(this->get_logger(), "Muxed Pointcloud Publisher is publishing to topic: %s with queue size %d", 
    output_muxed_pointcloud_topic_name.c_str(), output_muxed_pointcloud_queue_size);
}

void PointCloudMuxNode::initializeMuxTimer()
{
  this->declare_parameter("mux_timer_period", 0.1);

  double mux_timer_period = this->get_parameter("mux_timer_period").as_double();

  m_mux_timer = this->create_wall_timer(std::chrono::milliseconds((int)(mux_timer_period * 1000)),
    std::bind(&PointCloudMuxNode::publishMuxedPointCloud, this));
  RCLCPP_INFO(this->get_logger(), "Mux Timer is set to publish muxed pointcloud every %f seconds.", mux_timer_period);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMuxNode>());
  rclcpp::shutdown();
  return 0;
}