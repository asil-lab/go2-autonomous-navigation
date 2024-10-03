/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 03-10-2024.
 */

#include "ltm_pointcloud_transformer/pointcloud_transformer_node.hpp"

#include <pcl/common/common.h>
#include <pcl_ros/transforms.hpp>

using namespace LTM;

PointCloudTransformerNode::PointCloudTransformerNode()
: Node("ltm_pointcloud_transformer_node")
{
  // Initialize the transform listener
  initializeTransformListener();

  // Initialize the input pointcloud subscriber
  initializeInputPointCloudSubscriber();

  // Initialize the output pointcloud publisher
  initializeOutputPointCloudPublisher();

  // Initialize the parameters
  initializeParameters();

  RCLCPP_INFO(this->get_logger(), "LTM Pointcloud Transformer Node has been initialized.");
}

PointCloudTransformerNode::~PointCloudTransformerNode()
{
  // m_input_pointcloud_sub.reset();
  // m_output_pointcloud_pub.reset();
  RCLCPP_WARN(this->get_logger(), "LTM Pointcloud Transformer Node has been destroyed.");
}

void PointCloudTransformerNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Transform the pointcloud
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_output(new sensor_msgs::msg::PointCloud2);
  transformPointCloud(msg, cloud_output);

  // Publish the transformed pointcloud
  m_output_pointcloud_pub->publish(*cloud_output);
}

void PointCloudTransformerNode::transformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_input,
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_output)
{
  // Set the frame ID
  cloud_output->header.frame_id = m_target_frame;

  // Transform the point cloud from the source frame to the target frame
  try {
    geometry_msgs::msg::TransformStamped transform_stamped = m_tf_buffer->lookupTransform(
      m_target_frame, m_source_frame, cloud_input->header.stamp);
    pcl_ros::transformPointCloud(m_target_frame, transform_stamped, *cloud_input, *cloud_output);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform pointcloud: %s", ex.what());
    return;
  }
}

void PointCloudTransformerNode::initializeTransformListener()
{
  // Initialize the transform listener
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}

void PointCloudTransformerNode::initializeInputPointCloudSubscriber()
{
  // Get the input pointcloud topic parameter from the parameter server
  this->declare_parameter("input_pointcloud_topic_name", "input_point_cloud");
  std::string input_pointcloud_topic = this->get_parameter("input_pointcloud_topic_name").as_string();
  RCLCPP_INFO(this->get_logger(), "Subscribe to input pointcloud topic: %s", input_pointcloud_topic.c_str());

  // Create the input pointcloud subscriber
  m_input_pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_pointcloud_topic, rclcpp::SensorDataQoS(), 
    std::bind(&PointCloudTransformerNode::pointcloudCallback, this, std::placeholders::_1));
}

void PointCloudTransformerNode::initializeOutputPointCloudPublisher()
{
  // Get the output pointcloud topic parameter from the parameter server
  this->declare_parameter("output_pointcloud_topic_name", "output_point_cloud");
  std::string output_pointcloud_topic = this->get_parameter("output_pointcloud_topic_name").as_string();
  RCLCPP_INFO(this->get_logger(), "Publish output pointcloud topic: %s", output_pointcloud_topic.c_str());

  // Create the output pointcloud publisher
  m_output_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_pointcloud_topic, 1);
}

void PointCloudTransformerNode::initializeParameters()
{
  // Get the source frame parameter from the parameter server
  declare_parameter("source_frame_id", "odom");
  m_source_frame = this->get_parameter("source_frame_id").as_string();
  RCLCPP_INFO(this->get_logger(), "Source frame: %s", m_source_frame.c_str());

  // Get the target frame parameter from the parameter server
  declare_parameter("target_frame_id", "base_footprint");
  m_target_frame = this->get_parameter("target_frame_id").as_string();
  RCLCPP_INFO(this->get_logger(), "Target frame: %s", m_target_frame.c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<PointCloudTransformerNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}