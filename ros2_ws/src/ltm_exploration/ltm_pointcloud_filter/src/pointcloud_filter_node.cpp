/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#include "ltm_pointcloud_filter/pointcloud_filter_node.hpp"

using namespace LTM;

PointCloudFilterNode::PointCloudFilterNode()
: Node("pointcloud_filter_node")
{
  // Initialize ROS communication
  initializeInputPointcloudSubscriber();
  initializeOutputPointcloudPublisher();
  initializeTransformListener();

  RCLCPP_INFO(get_logger(), "Point Cloud Filter Node has been initialized");
}

PointCloudFilterNode::~PointCloudFilterNode()
{
  RCLCPP_WARN(get_logger(), "Point Cloud Filter Node has been terminated");
}

void PointCloudFilterNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud_in);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
  // filterPointCloud(cloud_in, cloud_out);

  publishFilteredPointCloud(cloud_out, msg->header.stamp);
}

void PointCloudFilterNode::publishFilteredPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  const rclcpp::Time& stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = m_output_pointcloud_frame_id;
  msg.header.stamp = stamp;
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
  this->declare_parameter("input_pointcloud_topic_name", "pointcloud");
  this->declare_parameter("input_pointcloud_topic_queue_size", 10);
  this->declare_parameter("input_pointcloud_topic_frame_id", "radar");

  std::string input_pointcloud_topic = this->get_parameter("input_pointcloud_topic_name").as_string();
  int input_pointcloud_queue_size = this->get_parameter("input_pointcloud_topic_queue_size").as_int();
  m_input_pointcloud_frame_id = this->get_parameter("input_pointcloud_topic_frame_id").as_string();
  RCLCPP_INFO(get_logger(), "Subscribing to %s with queue size %d in frame %s",
    input_pointcloud_topic.c_str(), input_pointcloud_queue_size, m_input_pointcloud_frame_id.c_str());

  m_input_pointcloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(input_pointcloud_topic, 
    input_pointcloud_queue_size, std::bind(&PointCloudFilterNode::pointcloudCallback, this, std::placeholders::_1));
}

void PointCloudFilterNode::initializeOutputPointcloudPublisher()
{
  this->declare_parameter("output_pointcloud_topic_name", "filtered_pointcloud");
  this->declare_parameter("output_pointcloud_topic_queue_size", 10);
  this->declare_parameter("output_pointcloud_topic_frame_id", "map");

  std::string output_pointcloud_topic = this->get_parameter("output_pointcloud_topic_name").as_string();
  int output_pointcloud_queue_size = this->get_parameter("output_pointcloud_topic_queue_size").as_int();
  m_output_pointcloud_frame_id = this->get_parameter("output_pointcloud_topic_frame_id").as_string();
  RCLCPP_INFO(get_logger(), "Publishing to %s with queue size %d in frame %s",
    output_pointcloud_topic.c_str(), output_pointcloud_queue_size, m_output_pointcloud_frame_id.c_str());

  m_output_pointcloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_pointcloud_topic, output_pointcloud_queue_size);
}

void PointCloudFilterNode::initializeTransformListener()
{
  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
