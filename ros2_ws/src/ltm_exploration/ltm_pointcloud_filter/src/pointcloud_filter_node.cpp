/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#include "ltm_pointcloud_filter/pointcloud_filter_node.hpp"

#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>

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
: Node("pointcloud_filter_node")
{
  initializePointcloudSubscriber();
  initializePointcloudPublisher();
  initializeTransformListener();
  RCLCPP_INFO(get_logger(), "Point Cloud Filter Node has been initialized");

  // Determine if visualization debugging is enabled
  this->declare_parameter("visualize_debug", false);
  bool visualize_debug = this->get_parameter("visualize_debug").as_bool();
  if (!visualize_debug)
  {
    RCLCPP_INFO(get_logger(), "Visualization debugging disabled");
    return;
  }
  RCLCPP_INFO(get_logger(), "Visualization debugging enabled");
  initializeCropBoxVisualizationPublisher();
  initializeVisualizationTimer();
}

PointCloudFilterNode::~PointCloudFilterNode()
{
  RCLCPP_WARN(get_logger(), "Point Cloud Filter Node has been terminated");
}

void PointCloudFilterNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  cropPointCloud(cloud, cloud_filtered);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  transformPointCloud(cloud_filtered, cloud_transformed, m_output_pointcloud_frame_id, msg->header.frame_id);

  publishFilteredPointCloud(cloud_transformed, msg->header.stamp);
}

void PointCloudFilterNode::publishFilteredPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  const rclcpp::Time& stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.frame_id = m_output_pointcloud_frame_id;
  msg.header.stamp = stamp;
  m_pointcloud_pub->publish(msg);
}

void PointCloudFilterNode::visualizationTimerCallback()
{
  publishCropBoxVisualization();
}

void PointCloudFilterNode::publishCropBoxVisualization()
{
  // Get the min and max points of the crop box in (x, y, z, 1) format
  Eigen::Vector4f min_pt = m_crop_box.getMin();
  Eigen::Vector4f max_pt = m_crop_box.getMax();

  // Create a box of markers
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = m_input_pointcloud_frame_id;
  msg.header.stamp = get_clock()->now();
  msg.type = visualization_msgs::msg::Marker::CUBE;
  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.pose.position.x = (max_pt[X] + min_pt[X]) / 2;
  msg.pose.position.y = (max_pt[Y] + min_pt[Y]) / 2;
  msg.pose.position.z = (max_pt[Z] + min_pt[Z]) / 2;
  msg.scale.x = max_pt[X] - min_pt[X];
  msg.scale.y = max_pt[Y] - min_pt[Y];
  msg.scale.z = max_pt[Z] - min_pt[Z];
  msg.color.r = 0.0;
  msg.color.g = 1.0;
  msg.color.b = 0.0;
  msg.color.a = 0.5;

  m_crop_box_visualization_pub->publish(msg);
}

void PointCloudFilterNode::cropPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output)
{
  m_crop_box.setInputCloud(cloud_input);
  m_crop_box.filter(*cloud_output);
}

void PointCloudFilterNode::transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output, const std::string& target_frame,
  const std::string& source_frame) const
{
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    pcl_ros::transformPointCloud(target_frame, rclcpp::Time(0), *cloud_input, source_frame, *cloud_output, *m_tf_buffer);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
    return;
  }
}

void PointCloudFilterNode::initializePointcloudSubscriber()
{
  this->declare_parameter("input_pointcloud_topic_name", "pointcloud");
  this->declare_parameter("input_pointcloud_topic_queue_size", 10);
  this->declare_parameter("input_pointcloud_topic_frame_id", "radar");

  std::string input_pointcloud_topic = this->get_parameter("input_pointcloud_topic_name").as_string();
  int input_pointcloud_queue_size = this->get_parameter("input_pointcloud_topic_queue_size").as_int();
  m_input_pointcloud_frame_id = this->get_parameter("input_pointcloud_topic_frame_id").as_string();
  RCLCPP_INFO(get_logger(), "Subscribing to %s with queue size %d in frame %s",
    input_pointcloud_topic.c_str(), input_pointcloud_queue_size, m_input_pointcloud_frame_id.c_str());

  m_pointcloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_pointcloud_topic, input_pointcloud_queue_size,
    std::bind(&PointCloudFilterNode::pointcloudCallback, this, std::placeholders::_1));
}

void PointCloudFilterNode::initializePointcloudPublisher()
{
  this->declare_parameter("output_pointcloud_topic_name", "filtered_pointcloud");
  this->declare_parameter("output_pointcloud_topic_queue_size", 10);
  this->declare_parameter("output_pointcloud_topic_frame_id", "map");

  std::string output_pointcloud_topic = this->get_parameter("output_pointcloud_topic_name").as_string();
  int output_pointcloud_queue_size = this->get_parameter("output_pointcloud_topic_queue_size").as_int();
  m_output_pointcloud_frame_id = this->get_parameter("output_pointcloud_topic_frame_id").as_string();
  RCLCPP_INFO(get_logger(), "Publishing to %s with queue size %d in frame %s",
    output_pointcloud_topic.c_str(), output_pointcloud_queue_size, m_output_pointcloud_frame_id.c_str());

  m_pointcloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_pointcloud_topic, output_pointcloud_queue_size);
}

void PointCloudFilterNode::initializeVisualizationTimer()
{
  this->declare_parameter("visualization_timer_period", 0.1);
  double visualization_timer_period = this->get_parameter("visualization_timer_period").as_double();
  RCLCPP_INFO(get_logger(), "Visualization timer period: %f", visualization_timer_period);

  m_visualization_timer = create_wall_timer(std::chrono::duration<double>(visualization_timer_period),
    std::bind(&PointCloudFilterNode::visualizationTimerCallback, this));
}

void PointCloudFilterNode::initializeCropBoxVisualizationPublisher()
{
  this->declare_parameter("crop_box_visualization_topic_name", "crop_box_visualization");
  this->declare_parameter("crop_box_visualization_topic_queue_size", 10);

  std::string crop_box_visualization_topic = this->get_parameter("crop_box_visualization_topic_name").as_string();
  int crop_box_visualization_queue_size = this->get_parameter("crop_box_visualization_topic_queue_size").as_int();
  RCLCPP_INFO(get_logger(), "Publishing crop box visualization to %s with queue size %d",
    crop_box_visualization_topic.c_str(), crop_box_visualization_queue_size);

  m_crop_box_visualization_pub = create_publisher<visualization_msgs::msg::Marker>(
    crop_box_visualization_topic, crop_box_visualization_queue_size);
}

void PointCloudFilterNode::initializeTransformListener()
{
  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);
}

void PointCloudFilterNode::initializeCropBox()
{
  this->declare_parameter("crop_box.x_min", -1.0);
  this->declare_parameter("crop_box.x_max", 1.0);
  this->declare_parameter("crop_box.y_min", -1.0);
  this->declare_parameter("crop_box.y_max", 1.0);
  this->declare_parameter("crop_box.z_min", -1.0);
  this->declare_parameter("crop_box.z_max", 1.0);

  double crop_box_x_min = this->get_parameter("crop_box.x_min").as_double();
  double crop_box_x_max = this->get_parameter("crop_box.x_max").as_double();
  double crop_box_y_min = this->get_parameter("crop_box.y_min").as_double();
  double crop_box_y_max = this->get_parameter("crop_box.y_max").as_double();
  double crop_box_z_min = this->get_parameter("crop_box.z_min").as_double();
  double crop_box_z_max = this->get_parameter("crop_box.z_max").as_double();

  m_crop_box.setMin(Eigen::Vector4f(crop_box_x_min, crop_box_y_min, crop_box_z_min, 1.0));
  m_crop_box.setMax(Eigen::Vector4f(crop_box_x_max, crop_box_y_max, crop_box_z_max, 1.0));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
