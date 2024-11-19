/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 13-08-2024.
 */

#include <ltm_go2_driver/point_cloud_processing.hpp>

using namespace LTM;

PointCloudProcessing::PointCloudProcessing() : Node(POINT_CLOUD_PROCESSING_NODE_NAME)
{
  initializePointcloudMsg();
  initializeROS();
  RCLCPP_INFO(this->get_logger(), "Point Cloud Processing Node initialized.");
}

PointCloudProcessing::~PointCloudProcessing()
{
  point_cloud_sub_.reset();
  point_cloud_pub_.reset();
  RCLCPP_WARN(this->get_logger(), "Point Cloud Processing Node destroyed.");
}

void PointCloudProcessing::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Received Point Cloud Message.");
  updatePointCloud(msg);
  publishPointCloud(msg);
}

void PointCloudProcessing::publishPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  changeFrameId(msg);
  point_cloud_pub_->publish(*msg);
}

void PointCloudProcessing::serviceCallback(const std::shared_ptr<ltm_shared_msgs::srv::GetPointCloud::Request> request,
  std::shared_ptr<ltm_shared_msgs::srv::GetPointCloud::Response> response)
{
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received service request to get pointcloud");
  (void) request;

  // 
  response->point_cloud = *m_recent_input_pointcloud_msg;
  response->point_cloud.header.stamp = this->now();
  response->point_cloud.header.frame_id = POINT_CLOUD_PUB_FRAME_ID;
}

void PointCloudProcessing::updatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  m_recent_input_pointcloud_msg = msg;
}


void PointCloudProcessing::changeFrameId(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  msg->header.frame_id = POINT_CLOUD_PUB_FRAME_ID;
  msg->header.stamp = this->get_clock()->now();
}

void PointCloudProcessing::initializeROS()
{
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    POINT_CLOUD_PROCESSING_SUB_TOPIC, POINT_CLOUD_PROCESSING_SUB_QUEUE_SIZE,
    std::bind(&PointCloudProcessing::pointCloudCallback, this, std::placeholders::_1));

  point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    POINT_CLOUD_PROCESSING_PUB_TOPIC, POINT_CLOUD_PROCESSING_PUB_QUEUE_SIZE);

  get_pointcloud_service_ = this->create_service<ltm_shared_msgs::srv::GetPointCloud>(POINT_CLOUD_PROCESSING_GET_POINTCLOUD_SERVICE, 
    std::bind(&PointCloudProcessing::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));  
}

void PointCloudProcessing::initializePointcloudMsg()
{
  m_recent_input_pointcloud_msg = sensor_msgs::msg::PointCloud2::SharedPtr(new sensor_msgs::msg::PointCloud2);
}