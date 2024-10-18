/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 13-08-2024.
 */

#include <ltm_go2_driver/point_cloud_processing.hpp>

using namespace LTM;

PointCloudProcessing::PointCloudProcessing() : Node(POINT_CLOUD_PROCESSING_NODE_NAME)
{
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
  publishPointCloud(msg);
}

void PointCloudProcessing::publishPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  changeFrameId(msg);
  point_cloud_pub_->publish(*msg);
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
}