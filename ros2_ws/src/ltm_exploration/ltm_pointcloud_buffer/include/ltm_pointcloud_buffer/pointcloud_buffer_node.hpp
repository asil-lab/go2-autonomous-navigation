/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 01-08-2024.
 */

#ifndef LTM_POINTCLOUD_BUFFER__POINTCLOUD_BUFFER_NODE_HPP_
#define LTM_POINTCLOUD_BUFFER__POINTCLOUD_BUFFER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ltm_shared_msgs/srv/get_point_cloud.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/common.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>

#include <string>

namespace LTM
{
  class PointCloudBufferNode : public rclcpp::Node
  {
  public:
    PointCloudBufferNode();
    ~PointCloudBufferNode();

  private:
    void timerCallback();
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void serviceCallback(
      const std::shared_ptr<ltm_shared_msgs::srv::GetPointCloud::Request> request,
      std::shared_ptr<ltm_shared_msgs::srv::GetPointCloud::Response> response);
    void publishPointcloudBuffer() const;

    void updateInputPointcloudMsg(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void bufferPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void removeRobotFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) const;
    void clearPointcloudBuffer();

    void initializeROSTopics();
    void initializeROSService();
    void initializeTFListener();
    void initializePointcloudBuffer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertPointCloud2ToPCL(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
    sensor_msgs::msg::PointCloud2::SharedPtr convertPCLToPointCloud2(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pointcloud_buffer;
    sensor_msgs::msg::PointCloud2::SharedPtr m_recent_input_pointcloud_msg;
    std::string m_source_frame;
    std::string m_target_frame;

    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_subscription;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher;
    rclcpp::Service<ltm_shared_msgs::srv::GetPointCloud>::SharedPtr m_get_pointcloud_service;

  }; // class PointCloudBufferNode
} // namespace LTM

#endif // LTM_POINTCLOUD_BUFFER__POINTCLOUD_BUFFER_NODE_HPP_