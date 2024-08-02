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

    void initializePointcloudBuffer();
    void updateInputPointcloudMsg(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void bufferPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void clearPointcloudBuffer();
    void publishPointcloudBuffer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string target_frame) const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertPointCloud2ToPCL(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
    sensor_msgs::msg::PointCloud2::SharedPtr convertPCLToPointCloud2(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pointcloud_buffer;
    sensor_msgs::msg::PointCloud2::SharedPtr m_recent_input_pointcloud_msg;

    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_subscription;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher;

  }; // class PointCloudBufferNode
} // namespace LTM

#endif // LTM_POINTCLOUD_BUFFER__POINTCLOUD_BUFFER_NODE_HPP_