/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 17-10-2024.
 */

#ifndef LTM_POINTCLOUD_MULTIPLEXER__POINTCLOUD_MUX_NODE_HPP_
#define LTM_POINTCLOUD_MULTIPLEXER__POINTCLOUD_MUX_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

namespace LTM
{
  class PointCloudMuxNode : public rclcpp::Node
  {
  public:
    PointCloudMuxNode();
    ~PointCloudMuxNode();

  private:
    void lidarPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void cameraPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishMuxedPointCloud();

    pcl::PointCloud<pcl::PointXYZ>::Ptr mergePointClouds(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud) const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertPointCloud2ToPCL(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
    sensor_msgs::msg::PointCloud2::SharedPtr convertPCLToPointCloud2(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;

    void resetLidarPointCloud();
    void resetCameraPointCloud();

    void initializeLidarPointCloudSubscriber();
    void initializeCameraPointCloudSubscriber();
    void initializeMuxedPointCloudPublisher();
    void initializeMuxTimer();

    sensor_msgs::msg::PointCloud2::SharedPtr m_lidar_pointcloud_msg;
    sensor_msgs::msg::PointCloud2::SharedPtr m_camera_pointcloud_msg;
    std::string m_output_mux_pointcloud_frame_id;

    rclcpp::TimerBase::SharedPtr m_mux_timer;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidar_pointcloud_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_camera_pointcloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_muxed_pointcloud_pub;

  }; // class PointCloudMuxNode
} // namespace LTM

#endif // LTM_POINTCLOUD_MULTIPLEXER__POINTCLOUD_MUX_NODE_HPP_
