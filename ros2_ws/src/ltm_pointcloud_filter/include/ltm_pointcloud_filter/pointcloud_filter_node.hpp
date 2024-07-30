/*
 * Project Lava Tube Mapping
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef LTM_POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_
#define LTM_POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/common.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>

#include <string>

namespace LTMPointcloudFilterNode
{
  class PointCloudFilterNode : public rclcpp::Node
  {
  public:
    PointCloudFilterNode();
    ~PointCloudFilterNode();

  private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr convertPointCloud2ToPCL(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
    sensor_msgs::msg::PointCloud2::SharedPtr convertPCLToPointCloud2(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;

    void configureRosSubscribers(bool in_simulation);
    void configureRosPublishers(bool in_simulation);

    enum Axis { X, Y, Z };

    double m_sac_segmentation_distance_threshold;
    int m_sac_segmentation_max_iterations;
    double m_sac_segmentation_probability;

    double m_voxel_grid_leaf_size;


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_raw_pointcloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_filtered_pointcloud_pub;


  }; // class PointCloudFilterNode
}   // namespace LTMPointcloudFilterNode

#endif  // LTM_POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_