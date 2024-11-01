/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 06-08-2024.
 */

#ifndef LTM_POINTCLOUD_FILTER__VOXEL_GRID_FILTER_HPP_
#define LTM_POINTCLOUD_FILTER__VOXEL_GRID_FILTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <string>

namespace LTM {
  class VoxelGridFilter: public rclcpp::Node {
    public:
      VoxelGridFilter();
      ~VoxelGridFilter() = default;

      void filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output);

      void setLeafSize(const double& x, const double& y, const double& z);

    private:
      pcl::VoxelGrid<pcl::PointXYZ> m_voxel_grid_filter;

      void initializeInputPointcloudSubscriber();
      void initializeOutputPointcloudPublisher();

      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_input_pointcloud_sub;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_output_pointcloud_pub;

  }; // class VoxelGridFilter
} // namespace LTM

#endif // LTM_POINTCLOUD_FILTER__VOXEL_GRID_FILTER_HPP_

// End of file: ltm_pointcloud_filter/include/ltm_pointcloud_filter/voxel_grid_filter.hpp