/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 06-08-2024.
 */

#ifndef LTM_POINTCLOUD_FILTER__VOXEL_GRID_FILTER_NODE_HPP_
#define LTM_POINTCLOUD_FILTER__VOXEL_GRID_FILTER_NODE_HPP_

#include <ltm_pointcloud_filter/pointcloud_filter_node.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <string>

namespace LTM {
  class VoxelGridFilter: public PointCloudFilterNode {
    public:
      VoxelGridFilter();
      ~VoxelGridFilter() = default;

    private:
      void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) override;
      void initialLeafSize();

      pcl::VoxelGrid<pcl::PointXYZ> m_voxel_grid_filter;

  }; // class VoxelGridFilter
} // namespace LTM

#endif // LTM_POINTCLOUD_FILTER__VOXEL_GRID_FILTER_NODE_HPP_

// End of file: ltm_pointcloud_filter/include/ltm_pointcloud_filter/voxel_grid_filter.hpp