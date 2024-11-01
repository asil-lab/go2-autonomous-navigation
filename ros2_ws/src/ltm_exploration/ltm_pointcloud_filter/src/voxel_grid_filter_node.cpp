/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 06-08-2024.
 */

#include <ltm_pointcloud_filter/voxel_grid_filter_node.hpp>

using namespace LTM;

VoxelGridFilter::VoxelGridFilter()
{
  // Set default leaf size to 0.1 m
  double default_leaf_size = 0.1;
  setLeafSize(default_leaf_size, default_leaf_size, default_leaf_size);
}

void VoxelGridFilter::filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output)
{
  m_voxel_grid_filter.setInputCloud(cloud_input);
  m_voxel_grid_filter.filter(*cloud_output);
}

void VoxelGridFilter::setLeafSize(const double& x, const double& y, const double& z)
{
  m_voxel_grid_filter.setLeafSize(x, y, z);
}



// End of file: ltm_pointcloud_filter/src/voxel_grid_filter.cpp