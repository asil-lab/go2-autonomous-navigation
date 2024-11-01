/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 06-08-2024.
 */

#include <ltm_pointcloud_filter/voxel_grid_filter_node.hpp>

using namespace LTM;

VoxelGridFilter::VoxelGridFilter()
: PointCloudFilterNode("voxel_grid_filter_node")
{
  initialLeafSize();
}

void VoxelGridFilter::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
  m_voxel_grid_filter.setInputCloud(cloud_in);
  m_voxel_grid_filter.filter(*cloud_out);
}

void VoxelGridFilter::initialLeafSize()
{
  this->declare_parameter("leaf_size_x", 0.1);
  this->declare_parameter("leaf_size_y", 0.1);
  this->declare_parameter("leaf_size_z", 0.1);

  double leaf_size_x, leaf_size_y, leaf_size_z;
  leaf_size_x = this->get_parameter("leaf_size_x").as_double();
  leaf_size_y = this->get_parameter("leaf_size_y").as_double();
  leaf_size_z = this->get_parameter("leaf_size_z").as_double();

  m_voxel_grid_filter.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
  RCLCPP_INFO(get_logger(), "Voxel Grid Filter has been initialized with leaf size: %f m, %f m, %f m",
    leaf_size_x, leaf_size_y, leaf_size_z);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto voxel_grid_filter = std::make_shared<VoxelGridFilter>();
  rclcpp::spin(voxel_grid_filter);
  rclcpp::shutdown();
  return 0;
}

// End of file: ltm_pointcloud_filter/src/voxel_grid_filter.cpp