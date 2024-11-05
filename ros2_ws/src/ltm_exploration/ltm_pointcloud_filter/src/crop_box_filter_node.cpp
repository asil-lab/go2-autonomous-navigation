/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 01-11-2024.
 */

#include "ltm_pointcloud_filter/crop_box_filter_node.hpp"

using namespace LTM;

CropBoxFilterNode::CropBoxFilterNode()
: PointCloudFilterNode("crop_box_filter_node")
{
  initializeCropBox();
}

void CropBoxFilterNode::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
  m_crop_box_filter.setInputCloud(cloud_in);
  m_crop_box_filter.filter(*cloud_out);
}

void CropBoxFilterNode::initializeCropBox()
{
  this->declare_parameter("crop_box_min_x", -1.0);
  this->declare_parameter("crop_box_min_y", -1.0);
  this->declare_parameter("crop_box_min_z", -1.0);
  this->declare_parameter("crop_box_max_x", 1.0);
  this->declare_parameter("crop_box_max_y", 1.0);
  this->declare_parameter("crop_box_max_z", 1.0);

  double min_x, min_y, min_z, max_x, max_y, max_z;
  min_x = this->get_parameter("crop_box_min_x").as_double();
  min_y = this->get_parameter("crop_box_min_y").as_double();
  min_z = this->get_parameter("crop_box_min_z").as_double();
  max_x = this->get_parameter("crop_box_max_x").as_double();
  max_y = this->get_parameter("crop_box_max_y").as_double();
  max_z = this->get_parameter("crop_box_max_z").as_double();

  // Set the crop box filter parameters
  Eigen::Vector4f min_point(min_x, min_y, min_z, 1.0);
  Eigen::Vector4f max_point(max_x, max_y, max_z, 1.0);
  m_crop_box_filter.setMin(min_point);
  m_crop_box_filter.setMax(max_point);
  m_crop_box_filter.setNegative(true);

  RCLCPP_INFO(get_logger(), "Crop box filter has been initialized with min point (%.2f, %.2f, %.2f) m, and max point (%.2f, %.2f, %.2f) m",
    min_x, min_y, min_z, max_x, max_y, max_z);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CropBoxFilterNode>());
  rclcpp::shutdown();
  return 0;
}
