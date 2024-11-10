/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 31-07-2024.
 */

#ifndef LTM_POINTCLOUD_FILTER__GROUND_PLANE_SEGMENTATION_NODE_HPP_
#define LTM_POINTCLOUD_FILTER__GROUND_PLANE_SEGMENTATION_NODE_HPP_

#include <ltm_pointcloud_filter/pointcloud_filter_node.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace LTM
{
  class GroundPlaneSegmentation : public PointCloudFilterNode {
  public:
    GroundPlaneSegmentation();
    ~GroundPlaneSegmentation() = default;

  private:
    void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) override;

    bool findPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients) const;
    void removePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, pcl::PointIndices::Ptr inliers) const;

    void initializeSacSegmentationParameters();

    std::unique_ptr<pcl::SACSegmentation<pcl::PointXYZ>> m_sac_segmentation;
    std::unique_ptr<pcl::ExtractIndices<pcl::PointXYZ>> m_extract_indices;

  }; // class GroundPlaneSegmentation
} // namespace LTMPointcloudFilter

#endif // LTM_POINTCLOUD_FILTER__GROUND_PLANE_SEGMENTATION_NODE_HPP_
