/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 31-07-2024.
 */

#ifndef LTM_POINTCLOUD_FILTER__GROUND_PLANE_SEGMENTATION_HPP_
#define LTM_POINTCLOUD_FILTER__GROUND_PLANE_SEGMENTATION_HPP_

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace LTM
{
  class GroundPlaneSegmentation
  {
  public:
    GroundPlaneSegmentation();
    ~GroundPlaneSegmentation();

    void segmentPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane) const;
    bool findPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients) const;
    void removePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, pcl::PointIndices::Ptr inliers) const;

    void configureSACSegmentationParameters(const double& distance_threshold,
      int max_iterations, const double& probability);

  private:
    std::unique_ptr<pcl::SACSegmentation<pcl::PointXYZ>> m_sac_segmentation;
    std::unique_ptr<pcl::ExtractIndices<pcl::PointXYZ>> m_extract_indices;

  }; // class GroundPlaneSegmentation
} // namespace LTMPointcloudFilter

#endif // LTM_POINTCLOUD_FILTER__GROUND_PLANE_SEGMENTATION_HPP_