/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 07-08-2024.
 */

#ifndef LTM_POINTCLOUD_FILTER__STATISTICAL_OUTLIER_REMOVAL_NODE_HPP_
#define LTM_POINTCLOUD_FILTER__STATISTICAL_OUTLIER_REMOVAL_NODE_HPP_

#include <ltm_pointcloud_filter/pointcloud_filter_node.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace LTM
{
  class StatisticalPointcloudFilter : public PointCloudFilterNode {
    public:
      StatisticalPointcloudFilter();
      ~StatisticalPointcloudFilter() = default;

    private:
      void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) override;

      void initializeStatisticalOutlierRemovalParameters();

      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> m_statistical_outlier_removal;

  };  // class Statistical
} // namespace LTM

#endif  // LTM_POINTCLOUD_FILTER__STATISTICAL_OUTLIER_REMOVAL_NODE_HPP_

// End of file: ltm_pointcloud_filter/include/ltm_pointcloud_filter/statistical_outlier_removal.hpp