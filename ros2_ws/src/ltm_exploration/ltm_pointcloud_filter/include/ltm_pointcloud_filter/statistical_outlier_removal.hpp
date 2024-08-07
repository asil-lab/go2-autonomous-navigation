/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 07-08-2024.
 */

#ifndef LTM_POINTCLOUD_FILTER__STATISTICAL_OUTLIER_REMOVAL_HPP_
#define LTM_POINTCLOUD_FILTER__STATISTICAL_OUTLIER_REMOVAL_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace LTM
{
  class StatisticalPointcloudFilter
  {
    public:
        StatisticalPointcloudFilter();
        ~StatisticalPointcloudFilter() = default;

        void filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output);
        void setMeanK(const int& mean_k);
        void setStddevMulThresh(const double& stddev_mul_thresh);

    private:
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> m_statistical_outlier_removal;

  };  // class Statistical
} // namespace LTM

#endif  // LTM_POINTCLOUD_FILTER__STATISTICAL_OUTLIER_REMOVAL_HPP_

// End of file: ltm_pointcloud_filter/include/ltm_pointcloud_filter/statistical_outlier_removal.hpp