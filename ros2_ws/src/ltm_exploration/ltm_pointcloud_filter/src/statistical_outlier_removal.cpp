/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 07-08-2024.
 */

#include <ltm_pointcloud_filter/statistical_outlier_removal.hpp>

using namespace LTM;

StatisticalPointcloudFilter::StatisticalPointcloudFilter()
{
    // Set default values for the statistical outlier removal filter
    m_statistical_outlier_removal.setMeanK(50);
    m_statistical_outlier_removal.setStddevMulThresh(1.0);
}

void StatisticalPointcloudFilter::filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output)
{
    m_statistical_outlier_removal.setInputCloud(cloud_input);
    m_statistical_outlier_removal.filter(*cloud_output);
}

void StatisticalPointcloudFilter::setMeanK(const int& mean_k)
{
    m_statistical_outlier_removal.setMeanK(mean_k);
}

void StatisticalPointcloudFilter::setStddevMulThresh(const double& stddev_mul_thresh)
{
    m_statistical_outlier_removal.setStddevMulThresh(stddev_mul_thresh);
}