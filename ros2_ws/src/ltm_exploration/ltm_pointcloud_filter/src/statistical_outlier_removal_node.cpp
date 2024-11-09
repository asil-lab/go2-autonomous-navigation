/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 07-08-2024.
 */

#include <ltm_pointcloud_filter/statistical_outlier_removal_node.hpp>

using namespace LTM;

StatisticalPointcloudFilter::StatisticalPointcloudFilter()
: PointCloudFilterNode("statistical_outlier_removal_node")
{
    initializeStatisticalOutlierRemovalParameters();
}

void StatisticalPointcloudFilter::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    m_statistical_outlier_removal.setInputCloud(cloud_in);
    m_statistical_outlier_removal.filter(*cloud_out);
}

void StatisticalPointcloudFilter::initializeStatisticalOutlierRemovalParameters()
{
    this->declare_parameter("mean_k", 50);
    this->declare_parameter("stddev_mul_thresh", 1.0);

    int mean_k = this->get_parameter("mean_k").as_int();
    double stddev_mul_thresh = this->get_parameter("stddev_mul_thresh").as_double();

    m_statistical_outlier_removal.setMeanK(mean_k);
    m_statistical_outlier_removal.setStddevMulThresh(stddev_mul_thresh);
    RCLCPP_INFO(this->get_logger(), 
        "Statistical Outlier Removal Parameters with mean_k: %d, stddev_mul_thresh: %f",
        mean_k, stddev_mul_thresh);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatisticalPointcloudFilter>());
    rclcpp::shutdown();
    return 0;
}