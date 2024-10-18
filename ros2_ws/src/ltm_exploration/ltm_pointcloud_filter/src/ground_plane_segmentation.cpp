/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#include "ltm_pointcloud_filter/ground_plane_segmentation.hpp"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <vector>
#include <algorithm>
#include <functional>

namespace LTM
{
  GroundPlaneSegmentation::GroundPlaneSegmentation()
  {
    // Set up the SACSegmentation object and the ExtractIndices object
    m_sac_segmentation = std::make_unique<pcl::SACSegmentation<pcl::PointXYZ>>();
    m_extract_indices = std::make_unique<pcl::ExtractIndices<pcl::PointXYZ>>();

    // Optimize coefficients to minimize the error
    m_sac_segmentation->setOptimizeCoefficients(true);

    // Set the type of model to be fitted
    m_sac_segmentation->setModelType(pcl::SACMODEL_PLANE);

    // RANSAC is used because it is a robust method for fitting
    // a model to data that contains outliers.
    m_sac_segmentation->setMethodType(pcl::SAC_RANSAC);

    // Default values for SACSegmentation parameters
    m_sac_segmentation->setDistanceThreshold(0.01);
    m_sac_segmentation->setMaxIterations(1000);
    m_sac_segmentation->setProbability(0.99);
  }

  GroundPlaneSegmentation::~GroundPlaneSegmentation()
  {
    // Delete the pointers
    m_sac_segmentation.reset();
    m_extract_indices.reset();
  }

  void GroundPlaneSegmentation::segmentPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane) const
  {
    // Create the inliers and coefficients objects
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Find the plane in the point cloud and store the inliers and coefficients
    if (!findPlane(cloud, inliers, coefficients))
    {
      // If the plane was not found, return the original point cloud
      return;
    }

    // Remove the plane from the point cloud
    removePlane(cloud, cloud_plane, inliers);
  }

  bool GroundPlaneSegmentation::findPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients) const
  {
    // Perform the segmentation and store the inliers and coefficients
    m_sac_segmentation->setInputCloud(cloud);
    m_sac_segmentation->segment(*inliers, *coefficients);

    // Check if the plane was successfully segmented
    return !inliers->indices.empty();
  }

  void GroundPlaneSegmentation::removePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, pcl::PointIndices::Ptr inliers) const
  {
    // Extract the ground plane from the point cloud using the inliers
    m_extract_indices->setInputCloud(cloud);
    m_extract_indices->setIndices(inliers);
    m_extract_indices->setNegative(true);
    m_extract_indices->filter(*cloud_filtered);
  }

  void GroundPlaneSegmentation::configureSACSegmentationParameters(
    const double& distance_threshold, int max_iterations, const double& probability)
  {
    m_sac_segmentation->setDistanceThreshold(distance_threshold);
    m_sac_segmentation->setMaxIterations(max_iterations);
    m_sac_segmentation->setProbability(probability);
  }
} // namespace LTMPointcloudFilter

// End of file: ltm_pointcloud_filter/ground_plane_segmentation.cpp