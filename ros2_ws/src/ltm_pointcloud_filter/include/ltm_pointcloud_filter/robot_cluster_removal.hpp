/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#ifndef LTM_POINTCLOUD_FILTER__ROBOT_CLUSTER_REMOVAL_HPP_
#define LTM_POINTCLOUD_FILTER__ROBOT_CLUSTER_REMOVAL_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <string>

namespace LTMPointcloudFilterNode {
  class RobotClusterRemoval {
    public:
      RobotClusterRemoval();
      ~RobotClusterRemoval();

      void setRobotModel(const std::string &robot_description);
      void setRobotModel(const urdf::Model &robot_model);

      void setTransformBuffer(const std::shared_ptr<tf2_ros::Buffer> &tf_buffer);

      void setRobotFrame(const std::string &robot_frame);
      void setWorldFrame(const std::string &world_frame);

      void setRobotRadius(const double &robot_radius);
      void setRobotHeight(const double &robot_height);

      void setClusterTolerance(const double &cluster_tolerance);
      void setMinClusterSize(const int &min_cluster_size);
      void setMaxClusterSize(const int &max_cluster_size);

      void removeRobotClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);

    private:
      urdf::Model m_robot_model_;
      std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
      std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

      std::string m_robot_frame;
      std::string m_world_frame;

      double m_cluster_tolerance;
      int m_min_cluster_size;
      int m_max_cluster_size;

      void transformPointCloud(const Eigen::Affine3d &transform, 
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);
  }; // class RobotClusterRemoval
} // namespace LTMPointcloudFilterNode

#endif // LTM_POINTCLOUD_FILTER__ROBOT_CLUSTER_REMOVAL_HPP_