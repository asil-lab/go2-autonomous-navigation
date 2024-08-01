/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#ifndef LTM_POINTCLOUD_FILTER__ROBOT_CLUSTER_REMOVAL_HPP_
#define LTM_POINTCLOUD_FILTER__ROBOT_CLUSTER_REMOVAL_HPP_

#include <rclcpp/rclcpp.hpp>
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

#include <string>
#include <unordered_map>
#include <vector>

namespace LTM {
  class RobotClusterRemoval {
    public:
      typedef std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> ClusterMeshMap;

      RobotClusterRemoval(rclcpp::Clock::SharedPtr clock);
      ~RobotClusterRemoval() = default;

      void setRobotModel(const std::string &robot_description);
      
    private:
      void generateRobotMeshes();

      urdf::Model m_robot_model_;
      std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
      std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

      ClusterMeshMap m_robot_meshes;

  }; // class RobotClusterRemoval
} // namespace LTMPointcloudFilterNode

#endif // LTM_POINTCLOUD_FILTER__ROBOT_CLUSTER_REMOVAL_HPP_