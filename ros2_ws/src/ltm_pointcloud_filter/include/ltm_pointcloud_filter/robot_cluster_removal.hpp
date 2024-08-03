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
#include <tf2_ros/transform_listener.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace LTM {
  class RobotClusterRemoval {
    public:
      typedef std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> ClusterMeshMap;

      RobotClusterRemoval(rclcpp::Clock::SharedPtr clock);
      ~RobotClusterRemoval();

      void removeRobotCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output) const;

      bool setRobotModel(const std::string &robot_description);
      void setRobotMeshResolution(const double& resolution);

      pcl::PointCloud<pcl::PointXYZ>::Ptr getRobotMesh(const std::string &link_name) const;
      
    private:
      void transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output, const std::string &target_frame) const;
      void drawRobotMeshes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output) const;
      void clusterRobotMeshes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output) const;
      void generateRobotMeshes();

      void initializeTransformListener(const rclcpp::Clock::SharedPtr clock);
      void setClock(const rclcpp::Clock::SharedPtr clock);

      urdf::Model m_robot_model;
      std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
      std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
      rclcpp::Clock::SharedPtr m_clock;

      ClusterMeshMap m_robot_meshes;
      double m_robot_mesh_resolution;

  }; // class RobotClusterRemoval
} // namespace LTMPointcloudFilterNode

#endif // LTM_POINTCLOUD_FILTER__ROBOT_CLUSTER_REMOVAL_HPP_