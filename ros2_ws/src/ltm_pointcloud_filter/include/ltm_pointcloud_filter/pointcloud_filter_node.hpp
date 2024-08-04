/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#ifndef LTM_POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_
#define LTM_POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/common.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>

#include <urdf/model.h>
#include <pcl/PolygonMesh.h>

#include <string>

#include <ltm_pointcloud_filter/ground_plane_removal.hpp>
#include <ltm_pointcloud_filter/robot_cluster_removal.hpp>

namespace LTM // TODO: Change this to LTM
{
  class PointCloudFilterNode : public rclcpp::Node
  {
  public:
    PointCloudFilterNode();
    ~PointCloudFilterNode();

  private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishFilteredPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      const std::string& frame_id, const rclcpp::Time& stamp);

    void removeGroundPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr convertPointCloud2ToPCL(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
    sensor_msgs::msg::PointCloud2::SharedPtr convertPCLToPointCloud2(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const;

    void initializeGroundPlaneRemoval();
    void initializeRobotClusterRemoval();

    void configureRosSubscribers(bool in_simulation);
    void configureRosPublishers(bool in_simulation);

    enum Axis { X, Y, Z };

    double m_voxel_grid_leaf_size;

    std::unique_ptr<LTM::GroundPlaneRemoval> m_ground_plane_removal;
    std::unique_ptr<LTM::RobotClusterRemoval> m_robot_cluster_removal;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_raw_pointcloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_filtered_pointcloud_pub;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox3D>::SharedPtr m_bounding_box_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_array_pub;

  }; // class PointCloudFilterNode
}   // namespace LTMPointcloudFilterNode

#endif  // LTM_POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_