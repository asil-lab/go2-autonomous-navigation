/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#ifndef LTM_POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_
#define LTM_POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/common.h>

#include <string>

#include <ltm_pointcloud_filter/ground_plane_removal.hpp>
#include <ltm_pointcloud_filter/robot_cluster_removal.hpp>
#include <ltm_pointcloud_filter/voxel_grid_filter.hpp>

namespace LTM // TODO: Change this to LTM
{
  class PointCloudFilterNode : public rclcpp::Node
  {
  public:
    PointCloudFilterNode();
    ~PointCloudFilterNode();

  private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
      unitree_go::msg::SportModeState> SyncPolicy;

    // void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg,
      const unitree_go::msg::SportModeState::SharedPtr sport_mode_state_msg);

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
    void initializeVoxelGridFilter();

    void configureRosSubscribers(bool in_simulation);
    void configureRosPublishers(bool in_simulation);

    enum Axis { X, Y, Z };

    double m_voxel_grid_leaf_size;

    std::unique_ptr<LTM::GroundPlaneRemoval> m_ground_plane_removal;
    std::unique_ptr<LTM::RobotClusterRemoval> m_robot_cluster_removal;
    std::unique_ptr<LTM::VoxelGridFilter> m_voxel_grid_filter;

    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_raw_pointcloud_sub;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_raw_pointcloud_sub;
    message_filters::Subscriber<unitree_go::msg::SportModeState> m_sport_mode_state_sub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> m_sync;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_filtered_pointcloud_pub;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox3D>::SharedPtr m_bounding_box_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_array_pub;

  }; // class PointCloudFilterNode
}   // namespace LTMPointcloudFilterNode

#endif  // LTM_POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_