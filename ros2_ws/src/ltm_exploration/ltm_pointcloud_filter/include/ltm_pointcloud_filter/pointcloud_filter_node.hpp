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
#include <visualization_msgs/msg/marker.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>

#include <urdf/model.h>
#include <pcl/PolygonMesh.h>

#include <string>

#include <ltm_pointcloud_filter/ground_plane_segmentation.hpp>

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
      const rclcpp::Time& stamp);

    void visualizationTimerCallback();
    void publishCropBoxVisualization();

    void cropPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output);
    void transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output, const std::string& target_frame,
      const std::string& source_frame) const;

    void initializeGroundPlaneSegmentation();
    void initializeCropBoxFilter();

    void initializeLidarPointcloudSubscriber();
    void initializeCameraPointcloudSubscriber();
    void initializePointcloudPublisher();
    void initializeTransformListener();

    void initializeVisualizationTimer();
    void initializeCropBoxVisualizationPublisher();

    enum { X, Y, Z, ROLL, PITCH, YAW };

    pcl::CropBox<pcl::PointXYZ> m_crop_box;

    std::unique_ptr<GroundPlaneSegmentation> m_ground_plane_segmentation;

    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;
    std::string m_input_pointcloud_frame_id;
    std::string m_output_pointcloud_frame_id;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidar_pointcloud_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_camera_pointcloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_pub;

    rclcpp::TimerBase::SharedPtr m_visualization_timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_crop_box_visualization_pub;

  }; // class PointCloudFilterNode
}   // namespace LTMPointcloudFilterNode

#endif  // LTM_POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_