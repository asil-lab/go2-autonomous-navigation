/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#ifndef LTM_POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_
#define LTM_POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/common.h>

#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>

#include <string>

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

    virtual void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

    void initializeInputPointcloudSubscriber();
    void initializeOutputPointcloudPublisher();
    void initializeTransformListener();

    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;

    std::string m_input_pointcloud_frame_id;
    std::string m_output_pointcloud_frame_id;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_input_pointcloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_output_pointcloud_pub;

  }; // class PointCloudFilterNode
}   // namespace LTMPointcloudFilterNode

#endif  // LTM_POINTCLOUD_FILTER__POINTCLOUD_FILTER_NODE_HPP_