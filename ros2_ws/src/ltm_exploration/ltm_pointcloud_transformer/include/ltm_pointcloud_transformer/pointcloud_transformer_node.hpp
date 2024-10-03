/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 03-10-2024.
 */

#ifndef LTM_POINTCLOUD_TRANSFORMER__POINTCLOUD_TRANSFORMER_NODE_HPP_
#define LTM_POINTCLOUD_TRANSFORMER__POINTCLOUD_TRANSFORMER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <string>

namespace LTM
{
  class PointCloudTransformerNode : public rclcpp::Node
  {
  public:
    PointCloudTransformerNode();
    ~PointCloudTransformerNode();

  private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void transformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_input,
      sensor_msgs::msg::PointCloud2::SharedPtr cloud_output, const std::string &target_frame);

    void initializeTransformListener();
    void initializeInputPointCloudSubscriber();
    void initializeOutputPointCloudPublisher();
    void initializeParameters();

    std::string m_target_frame;

    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_input_pointcloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_output_pointcloud_pub;

  }; // class PointCloudTransformerNode
} // namespace LTM

#endif // LTM_POINTCLOUD_TRANSFORMER__POINTCLOUD_TRANSFORMER_NODE_HPP_

// End of file: ltm_pointcloud_transformer/include/pointcloud_transformer_node.hpp