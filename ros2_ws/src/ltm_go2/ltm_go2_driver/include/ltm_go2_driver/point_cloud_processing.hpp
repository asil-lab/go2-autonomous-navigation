/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 13-08-2024.
 */

#ifndef LTM_GO2_DRIVER__POINT_CLOUD_PROCESSING_HPP_
#define LTM_GO2_DRIVER__POINT_CLOUD_PROCESSING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ltm_shared_msgs/srv/get_point_cloud.hpp>

#include <string>

#define POINT_CLOUD_PROCESSING_NODE_NAME "point_cloud_processing_node"

#define POINT_CLOUD_PROCESSING_SUB_TOPIC "utlidar/cloud"
#define POINT_CLOUD_PROCESSING_SUB_QUEUE_SIZE 10

#define POINT_CLOUD_PROCESSING_PUB_TOPIC "point_cloud/raw"
#define POINT_CLOUD_PROCESSING_PUB_QUEUE_SIZE 10

#define POINT_CLOUD_PROCESSING_GET_POINTCLOUD_SERVICE "get_pointcloud"

#define POINT_CLOUD_SUB_FRAME_ID "utlidar"
#define POINT_CLOUD_PUB_FRAME_ID "radar"

namespace LTM
{
  class PointCloudProcessing : public rclcpp::Node
  {
    public:
      PointCloudProcessing();
      ~PointCloudProcessing();

    private:
      void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
      void publishPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
      void serviceCallback(const std::shared_ptr<ltm_shared_msgs::srv::GetPointCloud::Request> request,
        std::shared_ptr<ltm_shared_msgs::srv::GetPointCloud::Response> response);

      void updatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
      void changeFrameId(sensor_msgs::msg::PointCloud2::SharedPtr msg);

      void initializeROS();
      void initializePointcloudMsg();

      sensor_msgs::msg::PointCloud2::SharedPtr m_recent_input_pointcloud_msg;

      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
      rclcpp::Service<ltm_shared_msgs::srv::GetPointCloud>::SharedPtr get_pointcloud_service_;
      
  }; // class PointCloudProcessing
} // namespace LTM

#endif // LTM_GO2_DRIVER__POINT_CLOUD_PROCESSING_HPP_

// End of file: ros2_ws/src/ltm_go2_driver/include/ltm_go2_driver/point_cloud_processing.hpp