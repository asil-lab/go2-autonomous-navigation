/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 09-09-2024.
 */

#ifndef LTM_NAVIGATION_SERVICE__NAVIGATION_SERVICE_NODE_HPP_
#define LTM_NAVIGATION_SERVICE__NAVIGATION_SERVICE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ltm_shared_msgs/srv/navigate_to_pose.hpp>

#include <string>
#include <vector>

namespace lTM
{
  class NavigationServiceNode : public rclcpp::Node
  {
    public:
      NavigationServiceNode();
      ~NavigationServiceNode();

    private:
      void navigateToPoseCallback(
        const std::shared_ptr<ltm_shared_msgs::srv::NavigateToPose::Request> request,
        std::shared_ptr<ltm_shared_msgs::srv::NavigateToPose::Response> response);

      void initializeService();
      void initializeROS2Topics();
      void initializeTFListener();

      std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
      std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_publisher;
      rclcpp::Service<ltm_shared_msgs::srv::NavigateToPose>::SharedPtr m_navigation_to_pose_service;

  }; // class NavigationServiceNode
} // namespace lTM

#endif // LTM_NAVIGATION_SERVICE__NAVIGATION_SERVICE_NODE_HPP_

// End of file: ltm_navigation_service/include/ltm_navigation_service/navigation_service_node.hpp