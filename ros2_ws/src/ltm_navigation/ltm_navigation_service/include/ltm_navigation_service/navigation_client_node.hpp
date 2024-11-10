/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 2024-11-08.
 */

#ifndef LTM_NAVIGATION_SERVICE__NAVIGATION_CLIENT_NODE_HPP_
#define LTM_NAVIGATION_SERVICE__NAVIGATION_CLIENT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ltm_shared_msgs/srv/navigate_to_pose.hpp>

#include <string>
#include <vector>
#include <chrono>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace lTM
{
  class NavigationClientNode : public rclcpp::Node
  {
    public:
      NavigationClientNode();
      ~NavigationClientNode();

      void setExecutor(const rclcpp::executors::MultiThreadedExecutor::SharedPtr executor);

    private:
      void goalPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr goal_pose_2d);
      void navigateToPose(const geometry_msgs::msg::Pose& goal_pose_3d);

      void initializeGoalPoseSubscriber();
      void initializeNavigationToPoseClient();

      rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr m_goal_pose_subscriber;
      rclcpp::Client<ltm_shared_msgs::srv::NavigateToPose>::SharedPtr m_navigation_to_pose_client;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr m_executor;

  };  // class NavigationClientNode
} // namespace lTM

#endif  // LTM_NAVIGATION_SERVICE__NAVIGATION_CLIENT_NODE_HPP_
