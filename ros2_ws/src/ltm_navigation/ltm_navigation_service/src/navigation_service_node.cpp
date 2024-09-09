/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 01-08-2024.
 */

#include "ltm_navigation_service/navigation_service_node.hpp"

#include <memory>

using namespace lTM;

NavigationServiceNode::NavigationServiceNode()
: Node("navigation_service_node")
{
  initializeService();
  initializeROS2Topics();
  RCLCPP_INFO(this->get_logger(), "Navigation Service Node has been initialized");
}

NavigationServiceNode::~NavigationServiceNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down NavigationServiceNode");
}

void NavigationServiceNode::navigateToPoseCallback(
  const std::shared_ptr<ltm_shared_msgs::srv::NavigateToPose::Request> request,
  std::shared_ptr<ltm_shared_msgs::srv::NavigateToPose::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received navigation service request");
  response->success = true;
}

void NavigationServiceNode::initializeService()
{
  m_navigation_to_pose_service = this->create_service<ltm_shared_msgs::srv::NavigateToPose>(
    "navigate_to_pose", std::bind(&NavigationServiceNode::navigateToPoseCallback, 
      this, std::placeholders::_1, std::placeholders::_2));
}

void NavigationServiceNode::initializeROS2Topics()
{
  m_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
  RCLCPP_INFO(this->get_logger(), "Initialized ROS2 topics");
}

void NavigationServiceNode::initializeTFListener()
{
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer, this, false);
  RCLCPP_INFO(this->get_logger(), "Initialized TF2 listener");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationServiceNode>());
  rclcpp::shutdown();
  return 0;
}

// End of file: ltm_navigation_service/src/navigation_service_node.cpp