/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 2024-11-08.
 */

#include "ltm_navigation_service/navigation_client_node.hpp"

#include <memory>
#include <math.h>

using namespace lTM;

NavigationClientNode::NavigationClientNode()
: Node("navigation_client_node")
{
  initializeNavigationToPoseClient();
  initializeGoalPoseSubscriber();
  RCLCPP_INFO(this->get_logger(), "Navigation Client Node has been initialized");
}

NavigationClientNode::~NavigationClientNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down NavigationClientNode");
}

void NavigationClientNode::goalPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr goal_pose_2d)
{
  RCLCPP_INFO(this->get_logger(), "Goal pose received with position (%f, %f) and orientation %f", 
    goal_pose_2d->x, goal_pose_2d->y, goal_pose_2d->theta);

  // Convert yaw angle to quaternion
  Eigen::Quaterniond q_goal(Eigen::AngleAxisd(goal_pose_2d->theta, Eigen::Vector3d::UnitZ()));

  // Move the robot to the goal pose
  geometry_msgs::msg::Pose goal_pose_3d;
  goal_pose_3d.position.x = goal_pose_2d->x;
  goal_pose_3d.position.y = goal_pose_2d->y;
  goal_pose_3d.position.z = 0.0;
  goal_pose_3d.orientation.x = q_goal.x();
  goal_pose_3d.orientation.y = q_goal.y();
  goal_pose_3d.orientation.z = q_goal.z();
  goal_pose_3d.orientation.w = q_goal.w();

  RCLCPP_INFO(this->get_logger(), "Navigating to pose...");
  navigateToPose(goal_pose_3d);
}

void NavigationClientNode::navigateToPose(const geometry_msgs::msg::Pose &goal_pose_3d)
{
  auto request = std::make_shared<ltm_shared_msgs::srv::NavigateToPose::Request>();
  request->goal.header.frame_id = "map";
  request->goal.header.stamp = this->now();
  request->goal.pose = goal_pose_3d;
  
  auto result = m_navigation_to_pose_client->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "Navigation to pose successful");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Navigation to pose failed");
  }
}

void NavigationClientNode::initializeGoalPoseSubscriber()
{
  declare_parameter("goal_pose_topic_name", "desired_pose");
  declare_parameter("goal_pose_topic_queue_size", 10);

  m_goal_pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose2D>(
    this->get_parameter("goal_pose_topic_name").as_string(),
    this->get_parameter("goal_pose_topic_queue_size").as_int(),
    std::bind(&NavigationClientNode::goalPoseCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Initialized goal pose subscriber");
}

void NavigationClientNode::initializeNavigationToPoseClient()
{
  declare_parameter("navigate_to_pose_service_name", "send_robot");

  m_navigation_to_pose_client = this->create_client<ltm_shared_msgs::srv::NavigateToPose>(
    this->get_parameter("navigate_to_pose_service_name").as_string());
  RCLCPP_INFO(this->get_logger(), "Initialized navigation to pose client");
}

void NavigationClientNode::setExecutor(const rclcpp::executors::MultiThreadedExecutor::SharedPtr executor)
{
  m_executor = executor;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationClientNode>();
  // auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  // node->setExecutor(executor);
  // executor->add_node(node);
  // executor->spin();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}