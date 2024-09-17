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
  initializeRosTopic();
  initializeTfListener();
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
  RCLCPP_INFO(this->get_logger(), "Goal pose requested with position (%f, %f, %f) and orientation (%f, %f, %f, %f)", 
    request->goal.pose.position.x, request->goal.pose.position.y, request->goal.pose.position.z, 
    request->goal.pose.orientation.x, request->goal.pose.orientation.y,
    request->goal.pose.orientation.z, request->goal.pose.orientation.w);

  publishGoalPose(request->goal);
  Eigen::VectorXd goal_pose_vector = convertPoseToEigen(request->goal.pose);

  rclcpp::Time start_time = rclcpp::Time(request->goal.header.stamp);
  rclcpp::Time update_time = this->get_clock()->now();

  while (this->get_clock()->now() - start_time < rclcpp::Duration::from_seconds(m_navigate_to_pose_timeout))
  {
    // Check if the service has been interrupted
    if (!rclcpp::ok())
    {
      RCLCPP_WARN(this->get_logger(), "Navigation service interrupted");
      response->success = false;
      return;
    }

    // Skip the update if the update period has not been reached
    if (this->get_clock()->now() - update_time < rclcpp::Duration::from_seconds(m_navigate_to_pose_update_period))
      continue;

    // Get the current robot pose and calculate the error
    Eigen::VectorXd current_pose_vector = convertPoseToEigen(getCurrentRobotPose().pose);
    double position_error = (goal_pose_vector.block<POSE_EIGEN_VECTOR_POSITION_SIZE, 1>(
      POSE_EIGEN_VECTOR_POSITION_OFFSET, 0) - current_pose_vector.block<POSE_EIGEN_VECTOR_POSITION_SIZE, 1>(
        POSE_EIGEN_VECTOR_POSITION_OFFSET, 0)).norm();
    
    double orientation_error = (goal_pose_vector.block<POSE_EIGEN_VECTOR_ORIENTATION_SIZE, 1>(
      POSE_EIGEN_VECTOR_ORIENTATION_OFFSET, 0) - current_pose_vector.block<POSE_EIGEN_VECTOR_ORIENTATION_SIZE, 1>(
        POSE_EIGEN_VECTOR_ORIENTATION_OFFSET, 0)).norm();

    // Interrupt the service if the timeout has been reached
    if (position_error < m_navigate_to_pose_position_tolerance && orientation_error < m_navigate_to_pose_orientation_tolerance)
    {
      RCLCPP_INFO(this->get_logger(), "Reached goal pose");
      response->success = true;
      response->time_elapsed.sec = (this->get_clock()->now() - start_time).seconds();
      response->time_elapsed.nanosec = (this->get_clock()->now() - start_time).nanoseconds();
      return;
    }

    update_time = this->get_clock()->now();
  }

  // If the goal pose has not been reached, return a failure
  RCLCPP_WARN(this->get_logger(), "Failed to reach goal pose");
  response->success = false;
}

void NavigationServiceNode::publishGoalPose(const geometry_msgs::msg::PoseStamped& goal_pose) const
{
  m_pose_publisher->publish(goal_pose);
}

geometry_msgs::msg::PoseStamped NavigationServiceNode::getCurrentRobotPose()
{
  geometry_msgs::msg::PoseStamped current_pose;
  try
  {
    geometry_msgs::msg::TransformStamped transform = m_tf_buffer->lookupTransform(
      m_robot_target_frame_name, m_robot_source_frame_name, tf2::TimePointZero);
    current_pose.header.frame_id = m_robot_target_frame_name;
    current_pose.header.stamp = this->get_clock()->now();
    current_pose.pose.position.x = transform.transform.translation.x;
    current_pose.pose.position.y = transform.transform.translation.y;
    current_pose.pose.position.z = transform.transform.translation.z;
    current_pose.pose.orientation.x = transform.transform.rotation.x;
    current_pose.pose.orientation.y = transform.transform.rotation.y;
    current_pose.pose.orientation.z = transform.transform.rotation.z;
    current_pose.pose.orientation.w = transform.transform.rotation.w;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Error getting current robot pose: %s", ex.what());
  }
  return current_pose;
}

Eigen::VectorXd NavigationServiceNode::convertPoseToEigen(geometry_msgs::msg::Pose pose)
{
  Eigen::VectorXd pose_vector(POSE_EIGEN_VECTOR_SIZE);
  pose_vector << pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w;
  return pose_vector;
}

void NavigationServiceNode::initializeService()
{
  declare_parameter("navigate_to_pose_service_name", "navigate_to_pose");
  declare_parameter("navigate_to_pose_timeout", 30.0);
  declare_parameter("navigate_to_pose_update_rate", 1.0);
  declare_parameter("navigate_to_pose_position_tolerance", 0.1);
  declare_parameter("navigate_to_pose_orientation_tolerance", 0.1);

  m_navigation_to_pose_service = this->create_service<ltm_shared_msgs::srv::NavigateToPose>(
    this->get_parameter("navigate_to_pose_service_name").as_string(),
    std::bind(&NavigationServiceNode::navigateToPoseCallback, 
      this, std::placeholders::_1, std::placeholders::_2));
  m_navigate_to_pose_timeout = this->get_parameter("navigate_to_pose_timeout").as_double();
  m_navigate_to_pose_update_period = 1.0 / this->get_parameter("navigate_to_pose_update_rate").as_double();
  m_navigate_to_pose_position_tolerance = this->get_parameter("navigate_to_pose_position_tolerance").as_double();
  m_navigate_to_pose_orientation_tolerance = this->get_parameter("navigate_to_pose_orientation_tolerance").as_double();
}

void NavigationServiceNode::initializeRosTopic()
{
  declare_parameter("goal_pose_topic_name", "goal_pose");
  declare_parameter("goal_pose_topic_queue_size", 10);

  m_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    this->get_parameter("goal_pose_topic_name").as_string(),
    this->get_parameter("goal_pose_topic_queue_size").as_int());
  RCLCPP_INFO(this->get_logger(), "Initialized ROS2 topics");
}

void NavigationServiceNode::initializeTfListener()
{
  declare_parameter("robot_source_frame_name", "base");
  declare_parameter("robot_target_frame_name", "map");

  m_robot_source_frame_name = this->get_parameter("robot_source_frame_name").as_string();
  m_robot_target_frame_name = this->get_parameter("robot_target_frame_name").as_string();

  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer, this, false);
  RCLCPP_INFO(this->get_logger(), "Initialized TF2 listener");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr navigation_service_node = std::make_shared<NavigationServiceNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(navigation_service_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

// End of file: ltm_navigation_service/src/navigation_service_node.cpp