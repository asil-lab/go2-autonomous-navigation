/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 13-08-2024.
 */

#include <ltm_go2_driver/controller_processing.hpp>

#include <boost/algorithm/clamp.hpp>

using namespace LTM;

ControllerProcessing::ControllerProcessing() : Node(CONTROLLER_PROCESSING_NODE_NAME)
{
  initializeROS();
  RCLCPP_INFO(this->get_logger(), "ControllerProcessing node has been initialized.");
}

ControllerProcessing::~ControllerProcessing()
{
  m_cmd_vel_sub.reset();
  m_request_pub.reset();
  RCLCPP_WARN(this->get_logger(), "ControllerProcessing node has been destroyed.");
}

void ControllerProcessing::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  unitree_api::msg::Request request;
  request.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;

  // mapLinearVelocity(msg->linear);
  // mapAngularVelocity(msg->angular.z);

  nlohmann::json js;
  js["x"] = msg->linear.x;
  js["y"] = msg->linear.y;
  js["z"] = msg->angular.z;
  request.parameter = js.dump();

  m_request_pub->publish(request);
}

// void ControllerProcessing::mapLinearVelocity(const geometry_msgs::msg::Vector3& linear_velocity)
// {
//   m_wireless_controller_msg->ly = boost::algorithm::clamp(linear_velocity.x, -MAX_LINEAR_VELOCITY_X, MAX_LINEAR_VELOCITY_X);
//   m_wireless_controller_msg->lx = boost::algorithm::clamp(-linear_velocity.y, -MAX_LINEAR_VELOCITY_Y, MAX_LINEAR_VELOCITY_Y);
// }

// void ControllerProcessing::mapAngularVelocity(const double& angular_velocity)
// {
//   m_wireless_controller_msg->rx = boost::algorithm::clamp(-angular_velocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
// }

void ControllerProcessing::initializeROS()
{
  m_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    CMD_VEL_SUB_TOPIC, CMD_SUB_QUEUE_SIZE,
    std::bind(&ControllerProcessing::cmdVelCallback, this, std::placeholders::_1)
  );

  m_request_pub = this->create_publisher<unitree_api::msg::Request>(
    CMD_VEL_PUB_TOPIC, CMD_VEL_PUB_QUEUE_SIZE);
}

// End of file: ros2_ws/src/ltm_go2/ltm_go2_driver/src/controller_processing.cpp