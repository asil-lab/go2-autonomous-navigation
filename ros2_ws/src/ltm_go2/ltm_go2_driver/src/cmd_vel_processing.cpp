/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 13-08-2024.
 */

#include <ltm_go2_driver/cmd_vel_processing.hpp>

#include <boost/algorithm/clamp.hpp>

using namespace LTM;

CmdVelProcessing::CmdVelProcessing() : Node(CMD_VEL_PROCESSING_NODE_NAME)
{
  initializeROS();
  initializeWirelessControllerMsg();
  RCLCPP_INFO(this->get_logger(), "CmdVelProcessing node has been initialized.");
}

CmdVelProcessing::~CmdVelProcessing()
{
  m_cmd_vel_sub.reset();
  m_wireless_controller_pub.reset();
  m_wireless_controller_msg.reset();
  RCLCPP_WARN(this->get_logger(), "CmdVelProcessing node has been destroyed.");
}

void CmdVelProcessing::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  mapLinearVelocity(msg->linear);
  mapAngularVelocity(msg->angular.z);
  publishWirelessController(m_wireless_controller_msg);
}

void CmdVelProcessing::publishWirelessController(
    const unitree_go::msg::WirelessController::SharedPtr msg) const
{
  m_wireless_controller_pub->publish(*msg);
}

void CmdVelProcessing::mapLinearVelocity(const geometry_msgs::msg::Vector3& linear_velocity)
{
  m_wireless_controller_msg->ly = boost::algorithm::clamp(linear_velocity.x, -MAX_LINEAR_VELOCITY_X, MAX_LINEAR_VELOCITY_X);
  m_wireless_controller_msg->lx = boost::algorithm::clamp(-linear_velocity.y, -MAX_LINEAR_VELOCITY_Y, MAX_LINEAR_VELOCITY_Y);
}

void CmdVelProcessing::mapAngularVelocity(const double& angular_velocity)
{
  m_wireless_controller_msg->rx = boost::algorithm::clamp(-angular_velocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
}

void CmdVelProcessing::initializeROS()
{
  m_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      CMD_VEL_PROCESSING_SUB_TOPIC,
      CMD_VEL_PROCESSING_SUB_QUEUE_SIZE,
      std::bind(&CmdVelProcessing::cmdVelCallback, this, std::placeholders::_1)
  );

  m_wireless_controller_pub = this->create_publisher<unitree_go::msg::WirelessController>(
      CMD_VEL_PUBLISH_TOPIC,
      CMD_VEL_PUBLISH_QUEUE_SIZE
  );
}

void CmdVelProcessing::initializeWirelessControllerMsg()
{
    m_wireless_controller_msg = std::make_shared<unitree_go::msg::WirelessController>();
    m_wireless_controller_msg->lx = 0.0;
    m_wireless_controller_msg->ly = 0.0;
    m_wireless_controller_msg->rx = 0.0;
    m_wireless_controller_msg->ry = 0.0;
    m_wireless_controller_msg->keys = 0;
}

// End of file: ros2_ws/src/ltm_go2/ltm_go2_driver/src/cmd_vel_processing.cpp