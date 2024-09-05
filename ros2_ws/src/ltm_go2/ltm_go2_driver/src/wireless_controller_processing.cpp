/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 13-08-2024.
 */

#include <ltm_go2_driver/wireless_controller_processing.hpp>

#include <boost/algorithm/clamp.hpp>

using namespace LTM;

WirelessControllerProcessing::WirelessControllerProcessing() : Node(WIRELESS_CONTROLLER_PROCESSING_NODE_NAME)
{
  initializeROS();
  initializeWirelessControllerMsg();
  RCLCPP_INFO(this->get_logger(), "WirelessControllerProcessing node has been initialized.");
}

WirelessControllerProcessing::~WirelessControllerProcessing()
{
  m_cmd_vel_sub.reset();
  m_toggle_searchlight_sub.reset();
  m_wireless_controller_pub.reset();
  m_wireless_controller_msg.reset();
  RCLCPP_WARN(this->get_logger(), "WirelessControllerProcessing node has been destroyed.");
}

void WirelessControllerProcessing::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  mapLinearVelocity(msg->linear);
  mapAngularVelocity(msg->angular.z);
  publishWirelessController();
}

void WirelessControllerProcessing::toggleSearchlightCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void) msg;
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Received toggle searchlight message.");
  setSearchlightKeys();
  publishWirelessController();
  resetKeys();
}

void WirelessControllerProcessing::publishWirelessController() const
{
  m_wireless_controller_pub->publish(*m_wireless_controller_msg);
}

void WirelessControllerProcessing::mapLinearVelocity(const geometry_msgs::msg::Vector3& linear_velocity)
{
  m_wireless_controller_msg->ly = boost::algorithm::clamp(linear_velocity.x, -MAX_LINEAR_VELOCITY_X, MAX_LINEAR_VELOCITY_X);
  m_wireless_controller_msg->lx = boost::algorithm::clamp(-linear_velocity.y, -MAX_LINEAR_VELOCITY_Y, MAX_LINEAR_VELOCITY_Y);
}

void WirelessControllerProcessing::mapAngularVelocity(const double& angular_velocity)
{
  m_wireless_controller_msg->rx = boost::algorithm::clamp(-angular_velocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
}

void WirelessControllerProcessing::setKey(uint16_t key)
{
  m_wireless_controller_msg->keys = key;
}

void WirelessControllerProcessing::setSearchlightKeys()
{
  setKey(KEYS_SEARCHLIGHT);
}

void WirelessControllerProcessing::setSitDownKeys()
{
  setKey(KEYS_SIT_DOWN);
}

void WirelessControllerProcessing::resetKeys()
{
  setKey(KEYS_EMPTY);
}

void WirelessControllerProcessing::initializeROS()
{
  m_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    CMD_VEL_SUB_TOPIC, CMD_SUB_QUEUE_SIZE,
    std::bind(&WirelessControllerProcessing::cmdVelCallback, this, std::placeholders::_1)
  );

  m_toggle_searchlight_sub = this->create_subscription<std_msgs::msg::Empty>(
    TOGGLE_SEARCHLIGHT_SUB_TOPIC, TOGGLE_SEARCHLIGHT_QUEUE_SIZE,
    std::bind(&WirelessControllerProcessing::toggleSearchlightCallback, this, std::placeholders::_1)
  );

  m_wireless_controller_pub = this->create_publisher<unitree_go::msg::WirelessController>(
    CMD_VEL_PUB_TOPIC, CMD_VEL_PUB_QUEUE_SIZE);
}

void WirelessControllerProcessing::initializeWirelessControllerMsg()
{
    m_wireless_controller_msg = std::make_shared<unitree_go::msg::WirelessController>();
    m_wireless_controller_msg->lx = 0.0;
    m_wireless_controller_msg->ly = 0.0;
    m_wireless_controller_msg->rx = 0.0;
    m_wireless_controller_msg->ry = 0.0;
    m_wireless_controller_msg->keys = 0;
}

// End of file: ros2_ws/src/ltm_go2/ltm_go2_driver/src/cmd_vel_processing.cpp