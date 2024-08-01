/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 01-08-2024.
 */

#include <ltm_hardware_interface/hardware_interface_node.hpp>

using namespace LTM;

HardwareInterfaceNode::HardwareInterfaceNode()
: Node("hardware_interface_node")
{
  // Initialize the joint state message
  initializeJointStateMsg();

  // Create a subscription to the sport mode state
  m_sport_mode_sub = this->create_subscription<unitree_go::msg::SportModeState>(
    "sportmodestate", 10, std::bind(&HardwareInterfaceNode::sportModeCallback, this, std::placeholders::_1));

  // Create a publisher for the joint state
  m_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  RCLCPP_INFO(this->get_logger(), "LTM Hardware Interface Node initialized.");
}

HardwareInterfaceNode::~HardwareInterfaceNode()
{
  // Reset ROS subscriptions and publishers
  m_sport_mode_sub.reset();
  m_joint_state_pub.reset();

  // Reset msg pointers
  m_joint_state_msg.reset();

  RCLCPP_WARN(this->get_logger(), "LTM Hardware Interface Node shutting down.");
}

void HardwareInterfaceNode::sportModeCallback(const unitree_go::msg::SportModeState::SharedPtr msg)
{
  // Update the joint state message
  m_joint_state_msg->header.stamp = msg->header.stamp;
  m_joint_state_msg->position = std::vector<double>(
    msg->foot_position_body.begin(), msg->foot_position_body.end());
  m_joint_state_msg->velocity = std::vector<double>(
    msg->foot_speed_body.begin(), msg->foot_speed_body.end());

  // Publish the joint state message
  publishJointState(m_joint_state_msg);
}

void HardwareInterfaceNode::publishJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Publish the joint state message
  m_joint_state_pub->publish(*msg);
}

void HardwareInterfaceNode::initializeJointStateMsg()
{
  // Initialize the joint state message
  m_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

  // Set the joint state message header
  m_joint_state_msg->header.stamp = this->now();
  m_joint_state_msg->header.frame_id = "base_link";

  // Set the joint state message names
  m_joint_state_msg->name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
                              "joint7", "joint8", "joint9", "joint10", "joint11", "joint12"};

  // Set the joint state message positions
  m_joint_state_msg->position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Set the joint state message velocities
  m_joint_state_msg->velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Set the joint state message efforts
  m_joint_state_msg->effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}