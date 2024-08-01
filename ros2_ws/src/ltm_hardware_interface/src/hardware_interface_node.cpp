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
  m_low_state_sub = this->create_subscription<unitree_go::msg::LowState>(
    "lowstate", 10, std::bind(&HardwareInterfaceNode::lowStateCallback, this, std::placeholders::_1));

  // Create a publisher for the joint state
  m_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  RCLCPP_INFO(this->get_logger(), "LTM Hardware Interface Node initialized.");
}

HardwareInterfaceNode::~HardwareInterfaceNode()
{
  // Reset ROS subscriptions and publishers
  m_low_state_sub.reset();
  m_joint_state_pub.reset();

  // Reset msg pointers
  m_joint_state_msg.reset();

  RCLCPP_WARN(this->get_logger(), "LTM Hardware Interface Node shutting down.");
}

void HardwareInterfaceNode::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
  updateJointStateMsg(msg->motor_state);
  publishJointState();
}

void HardwareInterfaceNode::updateJointStateMsg(const std::array<unitree_go::msg::MotorState, MOTOR_SIZE>& motor_state)
{
  // Update the joint state message
  m_joint_state_msg->header.stamp = this->now();

  // Update the joint state values
  for (unsigned long int i = 0; i < m_joint_idx.size(); i++)
  {
    m_joint_state_msg->position[i] = motor_state[m_joint_idx[i]].q;
    m_joint_state_msg->velocity[i] = motor_state[m_joint_idx[i]].dq;
    m_joint_state_msg->effort[i] = motor_state[m_joint_idx[i]].tau_est;
  }
}

void HardwareInterfaceNode::publishJointState()
{
  // Publish the joint state message
  m_joint_state_pub->publish(*m_joint_state_msg);
}

void HardwareInterfaceNode::initializeJointStateMsg()
{
  // Initialize the joint state message
  m_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

  // Set the joint state message header
  m_joint_state_msg->header.stamp = this->now();
  m_joint_state_msg->header.frame_id = "base_link";

  // Set the joint state message names
  m_joint_state_msg->name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

  // Set zero values for the joint state message
  m_joint_state_msg->position = std::vector<double>(12, 0.0);
  m_joint_state_msg->velocity = std::vector<double>(12, 0.0);
  m_joint_state_msg->effort = std::vector<double>(12, 0.0);

  // Set the joint state message indices
  m_joint_idx = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8}; // TODO: Parameterize this
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}