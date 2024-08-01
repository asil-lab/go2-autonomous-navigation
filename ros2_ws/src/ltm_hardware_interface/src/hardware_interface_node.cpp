/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 01-08-2024.
 */

#include <ltm_hardware_interface/hardware_interface_node.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

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
  // Update the joint state message
  m_joint_state_msg->header.stamp = this->now();
  // m_joint_state_msg->position = std::vector<double>(
  //   msg->foot_position_body.begin(), msg->foot_position_body.end());
  // m_joint_state_msg->velocity = std::vector<double>(
  //   msg->foot_speed_body.begin(), msg->foot_speed_body.end());

  // m_joint_state_msg->position[0] = static_cast<double>(msg->motor_state[3]);
  // m_joint_state_msg->position[1] = static_cast<double>(msg->motor_state[4]);
  // m_joint_state_msg->position[2] = static_cast<double>(msg->motor_state[5]);
  // m_joint_state_msg->position[3] = static_cast<double>(msg->motor_state[0]);
  // m_joint_state_msg->position[4] = static_cast<double>(msg->motor_state[1]);
  // m_joint_state_msg->position[5] = static_cast<double>(msg->motor_state[2]);
  // m_joint_state_msg->position[6] = static_cast<double>(msg->motor_state[9]);
  // m_joint_state_msg->position[7] = static_cast<double>(msg->motor_state[10]);
  // m_joint_state_msg->position[8] = static_cast<double>(msg->motor_state[11]);
  // m_joint_state_msg->position[9] = static_cast<double>(msg->motor_state[6]);
  // m_joint_state_msg->position[10] = static_cast<double>(msg->motor_state[7]);
  // m_joint_state_msg->position[11] = static_cast<double>(msg->motor_state[8]);
  m_joint_state_msg->position[0] = msg->motor_state[3].q;
  m_joint_state_msg->position[1] = msg->motor_state[4].q;
  m_joint_state_msg->position[2] = msg->motor_state[5].q;
  m_joint_state_msg->position[3] = msg->motor_state[0].q;
  m_joint_state_msg->position[4] = msg->motor_state[1].q;
  m_joint_state_msg->position[5] = msg->motor_state[2].q;
  m_joint_state_msg->position[6] = msg->motor_state[9].q;
  m_joint_state_msg->position[7] = msg->motor_state[10].q;
  m_joint_state_msg->position[8] = msg->motor_state[11].q;
  m_joint_state_msg->position[9] = msg->motor_state[6].q;
  m_joint_state_msg->position[10] = msg->motor_state[7].q;
  m_joint_state_msg->position[11] = msg->motor_state[8].q;

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
  m_joint_state_msg->name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
  // m_joint_state_msg->name = extractJointNames();

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

std::vector<std::string> HardwareInterfaceNode::extractJointNames()
{
  // Extract the joint names from the URDF file
  std::string urdf_file = 
    ament_index_cpp::get_package_share_directory("ltm_go2_description") + "/urdf/go2_description.urdf";
  urdf::Model model;
  if (!model.initFile(urdf_file))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF file: %s", urdf_file.c_str());
    return {};
  }

  std::vector<std::string> joint_names;
  for (auto const& joint : model.joints_)
  {
    if (joint.second->type != urdf::Joint::REVOLUTE)
    {
      continue;
    }
    joint_names.push_back(joint.first);
  }

  return joint_names;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}