/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 04-08-2024.
 */

#include <ltm_hardware_interface/joint_state_processing.hpp>

using namespace LTM;

JointStateProcessing::JointStateProcessing()
{
  initializeJointStateMsg();
  initializeJointIdx();
}

JointStateProcessing::~JointStateProcessing()
{
  m_joint_state_msg.reset();
  m_joint_idx.clear();
}

void JointStateProcessing::updateJointStateMsg(
  const std::array<unitree_go::msg::MotorState, MOTOR_SIZE>& motor_state)
{
  m_joint_state_msg->header.stamp = rclcpp::Clock().now();
  for (unsigned long int i = 0; i < m_joint_idx.size(); i++)
  {
    m_joint_state_msg->position[i] = motor_state[m_joint_idx[i]].q;
    m_joint_state_msg->velocity[i] = motor_state[m_joint_idx[i]].dq;
    m_joint_state_msg->effort[i] = motor_state[m_joint_idx[i]].tau_est;
  }
}

sensor_msgs::msg::JointState::SharedPtr JointStateProcessing::getJointStateMsg() const
{
  return m_joint_state_msg;
}

void JointStateProcessing::initializeJointStateMsg()
{
  // Initialize the joint state message
  m_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

  // Set the joint state message header
  m_joint_state_msg->header.frame_id = JOINT_STATE_FRAME_ID;

  // Set the joint state message names
  m_joint_state_msg->name = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", 
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

  // Set zero values for the joint state message
  m_joint_state_msg->position = initializeZeroVector(m_joint_state_msg->name.size());
  m_joint_state_msg->velocity = initializeZeroVector(m_joint_state_msg->name.size());
  m_joint_state_msg->effort = initializeZeroVector(m_joint_state_msg->name.size());
}

void JointStateProcessing::initializeJointIdx()
{
  m_joint_idx = {
    JOINT_FL_HIP_IDX, JOINT_FL_THIGH_IDX, JOINT_FL_CALF_IDX,
    JOINT_FR_HIP_IDX, JOINT_FR_THIGH_IDX, JOINT_FR_CALF_IDX,
    JOINT_RL_HIP_IDX, JOINT_RL_THIGH_IDX, JOINT_RL_CALF_IDX,
    JOINT_RR_HIP_IDX, JOINT_RR_THIGH_IDX, JOINT_RR_CALF_IDX};
}

std::vector<double> JointStateProcessing::initializeZeroVector(const unsigned long int size) const
{
  return std::vector<double>(size, 0.0);
}

// End of file: joint_state_processing.cpp