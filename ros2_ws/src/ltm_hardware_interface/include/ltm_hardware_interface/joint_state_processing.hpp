/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 04-08-2024.
 */

#ifndef LTM_HARDWARE_INTERFACE__JOINT_STATE_PROCESSING_HPP_
#define LTM_HARDWARE_INTERFACE__JOINT_STATE_PROCESSING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_go/msg/low_state.hpp>

#include <string>
#include <vector>

#define ROS_NODE_NAME "joint_state_processing_node"
#define ROS_SUB_TOPIC "lowstate"
#define ROS_SUB_QUEUE_SIZE 10
#define ROS_PUB_TOPIC "joint_states"
#define ROS_PUB_QUEUE_SIZE 10

#define MOTOR_SIZE 20
#define JOINT_STATE_FRAME_ID "joint_link"

#define JOINT_FR_HIP_IDX      0
#define JOINT_FR_THIGH_IDX    1
#define JOINT_FR_CALF_IDX     2
#define JOINT_FL_HIP_IDX      3
#define JOINT_FL_THIGH_IDX    4
#define JOINT_FL_CALF_IDX     5
#define JOINT_RR_HIP_IDX      6
#define JOINT_RR_THIGH_IDX    7
#define JOINT_RR_CALF_IDX     8
#define JOINT_RL_HIP_IDX      9
#define JOINT_RL_THIGH_IDX    10
#define JOINT_RL_CALF_IDX     11

#define JOINT_FR_HIP_NAME     "FR_hip_joint"
#define JOINT_FR_THIGH_NAME   "FR_thigh_joint"
#define JOINT_FR_CALF_NAME    "FR_calf_joint"
#define JOINT_FL_HIP_NAME     "FL_hip_joint"
#define JOINT_FL_THIGH_NAME   "FL_thigh_joint"
#define JOINT_FL_CALF_NAME    "FL_calf_joint"
#define JOINT_RR_HIP_NAME     "RR_hip_joint"
#define JOINT_RR_THIGH_NAME   "RR_thigh_joint"
#define JOINT_RR_CALF_NAME    "RR_calf_joint"
#define JOINT_RL_HIP_NAME     "RL_hip_joint"
#define JOINT_RL_THIGH_NAME   "RL_thigh_joint"
#define JOINT_RL_CALF_NAME    "RL_calf_joint"

namespace LTM
{
  class JointStateProcessing : public rclcpp::Node
  {
    public:
      JointStateProcessing();
      ~JointStateProcessing();

    sensor_msgs::msg::JointState::SharedPtr getJointStateMsg() const;

    private:
      void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
      void publishJointStateMsg() const;

      void updateJointStateMsg(
        const std::array<unitree_go::msg::MotorState, MOTOR_SIZE>& motor_state);

      void initializeROS();
      void initializeJointStateMsg();
      void initializeJointIdx();
      std::vector<double> initializeZeroVector(const unsigned long int size) const;

      sensor_msgs::msg::JointState::SharedPtr m_joint_state_msg;
      std::vector<int> m_joint_idx;

      rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr m_low_state_sub;
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;

  };  // class JointStateProcessing
} // namespace LTM

#endif  // LTM_HARDWARE_INTERFACE__JOINT_STATE_PROCESSING_HPP_

// End of file: joint_state_processing.hpp