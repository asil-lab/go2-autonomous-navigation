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

namespace LTM
{
  class JointStateProcessing
  {
    public:
      JointStateProcessing();
      ~JointStateProcessing();

      void updateJointStateMsg(
        const std::array<unitree_go::msg::MotorState, MOTOR_SIZE>& motor_state);

      sensor_msgs::msg::JointState::SharedPtr getJointStateMsg() const;

    private:
      void initializeJointStateMsg();
      void initializeJointIdx();
      std::vector<double> initializeZeroVector(const unsigned long int size) const;

      sensor_msgs::msg::JointState::SharedPtr m_joint_state_msg;
      std::vector<int> m_joint_idx;

  };  // class JointStateProcessing
} // namespace LTM

#endif  // LTM_HARDWARE_INTERFACE__JOINT_STATE_PROCESSING_HPP_

// End of file: joint_state_processing.hpp