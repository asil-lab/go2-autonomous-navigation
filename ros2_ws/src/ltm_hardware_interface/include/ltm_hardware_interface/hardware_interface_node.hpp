/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 01-08-2024.
 */

#ifndef LTM_HARDWARE_INTERFACE__HARDWARE_INTERFACE_NODE_HPP_
#define LTM_HARDWARE_INTERFACE__HARDWARE_INTERFACE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <unitree_go/msg/low_state.hpp>

#include <vector>
#include <string>

namespace LTM
{
  #define MOTOR_SIZE 20

  class HardwareInterfaceNode : public rclcpp::Node
  {
  public:
    HardwareInterfaceNode();
    ~HardwareInterfaceNode();

  private:
    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
    void updateJointStateMsg(const std::array<unitree_go::msg::MotorState, MOTOR_SIZE>& motor_state);
    void publishJointState();

    void initializeJointStateMsg();

    sensor_msgs::msg::JointState::SharedPtr m_joint_state_msg;
    std::vector<int> m_joint_idx;

    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr m_low_state_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;

  }; // class HardwareInterfaceNode
} // namespace LTM

#endif // LTM_HARDWARE_INTERFACE__HARDWARE_INTERFACE_NODE_HPP_