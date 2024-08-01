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
  class HardwareInterfaceNode : public rclcpp::Node
  {
  public:
    HardwareInterfaceNode();
    ~HardwareInterfaceNode();

  private:
    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
    void publishJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

    void initializeJointStateMsg();

    std::vector<std::string> extractJointNames();

    sensor_msgs::msg::JointState::SharedPtr m_joint_state_msg;

    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr m_low_state_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;

  }; // class HardwareInterfaceNode
} // namespace LTM

#endif // LTM_HARDWARE_INTERFACE__HARDWARE_INTERFACE_NODE_HPP_