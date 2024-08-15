/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 13-08-2024.
 */

#ifndef LTM_GO2_DRIVER__CMD_VEL_PROCESSING_HPP_
#define LTM_GO2_DRIVER__CMD_VEL_PROCESSING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <unitree_go/msg/wireless_controller.hpp>

#include <string>

#define CMD_VEL_PROCESSING_NODE_NAME "cmd_vel_processing_node"

#define CMD_VEL_PROCESSING_SUB_TOPIC "cmd_vel"
#define CMD_VEL_PROCESSING_SUB_QUEUE_SIZE 10

#define CMD_VEL_PUBLISH_TOPIC "wirelesscontroller"
#define CMD_VEL_PUBLISH_QUEUE_SIZE 10

#define MAX_LINEAR_VELOCITY_X 0.05
#define MAX_LINEAR_VELOCITY_Y 0.10
#define MAX_ANGULAR_VELOCITY 0.05

namespace LTM
{
  class CmdVelProcessing : public rclcpp::Node
  {
    public:
      CmdVelProcessing();
      ~CmdVelProcessing();

    private:
      void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
      void publishWirelessController(const unitree_go::msg::WirelessController::SharedPtr msg) const;

      void mapLinearVelocity(const geometry_msgs::msg::Vector3& linear_velocity);
      void mapAngularVelocity(const double& angular_velocity);

      void initializeROS();
      void initializeWirelessControllerMsg();

      unitree_go::msg::WirelessController::SharedPtr m_wireless_controller_msg;

      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;
      rclcpp::Publisher<unitree_go::msg::WirelessController>::SharedPtr m_wireless_controller_pub;

  }; // class CmdVelProcessing
} // namespace LTM

#endif // LTM_GO2_DRIVER__CMD_VEL_PROCESSING_HPP_

// End of file: ros2_ws/src/ltm_go2/ltm_go2_driver/include/ltm_go2_driver/cmd_vel_processing.hpp