/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 13-08-2024.
 */

#ifndef LTM_GO2_DRIVER__WIRELESS_CONTROLLER_PROCESSING_HPP_
#define LTM_GO2_DRIVER__WIRELESS_CONTROLLER_PROCESSING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/empty.hpp>
#include <unitree_go/msg/wireless_controller.hpp>

#include <string>

#define WIRELESS_CONTROLLER_PROCESSING_NODE_NAME "wireless_controller_processing_node"

#define CMD_VEL_SUB_TOPIC "cmd_vel"
#define CMD_SUB_QUEUE_SIZE 10

#define TOGGLE_SEARCHLIGHT_SUB_TOPIC "toggle_searchlight"
#define TOGGLE_SEARCHLIGHT_QUEUE_SIZE 1

#define CMD_VEL_PUB_TOPIC "wirelesscontroller"
#define CMD_VEL_PUB_QUEUE_SIZE 10

#define MAX_LINEAR_VELOCITY_X 0.80
#define MAX_LINEAR_VELOCITY_Y 0.40
#define MAX_ANGULAR_VELOCITY 0.80

#define KEYS_SELECT       0b0000000000001000
#define KEYS_L2           0b0000000000100000
#define KEYS_SEARCHLIGHT  0b0000000000101000
#define KEYS_SIT_DOWN     513
#define KEYS_EMPTY        0b0000000000000000

namespace LTM
{
  class WirelessControllerProcessing : public rclcpp::Node
  {
    public:
      WirelessControllerProcessing();
      ~WirelessControllerProcessing();

    private:
      void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
      void toggleSearchlightCallback(const std_msgs::msg::Empty::SharedPtr msg);
      void publishWirelessController() const;

      void mapLinearVelocity(const geometry_msgs::msg::Vector3& linear_velocity);
      void mapAngularVelocity(const double& angular_velocity);

      void setKey(uint16_t key);
      void setSearchlightKeys();
      void setSitDownKeys();
      void resetKeys();

      void initializeROS();
      void initializeWirelessControllerMsg();

      unitree_go::msg::WirelessController::SharedPtr m_wireless_controller_msg;

      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;
      rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_toggle_searchlight_sub;
      rclcpp::Publisher<unitree_go::msg::WirelessController>::SharedPtr m_wireless_controller_pub;

  }; // class WirelessControllerProcessing
} // namespace LTM

#endif // LTM_GO2_DRIVER__WIRELESS_CONTROLLER_PROCESSING_HPP_

// End of file: ros2_ws/src/ltm_go2/ltm_go2_driver/include/ltm_go2_driver/wireless_controller_processing.hpp