/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 13-08-2024.
 */

#ifndef LTM_GO2_DRIVER__CONTROLLER_PROCESSING_HPP_
#define LTM_GO2_DRIVER__CONTROLLER_PROCESSING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/empty.hpp>
#include <unitree_go/msg/wireless_controller.hpp>
#include "unitree_api/msg/request.hpp"

#include "nlohmann/json.hpp"

#include <string>

#define CONTROLLER_PROCESSING_NODE_NAME "wireless_controller_processing_node"

#define CMD_VEL_SUB_TOPIC "cmd_vel"
#define CMD_SUB_QUEUE_SIZE 10

#define TOGGLE_SEARCHLIGHT_SUB_TOPIC "toggle_searchlight"
#define TOGGLE_SEARCHLIGHT_QUEUE_SIZE 1

#define CMD_VEL_PUB_TOPIC "api/sport/request"
#define CMD_VEL_PUB_QUEUE_SIZE 1

#define ROBOT_SPORT_API_ID_MOVE 1008

#define MAX_LINEAR_VELOCITY_X 0.80
#define MAX_LINEAR_VELOCITY_Y 0.40
#define MAX_ANGULAR_VELOCITY 0.80

namespace LTM
{
  class ControllerProcessing : public rclcpp::Node
  {
    public:
      ControllerProcessing();
      ~ControllerProcessing();

    private:
      void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

      // void mapLinearVelocity(const geometry_msgs::msg::Vector3& linear_velocity);
      // void mapAngularVelocity(const double& angular_velocity);

      void initializeROS();

      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;
      rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr m_request_pub;

  }; // class ControllerProcessing
} // namespace LTM

#endif // LTM_GO2_DRIVER__CONTROLLER_PROCESSING_HPP_

// End of file: ros2_ws/src/ltm_go2/ltm_go2_driver/include/ltm_go2_driver/controller_processing.hpp