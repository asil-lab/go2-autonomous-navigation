/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 15-01-2025.
 */

#ifndef LTM_GO2_CONTROLLER__GO2_CONTROLLER_NODE_HPP_
#define LTM_GO2_CONTROLLER__GO2_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#include <string>

class Go2ControllerNode : public rclcpp::Node
{
  public:
    Go2ControllerNode();
    ~Go2ControllerNode();

  private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void initializeRobot();
    void initializeCmdVelSubscriber();

    unitree::robot::go2::SportClient m_sport_client;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;

}; // class Go2ControllerNode

#endif  // LTM_GO2_CONTROLLER__GO2_CONTROLLER_NODE_HPP_