/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 15-01-2025.
 */

#include <go2_controller_node/go2_controller_node.hpp>

Go2ControllerNode::Go2ControllerNode() : Node("go2_controller_node")
{
  initializeRobot();
  initializeCmdVelSubscriber();
  RCLCPP_INFO(this->get_logger(), "Go2ControllerNode has been initialized.");
}

Go2ControllerNode::~Go2ControllerNode()
{
  m_sport_client.StopMove();
  RCLCPP_INFO(this->get_logger(), "Go2ControllerNode has been terminated.");
}

void Go2ControllerNode::initializeRobot()
{
  m_sport_client.SetTimeout(10.0f);
  m_sport_client.Init();
}

void Go2ControllerNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  m_sport_client.SetVelocity(msg->linear.x, msg->linear.y, msg->angular.z);
}

void Go2ControllerNode::initializeCmdVelSubscriber()
{
  m_cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&Go2ControllerNode::cmdVelCallback, this, std::placeholders::_1));
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
