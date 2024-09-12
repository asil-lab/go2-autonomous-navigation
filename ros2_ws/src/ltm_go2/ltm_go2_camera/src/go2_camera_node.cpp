/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 12-09-2024.
 */

#include <ltm_go2_camera/go2_camera_node.hpp>

#include <chrono>

using namespace LTM;

Go2CameraNode::Go2CameraNode() : Node("go2_camera_node")
{
  RCLCPP_INFO(this->get_logger(), "Go2CameraNode starting up.");

  m_timer = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&Go2CameraNode::timerCallback, this));

  m_image_pub = this->create_subscription<sensor_msgs::msg::Image>(
    "image", 10, std::bind(&Go2CameraNode::publishImage, this, std::placeholders::_1));
}

Go2CameraNode::~Go2CameraNode()
{
  RCLCPP_WARN(this->get_logger(), "Go2CameraNode shutting down.");
}

void Go2CameraNode::timerCallback()
{
  RCLCPP_INFO(this->get_logger(), "Timer callback triggered.");
}

void Go2CameraNode::publishImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Publishing image.");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2CameraNode>());
  rclcpp::shutdown();
  return 0;
}

// Path: ros2_ws/src/ltm_go2/ltm_go2_camera/src/go2_camera_node.cpp