/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 12-09-2024.
 */

#ifndef LTM_GO2_CAMERA__GO2_CAMERA_NODE_HPP_
#define LTM_GO2_CAMERA__GO2_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <string>

namespace LTM
{
  class Go2CameraNode : public rclcpp::Node
  {
    public:
      Go2CameraNode();
      ~Go2CameraNode();

    private:
      void timerCallback();
      void publishImage(const sensor_msgs::msg::Image::SharedPtr msg);

      rclcpp::TimerBase::SharedPtr m_timer;
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_pub;

  }; // class Go2CameraNode
} // namespace LTM

#endif  // LTM_GO2_CAMERA__GO2_CAMERA_NODE_HPP_

// Path: ros2_ws/src/ltm_go2/ltm_go2_camera/src/go2_camera_node.cpp