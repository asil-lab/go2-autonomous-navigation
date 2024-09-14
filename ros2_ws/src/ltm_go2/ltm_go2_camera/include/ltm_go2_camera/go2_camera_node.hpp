/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 12-09-2024.
 */

#ifndef LTM_GO2_CAMERA__GO2_CAMERA_NODE_HPP_
#define LTM_GO2_CAMERA__GO2_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <ltm_shared_msgs/srv/get_image.hpp>

#include <string>

#include <opencv2/opencv.hpp>

namespace LTM
{
  class Go2CameraNode : public rclcpp::Node
  {
    public:
      Go2CameraNode();
      ~Go2CameraNode();

    private:
      void timerCallback();
      void serviceCallback(const std::shared_ptr<ltm_shared_msgs::srv::GetImage::Request> request,
        std::shared_ptr<ltm_shared_msgs::srv::GetImage::Response> response);
      void publishImage(const sensor_msgs::msg::Image::SharedPtr msg) const;

      sensor_msgs::msg::Image::SharedPtr captureImage();

      void initializeCamera();
      void initializeStreamMode();
      void initializeServiceMode();

      void initializeTimer();
      void initializeImagePublisher();
      void initializeImageService();

      std::string m_camera_frame_id;
      std::string m_camera_address;
      cv::VideoCapture m_cap;

      rclcpp::TimerBase::SharedPtr m_timer;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_pub;
      rclcpp::Service<ltm_shared_msgs::srv::GetImage>::SharedPtr m_image_service;

  }; // class Go2CameraNode
} // namespace LTM

#endif  // LTM_GO2_CAMERA__GO2_CAMERA_NODE_HPP_

// Path: ros2_ws/src/ltm_go2/ltm_go2_camera/src/go2_camera_node.cpp