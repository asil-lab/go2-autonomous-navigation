/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 12-09-2024.
 */

#include <ltm_go2_camera/go2_camera_node.hpp>

#include <chrono>
#include <stdio.h>

#include <cv_bridge/cv_bridge.h>

using namespace LTM;

Go2CameraNode::Go2CameraNode() : Node("go2_camera_node")
{
  RCLCPP_INFO(this->get_logger(), "Go2CameraNode starting up.");

  m_timer = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&Go2CameraNode::timerCallback, this));

  m_image_pub = this->create_publisher<sensor_msgs::msg::Image>("go2_camera/image", 10);

  m_camera_address = "udpsrc address=230.1.1.1 port=1720 multicast-iface=wlp2s0 ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1";
  m_cap.open(m_camera_address, cv::CAP_GSTREAMER);

  if (!m_cap.isOpened())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
  }

  RCLCPP_INFO(this->get_logger(), "Go2CameraNode started.");
}

Go2CameraNode::~Go2CameraNode()
{
  RCLCPP_WARN(this->get_logger(), "Go2CameraNode shutting down.");
}

void Go2CameraNode::timerCallback()
{
  RCLCPP_INFO(this->get_logger(), "Timer callback triggered.");

  cv::Mat frame;
  m_cap >> frame;

  if (frame.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to capture frame.");
    return;
  }

  sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  publishImage(msg);
}

void Go2CameraNode::publishImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Publishing image.");
  m_image_pub->publish(*msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2CameraNode>());
  rclcpp::shutdown();
  return 0;
}

// Path: ros2_ws/src/ltm_go2/ltm_go2_camera/src/go2_camera_node.cpp