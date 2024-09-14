/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 12-09-2024.
 */

#include <ltm_go2_camera/go2_camera_node.hpp>

#include <chrono>
#include <stdio.h>
#include <sstream>

#include <cv_bridge/cv_bridge.h>

using namespace LTM;

Go2CameraNode::Go2CameraNode() : Node("go2_camera_node")
{
  RCLCPP_INFO(this->get_logger(), "Go2CameraNode starting up.");

  initializeCamera();
  initializeImagePublisher();
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

void Go2CameraNode::initializeCamera()
{
  declare_parameter("camera_address.address", "230.1.1.1");
  declare_parameter("camera_address.port", 1720);
  declare_parameter("camera_address.multicast_iface", "wlp2s0");

  std::stringstream camera_address_stream;
  camera_address_stream << "udpsrc address=" << 
    this->get_parameter("camera_address.address").as_string() << 
    " port=" << this->get_parameter("camera_address.port").as_int() << 
    " multicast-iface=" << this->get_parameter("camera_address.multicast_iface").as_string() 
    << " ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1";

  m_camera_address = camera_address_stream.str();
  m_cap.open(m_camera_address, cv::CAP_GSTREAMER);

  // Shutdown if camera fails to open.
  if (!m_cap.isOpened())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(this->get_logger(), "Camera initialized.");
}
void Go2CameraNode::initializeImagePublisher()
{
  // Initialize image publisher if stream mode is enabled.
  declare_parameter("stream_mode", false);
  if (!this->get_parameter("stream_mode").as_bool())
  {
    RCLCPP_INFO(this->get_logger(), "Stream mode is disabled.");
    return;
  }

  declare_parameter("image_pub_topic_name", "camera/raw");
  declare_parameter("image_pub_topic_queue_size", 10);
  declare_parameter("image_pub_topic_frame_id", "camera_link");
  declare_parameter("image_pub_topic_rate", 24.0);


  m_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
    this->get_parameter("image_pub_topic_name").as_string(),
    this->get_parameter("image_pub_topic_queue_size").as_int());

  // Assert that the camera frame id is not empty.
  if (m_camera_frame_id.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Camera frame id is empty.");
    rclcpp::shutdown();
  }

  m_camera_frame_id = this->get_parameter("image_pub_frame_id").as_string();

  // Assert that the rate is not zero.
  if (this->get_parameter("image_pub_topic_rate").as_double() == 0.0)
  {
    RCLCPP_ERROR(this->get_logger(), "Image publisher rate is zero.");
    rclcpp::shutdown();
  }

  unsigned long int period = static_cast<unsigned long int>(
    1.0 / this->get_parameter("image_pub_topic_rate").as_double() * 1e9);
  m_timer = this->create_wall_timer(
    std::chrono::nanoseconds(period), std::bind(&Go2CameraNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Image publisher initialized.");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2CameraNode>());
  rclcpp::shutdown();
  return 0;
}

// Path: ros2_ws/src/ltm_go2/ltm_go2_camera/src/go2_camera_node.cpp