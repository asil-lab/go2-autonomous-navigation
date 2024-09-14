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
  publishImage(captureImage());
}

void Go2CameraNode::serviceCallback(const std::shared_ptr<ltm_shared_msgs::srv::GetImage::Request> request,
  std::shared_ptr<ltm_shared_msgs::srv::GetImage::Response> response)
{
  (void) request;
  response->image = *(captureImage());
}

void Go2CameraNode::publishImage(const sensor_msgs::msg::Image::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "Publishing image.");
  m_image_pub->publish(*msg);
}

sensor_msgs::msg::Image::SharedPtr Go2CameraNode::captureImage()
{
  // Initialize camera stream.
  cv::Mat frame;
  m_cap >> frame;

  // Shutdown if camera fails to capture frame.
  if (frame.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to capture frame.");
    return sensor_msgs::msg::Image::SharedPtr();
  }

  // Convert frame to image message.
  sensor_msgs::msg::Image::SharedPtr image_msg 
    = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

  // Set image message header.
  image_msg->header.frame_id = m_camera_frame_id;
  image_msg->header.stamp = this->now();

  return image_msg;
}

void Go2CameraNode::initializeCamera()
{
  declare_parameter("camera_address.address", "230.1.1.1");
  declare_parameter("camera_address.port", 1720);
  declare_parameter("camera_address.multicast_iface", "wlp2s0");

  // Construct camera address string.
  std::stringstream camera_address_stream;
  camera_address_stream << "udpsrc address=" << 
    this->get_parameter("camera_address.address").as_string() << 
    " port=" << this->get_parameter("camera_address.port").as_int() << 
    " multicast-iface=" << this->get_parameter("camera_address.multicast_iface").as_string() 
    << " ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1";

  // Open camera stream.
  m_camera_address = camera_address_stream.str();
  m_cap.open(m_camera_address, cv::CAP_GSTREAMER);

  // Shutdown if camera fails to open.
  if (!m_cap.isOpened())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
    rclcpp::shutdown();
  }

  // Initialize camera frame id.
  declare_parameter("camera_frame_id", "camera_link");
  m_camera_frame_id = this->get_parameter("camera_frame_id").as_string();

  RCLCPP_INFO(this->get_logger(), "Camera initialized.");
}

void Go2CameraNode::initializeStreamMode()
{
  // Assert that stream mode is enabled.
  declare_parameter("stream_mode", false);
  if (!this->get_parameter("stream_mode").as_bool())
  {
    RCLCPP_INFO(this->get_logger(), "Stream mode is disabled.");
    return;
  }

  // Initialize image publisher and timer to publish images at a fixed rate.
  initializeImagePublisher();
  initializeTimer();
}

void Go2CameraNode::initializeServiceMode()
{
  // Assert that service mode is enabled.
  declare_parameter("service_mode", false);
  if (!this->get_parameter("service_mode").as_bool())
  {
    RCLCPP_INFO(this->get_logger(), "Service mode is disabled.");
    return;
  }

  // Initialize image service.
  initializeImageService();
}

void Go2CameraNode::initializeTimer()
{
  declare_parameter("image_pub_topic_rate", 24.0);

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
}

void Go2CameraNode::initializeImagePublisher()
{
  declare_parameter("image_pub_topic_name", "camera/raw");
  declare_parameter("image_pub_topic_queue_size", 10);
  declare_parameter("image_pub_topic_frame_id", "camera_link");
  declare_parameter("image_pub_topic_rate", 24.0);

  // Initialize image publisher.
  m_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
    this->get_parameter("image_pub_topic_name").as_string(),
    this->get_parameter("image_pub_topic_queue_size").as_int());
}

void Go2CameraNode::initializeImageService()
{
  declare_parameter("image_service_name", "camera/raw");

  // Initialize image service.
  m_image_service = this->create_service<ltm_shared_msgs::srv::GetImage>(
    this->get_parameter("image_service_name").as_string(),
    std::bind(&Go2CameraNode::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto go2_camera_node = std::make_shared<Go2CameraNode>();
  executor.add_node(go2_camera_node);
  executor.spin();
  rclcpp::shutdown();
}

// Path: ros2_ws/src/ltm_go2/ltm_go2_camera/src/go2_camera_node.cpp