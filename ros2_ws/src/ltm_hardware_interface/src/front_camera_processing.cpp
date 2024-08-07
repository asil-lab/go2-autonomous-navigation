/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 04-08-2024.
 */

#include <ltm_hardware_interface/front_camera_processing.hpp>

using namespace LTM;

FrontCameraProcessing::FrontCameraProcessing()
{
  initializeFrontCameraMsgs();
}

FrontCameraProcessing::~FrontCameraProcessing()
{
  m_front_camera_180p_msg.reset();
  m_front_camera_360p_msg.reset();
  m_front_camera_720p_msg.reset();
}

void FrontCameraProcessing::updateFrontCameraMsgs(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg,
  const rclcpp::Time& timestamp)
{
  // Update the front camera image messages
  // Front camera 180p
  m_front_camera_180p_msg->header.stamp = timestamp;
  m_front_camera_180p_msg->data = msg->video180p;
  std::cout << msg->video180p.size() << std::endl;

  // Front camera 360p
  m_front_camera_360p_msg->header.stamp = timestamp;
  m_front_camera_360p_msg->data = msg->video360p;

  // Front camera 720p
  m_front_camera_720p_msg->header.stamp = timestamp;
  m_front_camera_720p_msg->data = msg->video180p;
}

sensor_msgs::msg::Image::SharedPtr FrontCameraProcessing::getFrontCamera180pMsg() const
{
  return m_front_camera_180p_msg;
}

sensor_msgs::msg::Image::SharedPtr FrontCameraProcessing::getFrontCamera360pMsg() const
{
  return m_front_camera_360p_msg;
}

sensor_msgs::msg::Image::SharedPtr FrontCameraProcessing::getFrontCamera720pMsg() const
{
  return m_front_camera_720p_msg;
}

void FrontCameraProcessing::initializeFrontCameraMsgs()
{
  // Initialize the front camera image messages
  // Front camera 180p
  m_front_camera_180p_msg = std::make_shared<sensor_msgs::msg::Image>();
  m_front_camera_180p_msg->header.frame_id = FRONT_CAMERA_FRAME_ID;
  m_front_camera_180p_msg->height = FRONT_CAMERA_180p_HEIGHT;
  m_front_camera_180p_msg->width = FRONT_CAMERA_180p_WIDTH;
  m_front_camera_180p_msg->encoding = IMAGE_ENCODING;
  m_front_camera_180p_msg->is_bigendian = IMAGE_IS_BIGENDIAN;
  m_front_camera_180p_msg->step = FRONT_CAMERA_180p_WIDTH * IMAGE_STEP_SCALE;

  // Front camera 360p
  m_front_camera_360p_msg = std::make_shared<sensor_msgs::msg::Image>();
  m_front_camera_360p_msg->header.frame_id = FRONT_CAMERA_FRAME_ID;
  m_front_camera_360p_msg->height = FRONT_CAMERA_360p_HEIGHT;
  m_front_camera_360p_msg->width = FRONT_CAMERA_360p_WIDTH;
  m_front_camera_360p_msg->encoding = IMAGE_ENCODING;
  m_front_camera_360p_msg->is_bigendian = IMAGE_IS_BIGENDIAN;
  m_front_camera_360p_msg->step = FRONT_CAMERA_360p_WIDTH * IMAGE_STEP_SCALE;

  // Front camera 720p
  m_front_camera_720p_msg = std::make_shared<sensor_msgs::msg::Image>();
  m_front_camera_720p_msg->header.frame_id = FRONT_CAMERA_FRAME_ID;
  m_front_camera_720p_msg->height = FRONT_CAMERA_720p_HEIGHT;
  m_front_camera_720p_msg->width = FRONT_CAMERA_720p_WIDTH;
  m_front_camera_720p_msg->encoding = IMAGE_ENCODING;
  m_front_camera_720p_msg->is_bigendian = IMAGE_IS_BIGENDIAN;
  m_front_camera_720p_msg->step = FRONT_CAMERA_720p_WIDTH * IMAGE_STEP_SCALE;
}

// End of file: front_camera_processing.cpp