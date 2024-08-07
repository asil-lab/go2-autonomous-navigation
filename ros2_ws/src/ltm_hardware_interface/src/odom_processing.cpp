/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 04-08-2024.
 */

#include <ltm_hardware_interface/odom_processing.hpp>

using namespace LTM;

OdomProcessing::OdomProcessing()
{
  initializeOdomTransformMsg();
}

OdomProcessing::~OdomProcessing()
{
  m_odom_msg.reset();
}

void OdomProcessing::updateOdom(const std::array<float, TRANSLATION_SIZE>& translation,
  const std::array<float, ORIENTATION_SIZE>& orientation)
{
  m_odom_msg->header.stamp = rclcpp::Clock().now();
  updateOdomTranslation(translation);
  updateOdomOrientation(orientation);
}

void OdomProcessing::updateOdom(const geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped)
{
  m_odom_msg->header.stamp = pose_stamped->header.stamp;
  updateOdomTranslation(pose_stamped->pose.position);
  updateOdomOrientation(pose_stamped->pose.orientation);
}

void OdomProcessing::updateOdomTranslation(const std::array<float, TRANSLATION_SIZE>& translation)
{
  m_odom_msg->transform.translation.x = translation[static_cast<int>(TranslationIdx::X)];
  m_odom_msg->transform.translation.y = translation[static_cast<int>(TranslationIdx::Y)];
  m_odom_msg->transform.translation.z = translation[static_cast<int>(TranslationIdx::Z)];
}

void OdomProcessing::updateOdomOrientation(const std::array<float, ORIENTATION_SIZE>& rotation)
{
  m_odom_msg->transform.rotation.x = rotation[static_cast<int>(OrientationIdx::X)];
  m_odom_msg->transform.rotation.y = rotation[static_cast<int>(OrientationIdx::Y)];
  m_odom_msg->transform.rotation.z = rotation[static_cast<int>(OrientationIdx::Z)];
  m_odom_msg->transform.rotation.w = rotation[static_cast<int>(OrientationIdx::W)];
}

void OdomProcessing::updateOdomTranslation(const geometry_msgs::msg::Point& translation)
{
  m_odom_msg->transform.translation.x = translation.x;
  m_odom_msg->transform.translation.y = translation.y;
  m_odom_msg->transform.translation.z = translation.z;
}

void OdomProcessing::updateOdomOrientation(const geometry_msgs::msg::Quaternion& rotation)
{
  m_odom_msg->transform.rotation = rotation;
}

geometry_msgs::msg::TransformStamped::SharedPtr OdomProcessing::getOdomMsg() const
{
  return m_odom_msg;
}

void OdomProcessing::initializeOdomTransformMsg()
{
  m_odom_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
  m_odom_msg->header.frame_id = ODOM_FRAME_ID;
  m_odom_msg->child_frame_id = ODOM_CHILD_FRAME_ID;
  m_odom_msg->transform.translation.x = 0.0;
  m_odom_msg->transform.translation.y = 0.0;
  m_odom_msg->transform.translation.z = 0.0;
  m_odom_msg->transform.rotation.x = 0.0;
  m_odom_msg->transform.rotation.y = 0.0;
  m_odom_msg->transform.rotation.z = 0.0;
  m_odom_msg->transform.rotation.w = 1.0;
}

// End of file: odom_processing.cpp