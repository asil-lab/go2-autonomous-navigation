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
  // Update odom
  m_odom_msg->header.stamp = rclcpp::Clock().now();
  updateOdomTranslation(translation);
  updateOdomOrientation(orientation);

  // Update base_footprint
  m_base_footprint_msg->header.stamp = rclcpp::Clock().now();
  updateBaseFootprintTranslation(translation);
}

void OdomProcessing::updateOdom(const geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped)
{
  // Update odom
  m_odom_msg->header.stamp = pose_stamped->header.stamp;
  updateOdomTranslation(pose_stamped->pose.position);
  updateOdomOrientation(pose_stamped->pose.orientation);

  // Update base_footprint
  m_base_footprint_msg->header.stamp = rclcpp::Clock().now();
  updateBaseFootprintTranslation(pose_stamped->pose.position);
}

void OdomProcessing::updateOdomTranslation(const std::array<float, TRANSLATION_SIZE>& translation)
{
  m_odom_msg->transform.translation.x = translation[static_cast<int>(TranslationIdx::X)];
  m_odom_msg->transform.translation.y = translation[static_cast<int>(TranslationIdx::Y)];
  m_odom_msg->transform.translation.z = translation[static_cast<int>(TranslationIdx::Z)];
}

void OdomProcessing::updateOdomTranslation(const geometry_msgs::msg::Point& position)
{
  m_odom_msg->transform.translation.x = position.x;
  m_odom_msg->transform.translation.y = position.y;
  m_odom_msg->transform.translation.z = position.z;
}

void OdomProcessing::updateOdomOrientation(const std::array<float, ORIENTATION_SIZE>& rotation)
{
  m_odom_msg->transform.rotation.x = rotation[static_cast<int>(OrientationIdx::X)];
  m_odom_msg->transform.rotation.y = rotation[static_cast<int>(OrientationIdx::Y)];
  m_odom_msg->transform.rotation.z = rotation[static_cast<int>(OrientationIdx::Z)];
  m_odom_msg->transform.rotation.w = rotation[static_cast<int>(OrientationIdx::W)];
}

void OdomProcessing::updateOdomOrientation(const geometry_msgs::msg::Quaternion& orientation)
{
  m_odom_msg->transform.rotation = orientation;
}

void OdomProcessing::updateBaseFootprintTranslation(const std::array<float, TRANSLATION_SIZE>& translation)
{
  m_base_footprint_msg->transform.translation.z = -translation[static_cast<int>(TranslationIdx::Z)];
}

void OdomProcessing::updateBaseFootprintTranslation(const geometry_msgs::msg::Point& position)
{
  m_base_footprint_msg->transform.translation.z = -position.z;
}

geometry_msgs::msg::TransformStamped::SharedPtr OdomProcessing::getOdomMsg() const
{
  return m_odom_msg;
}

geometry_msgs::msg::TransformStamped::SharedPtr OdomProcessing::getBaseFootprintMsg() const
{
  return m_base_footprint_msg;
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

void OdomProcessing::initializeBaseFootprintMsg()
{
  m_base_footprint_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
  m_base_footprint_msg->header.frame_id = BASE_FOOTPRINT_PARENT_ID;
  m_base_footprint_msg->child_frame_id = BASE_FOOTPRINT_FRAME_ID;

  m_base_footprint_msg->transform.translation.x = 0.0;
  m_base_footprint_msg->transform.translation.y = 0.0;
  m_base_footprint_msg->transform.translation.z = 0.0;

  m_base_footprint_msg->transform.rotation.x = 0.0;
  m_base_footprint_msg->transform.rotation.y = 0.0;
  m_base_footprint_msg->transform.rotation.z = 0.0;
  m_base_footprint_msg->transform.rotation.w = 1.0;
}

// End of file: odom_processing.cpp