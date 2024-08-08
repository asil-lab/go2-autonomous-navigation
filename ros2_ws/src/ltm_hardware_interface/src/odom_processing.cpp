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
  initializeBaseFootprintMsg();
}

OdomProcessing::~OdomProcessing()
{
  m_odom_msg.reset();
  m_base_footprint_msg.reset();
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
  updateBaseFootprintOrientation(orientation);
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
  updateBaseFootprintOrientation(pose_stamped->pose.orientation);
}

geometry_msgs::msg::TransformStamped::SharedPtr OdomProcessing::getOdomMsg() const
{
  return m_odom_msg;
}

geometry_msgs::msg::TransformStamped::SharedPtr OdomProcessing::getBaseFootprintMsg() const
{
  return m_base_footprint_msg;
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

void OdomProcessing::updateBaseFootprintOrientation(const std::array<float, ORIENTATION_SIZE>& rotation)
{
  std::array<float, ORIENTATION_SIZE> inverse_rotation = invertQuaternion(rotation);
  
  // Focus only on pitch due to sitting
  inverse_rotation[static_cast<int>(OrientationIdx::X)] = 0.0;
  inverse_rotation[static_cast<int>(OrientationIdx::Z)] = 0.0;
  inverse_rotation = normalizeQuaternion(inverse_rotation);

  m_base_footprint_msg->transform.rotation.y = inverse_rotation[static_cast<int>(OrientationIdx::Y)];
  m_base_footprint_msg->transform.rotation.w = inverse_rotation[static_cast<int>(OrientationIdx::W)];
}

void OdomProcessing::updateBaseFootprintOrientation(const geometry_msgs::msg::Quaternion& orientation)
{
  geometry_msgs::msg::Quaternion inverse_orientation = invertQuaternion(orientation);

  // Focus only on pitch due to sitting
  inverse_orientation.x = 0.0;
  inverse_orientation.z = 0.0;
  inverse_orientation = normalizeQuaternion(inverse_orientation);

  m_base_footprint_msg->transform.rotation.y = inverse_orientation.y;
  m_base_footprint_msg->transform.rotation.w = inverse_orientation.w;
}

std::array<float, ORIENTATION_SIZE> OdomProcessing::invertQuaternion(
  const std::array<float, ORIENTATION_SIZE>& quaternion)
{
  std::array<float, ORIENTATION_SIZE> inverse_quaternion;
  double norm = getQuaternionNorm(quaternion);

  for (int i = 0; i < ORIENTATION_SIZE - 1; i++)
  {
    inverse_quaternion[i] = -quaternion[i] / norm;
  }

  inverse_quaternion[ORIENTATION_SIZE - 1] = quaternion[ORIENTATION_SIZE - 1] / norm;
  return inverse_quaternion;
}

geometry_msgs::msg::Quaternion OdomProcessing::invertQuaternion(
  const geometry_msgs::msg::Quaternion& quaternion)
{
  geometry_msgs::msg::Quaternion inverse_quaternion;
  double norm = getQuaternionNorm(quaternion);

  inverse_quaternion.x = -quaternion.x / norm;
  inverse_quaternion.y = -quaternion.y / norm;
  inverse_quaternion.z = -quaternion.z / norm;
  inverse_quaternion.w = quaternion.w / norm;

  return inverse_quaternion;
}

std::array<float, ORIENTATION_SIZE> OdomProcessing::normalizeQuaternion(
  const std::array<float, ORIENTATION_SIZE>& quaternion)
{
  std::array<float, ORIENTATION_SIZE> normalized_quaternion;
  double norm = getQuaternionNorm(quaternion);
  for (int i = 0; i < ORIENTATION_SIZE; i++) {
    normalized_quaternion[i] = quaternion[i] / norm;
  }
  return normalized_quaternion;
}

geometry_msgs::msg::Quaternion OdomProcessing::normalizeQuaternion(
  const geometry_msgs::msg::Quaternion& quaternion)
{
  geometry_msgs::msg::Quaternion normalized_quaternion;
  double norm = getQuaternionNorm(quaternion);
  normalized_quaternion.x = quaternion.x / norm;
  normalized_quaternion.y = quaternion.y / norm;
  normalized_quaternion.z = quaternion.z / norm;
  normalized_quaternion.w = quaternion.w / norm;
  return normalized_quaternion;
}

double OdomProcessing::getQuaternionNorm(const std::array<float, ORIENTATION_SIZE>& quaternion)
{
  double norm = 0.0;
  for (int i = 0; i < ORIENTATION_SIZE; i++) {
    norm += quaternion[i] * quaternion[i];
  }
  return norm;
}

double OdomProcessing::getQuaternionNorm(const geometry_msgs::msg::Quaternion& quaternion)
{
  return quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
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