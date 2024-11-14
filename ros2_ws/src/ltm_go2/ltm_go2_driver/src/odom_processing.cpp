/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 04-08-2024.
 */

#include <ltm_go2_driver/odom_processing.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace LTM;

OdomProcessing::OdomProcessing() : Node(ODOM_PROCESSING_NODE_NAME)
{
  declareInitialTranslation();
  initializeOdomTransformMsg();
  initializeBaseFootprintMsg();
  initializeROS();
  RCLCPP_INFO(this->get_logger(), "Odom Processing Node initialized.");
}

OdomProcessing::~OdomProcessing()
{
  m_odom_msg.reset();
  m_base_footprint_msg.reset();
  m_tf_broadcaster.reset();
  RCLCPP_WARN(this->get_logger(), "Odom Processing Node destroyed.");
}

void OdomProcessing::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Received Low State Message.");
  publishImu(msg->imu_state);
}

void OdomProcessing::sportModeStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg)
{
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Received Sport Mode State Message.");

  // Initialize initial translation to set starting position to (0, 0)
  if (m_initial_translation[static_cast<int>(TranslationIdx::Z)] != 1.0)
  {
    initializeInitialTranslation(msg->position[static_cast<int>(TranslationIdx::X)], 
      msg->position[static_cast<int>(TranslationIdx::Y)]);
  }

  // Update odom and base_footprint
  updateOdom(msg->position, msg->imu_state.quaternion);
  broadcastTransform(m_odom_msg);
  broadcastTransform(m_base_footprint_msg);
}

void OdomProcessing::publishImu(const unitree_go::msg::IMUState& imu_state)
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = rclcpp::Clock().now();
  imu_msg.header.frame_id = IMU_FRAME_ID;

  imu_msg.orientation.x = imu_state.quaternion[static_cast<int>(OrientationIdx::X)];
  imu_msg.orientation.y = imu_state.quaternion[static_cast<int>(OrientationIdx::Y)];
  imu_msg.orientation.z = imu_state.quaternion[static_cast<int>(OrientationIdx::Z)];
  imu_msg.orientation.w = imu_state.quaternion[static_cast<int>(OrientationIdx::W)];

  imu_msg.angular_velocity.x = imu_state.gyroscope[static_cast<int>(TranslationIdx::X)];
  imu_msg.angular_velocity.y = imu_state.gyroscope[static_cast<int>(TranslationIdx::Y)];
  imu_msg.angular_velocity.z = imu_state.gyroscope[static_cast<int>(TranslationIdx::Z)];

  imu_msg.linear_acceleration.x = imu_state.accelerometer[static_cast<int>(TranslationIdx::X)];
  imu_msg.linear_acceleration.y = imu_state.accelerometer[static_cast<int>(TranslationIdx::Y)];
  imu_msg.linear_acceleration.z = imu_state.accelerometer[static_cast<int>(TranslationIdx::Z)];

  m_imu_pub->publish(imu_msg);
}

void OdomProcessing::broadcastTransform(const geometry_msgs::msg::TransformStamped::SharedPtr msg) const
{
  m_tf_broadcaster->sendTransform(*msg);
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
  updateBaseFootprint(translation, orientation);
}

void OdomProcessing::updateOdomTranslation(const std::array<float, TRANSLATION_SIZE>& translation)
{
  m_odom_msg->transform.translation.x = translation[static_cast<int>(TranslationIdx::X)] - m_initial_translation[static_cast<int>(TranslationIdx::X)];
  m_odom_msg->transform.translation.y = translation[static_cast<int>(TranslationIdx::Y)] - m_initial_translation[static_cast<int>(TranslationIdx::Y)];
  m_odom_msg->transform.translation.z = translation[static_cast<int>(TranslationIdx::Z)];
}

void OdomProcessing::updateOdomOrientation(const std::array<float, ORIENTATION_SIZE>& rotation)
{
  m_odom_msg->transform.rotation.x = rotation[static_cast<int>(OrientationIdx::X)];
  m_odom_msg->transform.rotation.y = rotation[static_cast<int>(OrientationIdx::Y)];
  m_odom_msg->transform.rotation.z = rotation[static_cast<int>(OrientationIdx::Z)];
  m_odom_msg->transform.rotation.w = rotation[static_cast<int>(OrientationIdx::W)];
}

void OdomProcessing::updateBaseFootprint(
  const std::array<float, TRANSLATION_SIZE>& translation, const std::array<float, ORIENTATION_SIZE>& rotation)
{
  Eigen::Quaterniond q(rotation[static_cast<int>(OrientationIdx::W)], rotation[static_cast<int>(OrientationIdx::X)],
    rotation[static_cast<int>(OrientationIdx::Y)], rotation[static_cast<int>(OrientationIdx::Z)]);
  Eigen::Quaterniond q_inverse = q.inverse();

  // Rotate the translation vector by the inverse of the orientation quaternion
  Eigen::Vector3d translation_vector(0, 0, -translation[static_cast<int>(TranslationIdx::Z)]);
  Eigen::Vector3d rotated_translation = q_inverse * translation_vector;

  // Update the base_footprint frame translation given the rotated translation vector.
  m_base_footprint_msg->transform.translation.x = rotated_translation.x();
  m_base_footprint_msg->transform.translation.y = rotated_translation.y();
  m_base_footprint_msg->transform.translation.z = rotated_translation.z();

  // Get the yaw angle from the orientation quaternion
  // double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
  // Eigen::Quaterniond q_reduced(cos(yaw / 2), 0, 0, sin(yaw / 2));
  // Eigen::Quaterniond q_rotated = q_reduced * q_inverse;

  // Update the base_footprint frame orientation given the rotated quaternion.
  m_base_footprint_msg->transform.rotation.x = q_inverse.x();
  m_base_footprint_msg->transform.rotation.y = q_inverse.y();
  m_base_footprint_msg->transform.rotation.z = q_inverse.z();
  m_base_footprint_msg->transform.rotation.w = q_inverse.w();
}

void OdomProcessing::initializeROS()
{
  m_sport_mode_state_sub = this->create_subscription<unitree_go::msg::SportModeState>(
    ODOM_PROCESSING_SUB_SPORT_MODE_STATE_TOPIC, ODOM_PROCESSING_SUB_SPORT_MODE_STATE_QUEUE_SIZE, 
    std::bind(&OdomProcessing::sportModeStateCallback, this, std::placeholders::_1));

  m_low_state_sub = this->create_subscription<unitree_go::msg::LowState>(
    ODOM_PROCESSING_SUB_LOW_STATE_TOPIC, ODOM_PROCESSING_SUB_LOW_STATE_QUEUE_SIZE, 
    std::bind(&OdomProcessing::lowStateCallback, this, std::placeholders::_1));

  m_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(
    ODOM_PROCESSING_PUB_IMU_TOPIC, ODOM_PROCESSING_PUB_IMU_QUEUE_SIZE);

  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this); 
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

void OdomProcessing::declareInitialTranslation()
{
  m_initial_translation = {0.0, 0.0, 0.0};
}

void OdomProcessing::initializeInitialTranslation(const double& x, const double& y)
{
  m_initial_translation[static_cast<int>(TranslationIdx::X)] = x;
  m_initial_translation[static_cast<int>(TranslationIdx::Y)] = y;
  m_initial_translation[static_cast<int>(TranslationIdx::Z)] = 1.0;
}

// End of file: odom_processing.cpp