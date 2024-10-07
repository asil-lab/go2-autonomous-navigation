/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 04-08-2024.
 */

#ifndef LTM_GO2_DRIVER__ODOM_PROCESSING_HPP_
#define LTM_GO2_DRIVER__ODOM_PROCESSING_HPP_

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/imu_state.hpp>

#include <string>

#define ODOM_PROCESSING_NODE_NAME "odom_processing_node"

#define ODOM_PROCESSING_SUB_ROBOT_POSE_TOPIC "utlidar/robot_pose"
#define ODOM_PROCESSING_SUB_ROBOT_POSE_QUEUE_SIZE 1

#define ODOM_PROCESSING_SUB_SPORT_MODE_STATE_TOPIC "sportmodestate"
#define ODOM_PROCESSING_SUB_SPORT_MODE_STATE_QUEUE_SIZE 1

#define ODOM_PROCESSING_SUB_LOW_STATE_TOPIC "lowstate"
#define ODOM_PROCESSING_SUB_LOW_STATE_QUEUE_SIZE 1

#define ODOM_PROCESSING_PUB_IMU_TOPIC "imu"
#define ODOM_PROCESSING_PUB_IMU_QUEUE_SIZE 1

#define ODOM_FRAME_ID "odom"
#define ODOM_CHILD_FRAME_ID "base"

#define BASE_FOOTPRINT_FRAME_ID "base_footprint"
#define BASE_FOOTPRINT_PARENT_ID "base"

#define IMU_FRAME_ID "imu"

#define TRANSLATION_SIZE 3
#define ORIENTATION_SIZE 4
namespace LTM
{
  class OdomProcessing : public rclcpp::Node
  {
    public:
      OdomProcessing();
      ~OdomProcessing();

      geometry_msgs::msg::TransformStamped::SharedPtr getOdomMsg() const;
      geometry_msgs::msg::TransformStamped::SharedPtr getBaseFootprintMsg() const;

    private:
      void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
      void robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
      void sportModeStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg);
      void publishImu(const unitree_go::msg::IMUState& imu_state);
      void broadcastTransform(const geometry_msgs::msg::TransformStamped::SharedPtr msg) const;

      void updateOdom(const std::array<float, TRANSLATION_SIZE>& translation,
        const std::array<float, ORIENTATION_SIZE>& orientation);
      void updateOdom(const geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped);

      void updateOdomTranslation(const std::array<float, TRANSLATION_SIZE>& translation);
      void updateOdomTranslation(const geometry_msgs::msg::Point& position);
      void updateOdomOrientation(const std::array<float, ORIENTATION_SIZE>& rotation);
      void updateOdomOrientation(const geometry_msgs::msg::Quaternion& orientation);

      void updateBaseFootprintTranslation(const std::array<float, TRANSLATION_SIZE>& translation);
      void updateBaseFootprintTranslation(const geometry_msgs::msg::Point& position);
      void updateBaseFootprintOrientation(const std::array<float, ORIENTATION_SIZE>& rotation);
      void updateBaseFootprintOrientation(const geometry_msgs::msg::Quaternion& orientation);
      void updateBaseFootprintOrientation(const unitree_go::msg::IMUState& imu_state);

      // TODO: Generalize updateFrameTranslation
      // TODO: Generalize updateFrameOrientation

      void initializeROS();
      void initializeOdomTransformMsg();
      void initializeBaseFootprintMsg();

      std::array<float, ORIENTATION_SIZE> invertQuaternion(
        const std::array<float, ORIENTATION_SIZE>& quaternion);
      geometry_msgs::msg::Quaternion invertQuaternion(
        const geometry_msgs::msg::Quaternion& quaternion);

      std::array<float, ORIENTATION_SIZE> normalizeQuaternion(
        const std::array<float, ORIENTATION_SIZE>& quaternion);
      geometry_msgs::msg::Quaternion normalizeQuaternion(
        const geometry_msgs::msg::Quaternion& quaternion);

      double getQuaternionNorm(const std::array<float, ORIENTATION_SIZE>& quaternion);
      double getQuaternionNorm(const geometry_msgs::msg::Quaternion& quaternion);

      enum class TranslationIdx { X = 0, Y = 1, Z = 2 };
      enum class OrientationIdx { X = 1, Y = 2, Z = 3, W = 0 };

      geometry_msgs::msg::TransformStamped::SharedPtr m_odom_msg;
      geometry_msgs::msg::TransformStamped::SharedPtr m_base_footprint_msg;

      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_robot_pose_sub;
      rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr m_sport_mode_state_sub;
      rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr m_low_state_sub;
      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_pub;

      std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  };  // class OdomProcessing
} // namespace LTM

#endif  // LTM_GO2_DRIVER__ODOM_PROCESSING_HPP_

// End of file: odom_processing.hpp