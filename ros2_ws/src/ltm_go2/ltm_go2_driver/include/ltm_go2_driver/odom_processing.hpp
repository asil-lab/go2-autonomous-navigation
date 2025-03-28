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

#define ODOM_PROCESSING_SUB_SPORT_MODE_STATE_TOPIC "sportmodestate"
#define ODOM_PROCESSING_SUB_SPORT_MODE_STATE_QUEUE_SIZE 10

#define ODOM_PROCESSING_SUB_LOW_STATE_TOPIC "lowstate"
#define ODOM_PROCESSING_SUB_LOW_STATE_QUEUE_SIZE 10

#define ODOM_PROCESSING_PUB_IMU_TOPIC "imu"
#define ODOM_PROCESSING_PUB_IMU_QUEUE_SIZE 1

#define ODOM_FRAME_ID "odom"
#define ODOM_CHILD_FRAME_ID "base"

#define BASE_FOOTPRINT_FRAME_ID "base_footprint"
#define BASE_FOOTPRINT_PARENT_ID "base"

#define IMU_FRAME_ID "imu"

#define ROBOT_POSE_TOPIC "robot_pose/odom"
#define ROBOT_POSE_QUEUE_SIZE 1

#define TRANSLATION_SIZE 3
#define ORIENTATION_SIZE 4
namespace LTM
{
  class OdomProcessing : public rclcpp::Node
  {
    public:
      OdomProcessing();
      ~OdomProcessing();

    private:
      void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
      void sportModeStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg);
      void publishImu(const unitree_go::msg::IMUState& imu_state);
      void publishRobotPose(const std::array<float, TRANSLATION_SIZE>& translation,
        const std::array<float, ORIENTATION_SIZE>& orientation) const;
      void broadcastTransform(const geometry_msgs::msg::TransformStamped::SharedPtr msg) const;

      void updateOdom(const std::array<float, TRANSLATION_SIZE>& translation,
        const std::array<float, ORIENTATION_SIZE>& orientation);

      void updateOdomTranslation(const std::array<float, TRANSLATION_SIZE>& translation);
      void updateOdomOrientation(const std::array<float, ORIENTATION_SIZE>& rotation);

      void updateBaseFootprint(const std::array<float, TRANSLATION_SIZE>& translation, 
        const std::array<float, ORIENTATION_SIZE>& rotation);

      void initializeROS();
      void initializeOdomTransformMsg();
      void initializeBaseFootprintMsg();

      void declareInitialTranslation();
      void initializeInitialTranslation(const double& x, const double& y);

      enum class TranslationIdx { X = 0, Y = 1, Z = 2 };
      enum class OrientationIdx { X = 1, Y = 2, Z = 3, W = 0 };

      std::array<float, TRANSLATION_SIZE> m_initial_translation;
      geometry_msgs::msg::TransformStamped::SharedPtr m_odom_msg;
      geometry_msgs::msg::TransformStamped::SharedPtr m_base_footprint_msg;

      rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr m_sport_mode_state_sub;
      rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr m_low_state_sub;
      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_pub;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_robot_pose_pub;

      std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  };  // class OdomProcessing
} // namespace LTM

#endif  // LTM_GO2_DRIVER__ODOM_PROCESSING_HPP_

// End of file: odom_processing.hpp