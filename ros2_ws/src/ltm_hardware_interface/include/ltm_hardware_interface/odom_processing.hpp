/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 04-08-2024.
 */

#ifndef LTM_HARDWARE_INTERFACE__ODOM_PROCESSING_HPP_
#define LTM_HARDWARE_INTERFACE__ODOM_PROCESSING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>

#define ROS_NODE_NAME "odom_processing_node"
#define ROS_SUB_TOPIC "utlidar/robot_pose"
#define ROS_SUB_QUEUE_SIZE 10

#define ODOM_FRAME_ID "odom"
#define ODOM_CHILD_FRAME_ID "base"

#define BASE_FOOTPRINT_FRAME_ID "base_footprint"
#define BASE_FOOTPRINT_PARENT_ID "base"

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
      void robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
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
      std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  };  // class OdomProcessing
} // namespace LTM

#endif  // LTM_HARDWARE_INTERFACE__ODOM_PROCESSING_HPP_

// End of file: odom_processing.hpp