/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 04-08-2024.
 */

#ifndef LTM_HARDWARE_INTERFACE__ODOM_PROCESSING_HPP_
#define LTM_HARDWARE_INTERFACE__ODOM_PROCESSING_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

// #define ODOM_FRAME_ID "odom"
#define ODOM_FRAME_ID "world"
#define ODOM_CHILD_FRAME_ID "base"
#define TRANSLATION_SIZE 3
#define ORIENTATION_SIZE 4

namespace LTM
{
  class OdomProcessing
  {
    public:
      OdomProcessing(const rclcpp::Node::SharedPtr& node);
      ~OdomProcessing();

      void updateOdomTranslation(const std::array<float, TRANSLATION_SIZE>& translation);
      void updateOdomOrientation(const std::array<float, ORIENTATION_SIZE>& orientation);
      void broadcastOdomTransform();

    private:
      void initializeOdomTransformMsg();

      enum class TranslationIdx { X = 0, Y = 1, Z = 2 };
      enum class OrientationIdx { X = 0, Y = 1, Z = 2, W = 3 };

      rclcpp::Node::SharedPtr m_node;
      std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
      geometry_msgs::msg::TransformStamped::SharedPtr m_odom_msg;

  };  // class OdomProcessing
} // namespace LTM

#endif  // LTM_HARDWARE_INTERFACE__ODOM_PROCESSING_HPP_

// End of file: odom_processing.hpp