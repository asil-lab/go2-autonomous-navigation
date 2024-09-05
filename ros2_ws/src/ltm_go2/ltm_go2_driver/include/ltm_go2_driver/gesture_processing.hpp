/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 05-09-2024.
 */

#ifndef LTM_GO2_DRIVER__GESTURE_PROCESSING_HPP_
#define LTM_GO2_DRIVER__GESTURE_PROCESSING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <unitree_api/msg/request.hpp>

#include <string>
#include <unordered_map>

#define GESTURE_PROCESSING_NODE_NAME "gesture_processing_node"
#define GESTURE_PROCESSING_SUB_TOPIC "gesture"
#define GESTURE_PROCESSING_SUB_QUEUE_SIZE 10
#define GESTURE_PROCESSING_PUB_TOPIC "api/sport/request"

#define GESTURE_STAND_UP_KEY    "stand_up"
#define GESTURE_STAND_DOWN_KEY  "stand_down"
#define GESTURE_RECOVERY_KEY    "recover"
#define GESTURE_SIT_DOWN_KEY    "sit"

#define GESTURE_STAND_UP_API_ID   1004
#define GESTURE_STAND_DOWN_API_ID 1005
#define GESTURE_RECOVERY_API_ID   1006
#define GESTURE_SIT_DOWN_API_ID   1009

namespace LTM
{
  class GestureProcessing : public rclcpp::Node
  {
    public:
      GestureProcessing();
      ~GestureProcessing();

    private:
      void gestureCallback(const std_msgs::msg::String::SharedPtr msg) const;
      void publishRequest(const uint64_t& gesture_api_id) const;

      void initializeROS();
      void initializeGestureMap();

      std::unordered_map<std::string, uint64_t> m_gesture_map;

      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_gesture_sub;
      rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr m_request_pub;

  }; // class GestureProcessing
} // namespace LTM

#endif  // LTM_GO2_DRIVER__GESTURE_PROCESSING_HPP_

// End of file: ros2_ws/src/ltm_go2/ltm_go2_driver/include/ltm_go2_driver/gesture_processing.hpp