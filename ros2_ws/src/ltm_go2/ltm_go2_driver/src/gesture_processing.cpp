/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 05-09-2024.
 */

#include <ltm_go2_driver/gesture_processing.hpp>

using namespace LTM;

GestureProcessing::GestureProcessing() : Node(GESTURE_PROCESSING_NODE_NAME)
{
    initializeROS();
    initializeGestureMap();
    RCLCPP_INFO(this->get_logger(), "Gesture Processing Node initialized.");
}

GestureProcessing::~GestureProcessing()
{
    m_gesture_sub.reset();
    m_request_pub.reset();
    RCLCPP_WARN(this->get_logger(), "Gesture Processing Node destroyed.");
}

void GestureProcessing::gestureCallback(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "Received Gesture Message: %s", msg->data.c_str());
    try {
        uint64_t gesture = m_gesture_map.at(msg->data);
        publishRequest(gesture);
    } catch (const std::out_of_range& e) {
        RCLCPP_ERROR(this->get_logger(), "Gesture not found: %s", msg->data.c_str());
    }
}

void GestureProcessing::publishRequest(const uint64_t& gesture_api_id) const
{
    auto request_msg = unitree_api::msg::Request();
    request_msg.header.identity.api_id = gesture_api_id;
    m_request_pub->publish(request_msg);
}

void GestureProcessing::initializeROS()
{
    // Initialize the ROS publishers and subscribers
    m_gesture_sub = this->create_subscription<std_msgs::msg::String>(
        GESTURE_PROCESSING_SUB_TOPIC, GESTURE_PROCESSING_SUB_QUEUE_SIZE,
        std::bind(&GestureProcessing::gestureCallback, this, std::placeholders::_1));

    m_request_pub = this->create_publisher<unitree_api::msg::Request>(
        GESTURE_PROCESSING_PUB_TOPIC, GESTURE_PROCESSING_SUB_QUEUE_SIZE);
}

void GestureProcessing::initializeGestureMap()
{
    m_gesture_map[GESTURE_STAND_UP_KEY] = GESTURE_STAND_UP_API_ID;
    m_gesture_map[GESTURE_STAND_DOWN_KEY] = GESTURE_STAND_DOWN_API_ID;
    m_gesture_map[GESTURE_RECOVERY_KEY] = GESTURE_RECOVERY_API_ID;
    m_gesture_map[GESTURE_SIT_DOWN_KEY] = GESTURE_SIT_DOWN_API_ID;
    RCLCPP_INFO(this->get_logger(), "Gesture Map initialized. There are %d gestures.", m_gesture_map.size());
}

// End of file: ros2_ws/src/ltm_go2/ltm_go2_driver/src/gesture_processing.cpp