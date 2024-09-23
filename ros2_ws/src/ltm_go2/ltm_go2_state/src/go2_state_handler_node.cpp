/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 23-09-2024.
 */

#include <ltm_go2_state/go2_state_handler_node.hpp>

using namespace LTM;

#include <chrono>
#include <thread>

Go2StateHandlerNode::Go2StateHandlerNode()
: Node("go2_state_handler_node")
{
  RCLCPP_INFO(get_logger(), "Initializing Go2StateHandlerNode...");
  configureBootupService();
  configureShutdownService();
  configureGesturePublisher();
  RCLCPP_INFO(get_logger(), "Go2StateHandlerNode initialized");
}

Go2StateHandlerNode::~Go2StateHandlerNode()
{
  RCLCPP_WARN(get_logger(), "Destroying Go2StateHandlerNode");
}

void Go2StateHandlerNode::bootupCallback(
  const std::shared_ptr<ltm_shared_msgs::srv::PerformState::Request> request,
  std::shared_ptr<ltm_shared_msgs::srv::PerformState::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received bootup request");

  // If this is not the first time, do not publish gesture
  if (!request->first_time) {
    return;
  }
  
  publishGesture(GESTURE_STAND_UP);
  publishGesture(GESTURE_RECOVER);
  response->success = true;
}

void Go2StateHandlerNode::shutdownCallback(
  const std::shared_ptr<ltm_shared_msgs::srv::PerformState::Request> request,
  std::shared_ptr<ltm_shared_msgs::srv::PerformState::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received shutdown request");

  // If this is not the first time, do not publish gesture
  if (!request->first_time) {
    return;
  }

  publishGesture(GESTURE_STAND_DOWN);
  response->success = true;
}

void Go2StateHandlerNode::publishGesture(const std::string & gesture)
{
  std_msgs::msg::String::SharedPtr msg = std::make_shared<std_msgs::msg::String>();
  msg->data = gesture;
  m_gesture_publisher->publish(*msg);
  sleepFor(GESTURE_DELAY);
}

void Go2StateHandlerNode::sleepFor(const int & seconds)
{
  std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

void Go2StateHandlerNode::configureBootupService()
{
  m_bootup_service = create_service<ltm_shared_msgs::srv::PerformState>(
    BOOTUP_SERVICE_NAME,
    std::bind(&Go2StateHandlerNode::bootupCallback, this, 
    std::placeholders::_1, std::placeholders::_2));
}

void Go2StateHandlerNode::configureShutdownService()
{
  m_shutdown_service = create_service<ltm_shared_msgs::srv::PerformState>(
    SHUTDOWN_SERVICE_NAME,
    std::bind(&Go2StateHandlerNode::shutdownCallback, this, 
    std::placeholders::_1, std::placeholders::_2));
}

void Go2StateHandlerNode::configureGesturePublisher()
{
  m_gesture_publisher = create_publisher<std_msgs::msg::String>(
    GESTURE_PUBLISHER_TOPIC, GESTURE_PUBLISHER_QUEUE_SIZE);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2StateHandlerNode>());
  rclcpp::shutdown();
  return 0;
}