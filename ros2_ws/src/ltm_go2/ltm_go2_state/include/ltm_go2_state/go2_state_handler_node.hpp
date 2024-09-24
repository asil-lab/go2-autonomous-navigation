/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 23-09-2024.
 */

#ifndef LTM_GO2_STATE__GO2_STATE_HANDLER_NODE_HPP_
#define LTM_GO2_STATE__GO2_STATE_HANDLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <ltm_shared_msgs/srv/perform_state.hpp>

// TODO: Parameterize these values
#define GESTURE_PUBLISHER_TOPIC "gesture"
#define GESTURE_PUBLISHER_QUEUE_SIZE 1

#define GESTURE_STAND_UP    "stand_up"
#define GESTURE_STAND_DOWN  "stand_down"
#define GESTURE_RECOVER     "recover"
#define GESTURE_DELAY       10 // seconds

#define BOOTUP_SERVICE_NAME   "state_machine/boot_up"
#define SHUTDOWN_SERVICE_NAME "state_machine/shutdown"

namespace LTM
{
  class Go2StateHandlerNode : public rclcpp::Node
  {
  public:
    Go2StateHandlerNode();
    ~Go2StateHandlerNode();

  private:
    void bootupCallback(
      const std::shared_ptr<ltm_shared_msgs::srv::PerformState::Request> request,
      std::shared_ptr<ltm_shared_msgs::srv::PerformState::Response> response);
    void shutdownCallback(
      const std::shared_ptr<ltm_shared_msgs::srv::PerformState::Request> request,
      std::shared_ptr<ltm_shared_msgs::srv::PerformState::Response> response);

    void publishGesture(const std::string & gesture);
    void sleepFor(const int & seconds);

    void configureBootupService();
    void configureShutdownService();
    void configureGesturePublisher();

    rclcpp::Service<ltm_shared_msgs::srv::PerformState>::SharedPtr m_bootup_service;
    rclcpp::Service<ltm_shared_msgs::srv::PerformState>::SharedPtr m_shutdown_service;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_gesture_publisher;

  };  // class Go2StateHandlerNode
}  // namespace LTM

#endif  // LTM_GO2_STATE__GO2_STATE_HANDLER_NODE_HPP_

// End of file: ros2_ws/src/ltm_go2/ltm_go2_state/include/go2_state_handler_node.hpp