/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 18-10-2024.
 */

#ifndef LTM_GO2_DRIVER__BATTERY_MANAGEMENT_PROCESSING_HPP_
#define LTM_GO2_DRIVER__BATTERY_MANAGEMENT_PROCESSING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <unitree_go/msg/low_state.hpp>
#include <ltm_shared_msgs/msg/battery_state.hpp>

#include <string>

#define BATTERY_MANAGEMENT_PROCESSING_NODE_NAME "battery_management_processing_node"

#define BATTERY_MANAGEMENT_PROCESSING_SUB_LOW_STATE_TOPIC "lowstate"
#define BATTERY_MANAGEMENT_PROCESSING_SUB_LOW_STATE_QUEUE_SIZE 10

#define BATTERY_MANAGEMENT_PROCESSING_PUB_BATTERY_STATUS_TOPIC "battery_state"
#define BATTERY_MANAGEMENT_PROCESSING_PUB_BATTERY_STATUS_QUEUE_SIZE 10

#define BATTERY_RANGE_MIN 0
#define BATTERY_RANGE_MAX 100

namespace LTM
{
  class BatteryManagementProcessing : public rclcpp::Node
  {
  public:
    BatteryManagementProcessing();
    ~BatteryManagementProcessing();

  private:
    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
    void publishBatteryState(uint8_t percentage);

    void initializeLowStateSubscriber();
    void initializeBatteryStatePublisher();

    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr m_low_state_sub;
    rclcpp::Publisher<ltm_shared_msgs::msg::BatteryState>::SharedPtr m_battery_state_pub;

  }; // class BatteryManagementProcessing
} // namespace LTM

#endif  // LTM_GO2_DRIVER__BATTERY_MANAGEMENT_PROCESSING_HPP_
