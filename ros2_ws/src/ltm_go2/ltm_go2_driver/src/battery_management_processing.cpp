/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 18-10-2024.
 */

#include <ltm_go2_driver/battery_management_processing.hpp>

using namespace LTM;

BatteryManagementProcessing::BatteryManagementProcessing() : Node(BATTERY_MANAGEMENT_PROCESSING_NODE_NAME)
{
  initializeBatteryStatePublisher();
  initializeLowStateSubscriber();
  RCLCPP_INFO(get_logger(), "Battery Management Processing Node has been initialized.");
}

BatteryManagementProcessing::~BatteryManagementProcessing()
{
  RCLCPP_WARN(get_logger(), "Battery Management Processing Node has been terminated.");
}

void BatteryManagementProcessing::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 10000, "Received LowState message: %d", msg->bms_state.soc);
  publishBatteryState(msg->bms_state.soc);
}

void BatteryManagementProcessing::publishBatteryState(uint8_t percentage)
{
  ltm_shared_msgs::msg::BatteryState battery_state;
  battery_state.percentage = percentage;
  m_battery_state_pub->publish(battery_state);
}

void BatteryManagementProcessing::initializeLowStateSubscriber()
{
  m_low_state_sub = create_subscription<unitree_go::msg::LowState>(
    BATTERY_MANAGEMENT_PROCESSING_SUB_LOW_STATE_TOPIC,
    BATTERY_MANAGEMENT_PROCESSING_SUB_LOW_STATE_QUEUE_SIZE,
    std::bind(&BatteryManagementProcessing::lowStateCallback, this, std::placeholders::_1)
  );
}

void BatteryManagementProcessing::initializeBatteryStatePublisher()
{
  m_battery_state_pub = create_publisher<ltm_shared_msgs::msg::BatteryState>(
    BATTERY_MANAGEMENT_PROCESSING_PUB_BATTERY_STATUS_TOPIC,
    BATTERY_MANAGEMENT_PROCESSING_PUB_BATTERY_STATUS_QUEUE_SIZE);
}
