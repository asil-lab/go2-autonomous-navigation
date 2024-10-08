/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 08-10-2024.
 */

#ifndef LTM_GO2_DRIVER__FOOT_CONTACT_PROCESSING_HPP_
#define LTM_GO2_DRIVER__FOOT_CONTACT_PROCESSING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <unitree_go/msg/low_state.hpp>
#include <ltm_shared_msgs/msg/foot_contact.hpp>

#include <string>
#include <array>
#include <functional>

#define FOOT_CONTACT_PROCESSING_NODE_NAME "foot_contact_processing_node"

#define FOOT_CONTACT_PROCESSING_SUB_LOW_STATE_TOPIC "lowstate"
#define FOOT_CONTACT_PROCESSING_SUB_LOW_STATE_QUEUE_SIZE 10

#define FOOT_CONTACT_PROCESSING_PUB_FOOT_CONTACT_TOPIC "foot_contact"
#define FOOT_CONTACT_PROCESSING_PUB_FOOT_CONTACT_QUEUE_SIZE 10

#define FOOT_CONTACT_THRESHOLD 18 // Unknown unit
#define NUMBER_OF_LEGS 4

#define FRONT_RIGHT_FOOT  0
#define FRONT_LEFT_FOOT   1
#define REAR_RIGHT_FOOT   2
#define REAR_LEFT_FOOT    3

namespace LTM
{
  class FootContactProcessing : public rclcpp::Node
  {
  public:
    FootContactProcessing();
    ~FootContactProcessing();

  private:
    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
    void publishFootContact(const std::array<bool, NUMBER_OF_LEGS>& foot_contact) const;

    std::array<bool, NUMBER_OF_LEGS> compareFootForces(const std::array<int16_t, NUMBER_OF_LEGS>& foot_forces) const;
    bool isFootContact(const uint16_t foot_force) const;

    void initializeLowStateSubscriber();
    void initializeFootContactPublisher();

    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr m_low_state_sub;
    rclcpp::Publisher<ltm_shared_msgs::msg::FootContact>::SharedPtr m_foot_contact_pub;

  }; // class FootContactProcessing
} // namespace LTM

#endif // LTM_GO2_DRIVER__FOOT_CONTACT_PROCESSING_HPP_

// End of file: ros2_ws/src/ltm_go2/ltm_go2_driver/include/ltm_go2_driver/foot_contact_processing.hpp