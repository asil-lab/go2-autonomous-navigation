/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 08-10-2024.
 */

#include <ltm_go2_driver/foot_contact_processing.hpp>

using namespace LTM;

FootContactProcessing::FootContactProcessing() : Node(FOOT_CONTACT_PROCESSING_NODE_NAME)
{
  initializeLowStateSubscriber();
  initializeFootContactPublisher();
}

FootContactProcessing::~FootContactProcessing()
{
  RCLCPP_WARN(get_logger(), "Destroying FootContactProcessing node");
}

void FootContactProcessing::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
  std::array<bool, NUMBER_OF_LEGS> foot_contact = compareFootForces(msg->foot_force);
  publishFootContact(foot_contact);
}

void FootContactProcessing::publishFootContact(const std::array<bool, NUMBER_OF_LEGS>& foot_contact) const
{
  ltm_shared_msgs::msg::FootContact msg;
  msg.front_right = foot_contact[FRONT_RIGHT_FOOT];
  msg.front_left = foot_contact[FRONT_LEFT_FOOT];
  msg.rear_right = foot_contact[REAR_RIGHT_FOOT];
  msg.rear_left = foot_contact[REAR_LEFT_FOOT];

  m_foot_contact_pub->publish(msg);
}

std::array<bool, NUMBER_OF_LEGS> FootContactProcessing::compareFootForces(const std::array<int16_t, NUMBER_OF_LEGS>& foot_forces) const
{
  std::array<bool, NUMBER_OF_LEGS> foot_contact;
  for (int i = 0; i < NUMBER_OF_LEGS; i++)
  {
    foot_contact[i] = isFootContact(foot_forces[i]);
  }
  return foot_contact;
}

bool FootContactProcessing::isFootContact(const uint16_t foot_force) const
{
  return foot_force >= FOOT_CONTACT_THRESHOLD;
}

void FootContactProcessing::initializeLowStateSubscriber()
{
  m_low_state_sub = create_subscription<unitree_go::msg::LowState>(
    FOOT_CONTACT_PROCESSING_SUB_LOW_STATE_TOPIC,
    FOOT_CONTACT_PROCESSING_SUB_LOW_STATE_QUEUE_SIZE,
    std::bind(&FootContactProcessing::lowStateCallback, this, std::placeholders::_1)
  );
}

void FootContactProcessing::initializeFootContactPublisher()
{
  m_foot_contact_pub = create_publisher<ltm_shared_msgs::msg::FootContact>(
    FOOT_CONTACT_PROCESSING_PUB_FOOT_CONTACT_TOPIC,
    FOOT_CONTACT_PROCESSING_PUB_FOOT_CONTACT_QUEUE_SIZE
  );
}