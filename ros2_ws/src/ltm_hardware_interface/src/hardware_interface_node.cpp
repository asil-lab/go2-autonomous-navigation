/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 01-08-2024.
 */

#include <ltm_hardware_interface/hardware_interface_node.hpp>

using namespace LTM;

HardwareInterfaceNode::HardwareInterfaceNode()
: Node("hardware_interface_node")
{
  // Initialize processing classes
  m_front_camera_processing = std::make_shared<FrontCameraProcessing>();
  m_joint_state_processing = std::make_shared<JointStateProcessing>();
  m_odom_processing = std::make_shared<OdomProcessing>(this->shared_from_this());

  // Create subscriptions
  m_low_state_sub = this->create_subscription<unitree_go::msg::LowState>(
    "lowstate", 10, std::bind(&HardwareInterfaceNode::lowStateCallback, this, std::placeholders::_1));
  m_sport_mode_state_sub = this->create_subscription<unitree_go::msg::SportModeState>(
    "sportmodestate", 10, std::bind(&HardwareInterfaceNode::sportModeStateCallback, this, std::placeholders::_1));
  m_point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "utlidar/cloud", 10, std::bind(&HardwareInterfaceNode::pointCloudCallback, this, std::placeholders::_1));
  m_front_video_sub = this->create_subscription<sensor_msgs::msg::Image>(
    "front_camera/image_raw", 10, std::bind(&HardwareInterfaceNode::frontVideoCallback, this, std::placeholders::_1));

  // Create publishers
  m_joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  m_point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud/raw", 10);
  m_front_video_180p_pub = this->create_publisher<sensor_msgs::msg::Image>("front_camera/180p", 10);
  m_front_video_360p_pub = this->create_publisher<sensor_msgs::msg::Image>("front_camera/360p", 10);
  m_front_video_720p_pub = this->create_publisher<sensor_msgs::msg::Image>("front_camera/720p", 10);

  RCLCPP_INFO(this->get_logger(), "LTM Hardware Interface Node initialized.");
}

HardwareInterfaceNode::~HardwareInterfaceNode()
{
  // Reset ROS subscriptions
  m_low_state_sub.reset();
  m_sport_mode_state_sub.reset();
  m_point_cloud_sub.reset();
  m_front_video_sub.reset();

  // Reset ROS publishers
  m_joint_state_pub.reset();
  m_point_cloud_pub.reset();
  m_front_video_180p_pub.reset();
  m_front_video_360p_pub.reset();
  m_front_video_720p_pub.reset();

  // Reset processing classes
  m_front_camera_processing.reset();
  m_joint_state_processing.reset();
  m_odom_processing.reset();

  RCLCPP_WARN(this->get_logger(), "LTM Hardware Interface Node shutting down.");
}

void HardwareInterfaceNode::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
  // Update the joint state message from the motor state
  m_joint_state_processing->updateJointStateMsg(msg->motor_state);
  m_joint_state_pub->publish(*(m_joint_state_processing->getJointStateMsg()));
}

void HardwareInterfaceNode::sportModeStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg)
{
  // Update the odometry transform from base to world
  m_odom_processing->updateOdomTranslation(msg->position);
  m_odom_processing->updateOdomOrientation(msg->imu_state.quaternion);
  m_odom_processing->broadcastOdomTransform();
}

void HardwareInterfaceNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Change the frame_id of the point cloud message to "radar"
  sensor_msgs::msg::PointCloud2 point_cloud_msg = *msg;
  point_cloud_msg.header.frame_id = "radar";
  m_point_cloud_pub->publish(point_cloud_msg);
}

void HardwareInterfaceNode::frontVideoCallback(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg)
{
  // Publish the front camera video messages in 180p, 360p, and 720p
  m_front_camera_processing->updateFrontCameraMsgs(msg, this->now());

  m_front_video_180p_pub->publish(*(m_front_camera_processing->getFrontCamera180pMsg()));
  m_front_video_360p_pub->publish(*(m_front_camera_processing->getFrontCamera360pMsg()));
  m_front_video_720p_pub->publish(*(m_front_camera_processing->getFrontCamera720pMsg()));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}

// End of file: hardware_interface_node.cpp