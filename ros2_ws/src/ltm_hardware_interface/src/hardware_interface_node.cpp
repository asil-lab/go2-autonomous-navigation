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
  m_front_camera = std::make_shared<FrontCamera>("front_camera");

  // Initialize the messages
  initializeJointStateMsg();
  initializeWorldToBaseTransformMsg();

  // Initialize the transform broadcaster
  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

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
  // Reset ROS subscriptions and publishers
  m_low_state_sub.reset();
  m_joint_state_pub.reset();

  // Reset msg pointers
  m_joint_state_msg.reset();
  m_world_to_base_transform_msg.reset();

  RCLCPP_WARN(this->get_logger(), "LTM Hardware Interface Node shutting down.");
}

void HardwareInterfaceNode::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
  updateJointStateMsg(msg->motor_state);
  publishJointState();
}

void HardwareInterfaceNode::sportModeStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg)
{
  updateWorldToBaseTranslation(msg->position);
  updateWorldToBaseOrientation(msg->imu_state.quaternion);
  broadcastWorldToBaseTransform();
}

void HardwareInterfaceNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 point_cloud_msg = *msg;
  point_cloud_msg.header.frame_id = "radar";
  m_point_cloud_pub->publish(point_cloud_msg);
}

void HardwareInterfaceNode::frontVideoCallback(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg)
{
  m_front_camera->updateFrontCameraMsgs(msg, this->now());

  m_front_video_180p_pub->publish(*(m_front_camera->getFrontCamera180pMsg()));
  m_front_video_360p_pub->publish(*(m_front_camera->getFrontCamera360pMsg()));
  m_front_video_720p_pub->publish(*(m_front_camera->getFrontCamera720pMsg()));
}

void HardwareInterfaceNode::updateJointStateMsg(const std::array<unitree_go::msg::MotorState, MOTOR_SIZE>& motor_state)
{
  for (unsigned long int i = 0; i < m_joint_idx.size(); i++)
  {
    m_joint_state_msg->position[i] = motor_state[m_joint_idx[i]].q;
    m_joint_state_msg->velocity[i] = motor_state[m_joint_idx[i]].dq;
    m_joint_state_msg->effort[i] = motor_state[m_joint_idx[i]].tau_est;
  }
}

void HardwareInterfaceNode::publishJointState()
{
  m_joint_state_msg->header.stamp = this->now();
  m_joint_state_pub->publish(*m_joint_state_msg);
}

void HardwareInterfaceNode::initializeJointStateMsg()
{
  // Initialize the joint state message
  m_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

  // Set the joint state message header
  m_joint_state_msg->header.stamp = this->now();
  m_joint_state_msg->header.frame_id = "base_link";

  // Set the joint state message names
  m_joint_state_msg->name = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", 
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

  // Set zero values for the joint state message
  m_joint_state_msg->position = std::vector<double>(12, 0.0);
  m_joint_state_msg->velocity = std::vector<double>(12, 0.0);
  m_joint_state_msg->effort = std::vector<double>(12, 0.0);

  // Set the joint state message indices
  m_joint_idx = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8}; // TODO: Parameterize this, use enum?
}

void HardwareInterfaceNode::updateWorldToBaseTranslation(const std::array<float, 3>& translation)
{
  m_world_to_base_transform_msg->transform.translation.x = translation[0];
  m_world_to_base_transform_msg->transform.translation.y = translation[1];
  m_world_to_base_transform_msg->transform.translation.z = translation[2];
}

void HardwareInterfaceNode::updateWorldToBaseOrientation(const std::array<float, 4>& orientation)
{
  m_world_to_base_transform_msg->transform.rotation.w = orientation[0];
  m_world_to_base_transform_msg->transform.rotation.x = orientation[1];
  m_world_to_base_transform_msg->transform.rotation.y = orientation[2];
  m_world_to_base_transform_msg->transform.rotation.z = orientation[3];
}

void HardwareInterfaceNode::broadcastWorldToBaseTransform()
{
  m_world_to_base_transform_msg->header.stamp = this->now();
  m_tf_broadcaster->sendTransform(*m_world_to_base_transform_msg);
}

void HardwareInterfaceNode::initializeWorldToBaseTransformMsg()
{
  // Initialize the IMU transform message
  m_world_to_base_transform_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();

  // Set the parent and child frames
  m_world_to_base_transform_msg->header.frame_id = "world";
  m_world_to_base_transform_msg->child_frame_id = "base";

  // Set the transform values to zero
  m_world_to_base_transform_msg->transform.translation.x = 0.0;
  m_world_to_base_transform_msg->transform.translation.y = 0.0;
  m_world_to_base_transform_msg->transform.translation.z = 0.0;

  m_world_to_base_transform_msg->transform.rotation.w = 0.0;
  m_world_to_base_transform_msg->transform.rotation.x = 0.0;
  m_world_to_base_transform_msg->transform.rotation.y = 0.0;
  m_world_to_base_transform_msg->transform.rotation.z = 0.0;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}