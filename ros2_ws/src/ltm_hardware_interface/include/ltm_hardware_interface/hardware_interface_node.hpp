/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 01-08-2024.
 */

#ifndef LTM_HARDWARE_INTERFACE__HARDWARE_INTERFACE_NODE_HPP_
#define LTM_HARDWARE_INTERFACE__HARDWARE_INTERFACE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unitree_go/msg/go2_front_video_data.hpp>
#include <unitree_go/msg/imu_state.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include <vector>
#include <string>

#include <tf2_ros/transform_broadcaster.h>

namespace LTM
{
  #define MOTOR_SIZE 20

  class HardwareInterfaceNode : public rclcpp::Node
  {
  public:
    HardwareInterfaceNode();
    ~HardwareInterfaceNode();

  private:
    void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
    void sportModeStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void frontVideoCallback(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg);

    void updateJointStateMsg(const std::array<unitree_go::msg::MotorState, MOTOR_SIZE>& motor_state);
    void publishJointState();
    void initializeJointStateMsg();

    void updateWorldToBaseTranslation(const std::array<float, 3>& translation);
    void updateWorldToBaseOrientation(const std::array<float, 4>& orientation);
    void broadcastWorldToBaseTransform();
    void initializeWorldToBaseTransformMsg();

    sensor_msgs::msg::JointState::SharedPtr m_joint_state_msg;
    std::vector<int> m_joint_idx;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    geometry_msgs::msg::TransformStamped::SharedPtr m_world_to_base_transform_msg;

    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr m_low_state_sub;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr m_sport_mode_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_point_cloud_sub;
    rclcpp::Subscription<unitree_go::msg::Go2FrontVideoData>::SharedPtr m_front_video_sub;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_point_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_front_video_720p_pub;

  }; // class HardwareInterfaceNode
} // namespace LTM

#endif // LTM_HARDWARE_INTERFACE__HARDWARE_INTERFACE_NODE_HPP_