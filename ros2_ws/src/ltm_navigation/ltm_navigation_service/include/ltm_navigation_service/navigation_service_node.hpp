/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 09-09-2024.
 */

#ifndef LTM_NAVIGATION_SERVICE__NAVIGATION_SERVICE_NODE_HPP_
#define LTM_NAVIGATION_SERVICE__NAVIGATION_SERVICE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ltm_shared_msgs/srv/navigate_to_pose.hpp>

#include <string>
#include <vector>
#include <chrono>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define POSE_EIGEN_VECTOR_SIZE 7
#define POSE_EIGEN_VECTOR_POSITION_OFFSET 0
#define POSE_EIGEN_VECTOR_ORIENTATION_OFFSET 3
#define POSE_EIGEN_VECTOR_POSITION_SIZE 3
#define POSE_EIGEN_VECTOR_ORIENTATION_SIZE 4

namespace lTM
{
  class NavigationServiceNode : public rclcpp::Node
  {
    public:
      NavigationServiceNode();
      ~NavigationServiceNode();

    private:
      void navigateToPoseCallback(
        const std::shared_ptr<ltm_shared_msgs::srv::NavigateToPose::Request> request,
        std::shared_ptr<ltm_shared_msgs::srv::NavigateToPose::Response> response);
      void publishGoalPose(const geometry_msgs::msg::PoseStamped& goal_pose) const;

      geometry_msgs::msg::PoseStamped getCurrentRobotPose();
      Eigen::VectorXd convertPoseToEigen(geometry_msgs::msg::Pose pose);

      double computeTranslationError(const geometry_msgs::msg::Point& p1, 
        const geometry_msgs::msg::Point& p2);
      double computeOrientationError(const geometry_msgs::msg::Quaternion& q1, 
        const geometry_msgs::msg::Quaternion& q2);

      void initializeService();
      void initializeRosTopic();
      void initializeTfListener();

      std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
      std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

      double m_navigate_to_pose_timeout;
      double m_navigate_to_pose_update_period;
      double m_navigate_to_pose_position_tolerance;
      double m_navigate_to_pose_orientation_tolerance;
      std::string m_robot_source_frame_name;
      std::string m_robot_target_frame_name;

      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_publisher;
      rclcpp::Service<ltm_shared_msgs::srv::NavigateToPose>::SharedPtr m_navigation_to_pose_service;

  }; // class NavigationServiceNode
} // namespace lTM

#endif // LTM_NAVIGATION_SERVICE__NAVIGATION_SERVICE_NODE_HPP_

// End of file: ltm_navigation_service/include/ltm_navigation_service/navigation_service_node.hpp