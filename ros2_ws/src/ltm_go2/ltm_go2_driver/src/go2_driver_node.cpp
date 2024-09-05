/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 13-08-2024.
 */

#ifndef LTM_GO2_DRIVER__GO2_DRIVER_NODE_CPP_
#define LTM_GO2_DRIVER__GO2_DRIVER_NODE_CPP_

#include <rclcpp/rclcpp.hpp>

#include <ltm_go2_driver/joint_state_processing.hpp>
#include <ltm_go2_driver/odom_processing.hpp>
#include <ltm_go2_driver/point_cloud_processing.hpp>
#include <ltm_go2_driver/wireless_controller_processing.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create nodes
  rclcpp::Node::SharedPtr joint_state_processing_node = std::make_shared<LTM::JointStateProcessing>();
  rclcpp::Node::SharedPtr odom_processing_node = std::make_shared<LTM::OdomProcessing>();
  rclcpp::Node::SharedPtr point_cloud_processing_node = std::make_shared<LTM::PointCloudProcessing>();
  rclcpp::Node::SharedPtr wireless_controller_processing_node = std::make_shared<LTM::WirelessControllerProcessing>();

  // Create executor, and add nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(joint_state_processing_node);
  executor.add_node(odom_processing_node);
  executor.add_node(point_cloud_processing_node);
  executor.add_node(wireless_controller_processing_node);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}

#endif  // LTM_GO2_DRIVER__GO2_DRIVER_NODE_CPP_

// End of file: ros2_ws/src/ltm_go2_driver/src/go2_driver_node.cpp