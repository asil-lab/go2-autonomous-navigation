/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 13-08-2024.
 */

#ifndef LTM_HARDWARE_INTERFACE__GO2_DRIVER_NODE_CPP_
#define LTM_HARDWARE_INTERFACE__GO2_DRIVER_NODE_CPP_

#include <rclcpp/rclcpp.hpp>

#include <ltm_hardware_interface/joint_state_processing.hpp>
#include <ltm_hardware_interface/odom_processing.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create nodes
  rclcpp::Node::SharedPtr joint_state_processing_node = std::make_shared<LTM::JointStateProcessing>();
  rclcpp::Node::SharedPtr odom_processing_node = std::make_shared<LTM::OdomProcessing>();

  // Create executor, and add nodes
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(joint_state_processing_node);
  executor.add_node(odom_processing_node);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}

#endif  // LTM_HARDWARE_INTERFACE__GO2_DRIVER_NODE_CPP_

// End of file: ros2_ws/src/ltm_hardware_interface/src/go2_driver_node.cpp