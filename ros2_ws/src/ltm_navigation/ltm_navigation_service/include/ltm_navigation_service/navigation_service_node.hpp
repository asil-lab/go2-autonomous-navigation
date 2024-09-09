/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 09-09-2024.
 */

#ifndef LTM_NAVIGATION_SERVICE__NAVIGATION_SERVICE_NODE_HPP_
#define LTM_NAVIGATION_SERVICE__NAVIGATION_SERVICE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>
#include <vector>

namespace lTM
{
  class NavigationServiceNode : public rclcpp::Node
  {
    public:
      NavigationServiceNode();
      ~NavigationServiceNode();

    private:
      void navigationServiceCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<geometry_msgs::msg::PoseStamped> request,
        std::shared_ptr<geometry_msgs::msg::PoseStamped> response);

      void initializeService();

      void initializeROS2Topics();

      rclcpp::Service<geometry_msgs::msg::PoseStamped>::SharedPtr m_navigation_service;
      std::string m_service_name;

  }; // class NavigationServiceNode
} // namespace lTM

#endif // LTM_NAVIGATION_SERVICE__NAVIGATION_SERVICE_NODE_HPP_

// End of file: ltm_navigation_service/include/ltm_navigation_service/navigation_service_node.hpp