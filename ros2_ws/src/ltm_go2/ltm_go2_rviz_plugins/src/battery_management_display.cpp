/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 21-10-2024.
 */

#include <ltm_go2_rviz_plugins/battery_management_display.hpp>
#include <rviz_common/logging.hpp>

namespace LTM
{
  void BatteryManagementDisplay::processMessage(const ltm_shared_msgs::msg::BatteryState::ConstSharedPtr msg)
  {
    RVIZ_COMMON_LOG_INFO_STREAM("BatteryManagementDisplay::processMessage: " << msg->percentage);
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(LTM::BatteryManagementDisplay, rviz_common::Display)