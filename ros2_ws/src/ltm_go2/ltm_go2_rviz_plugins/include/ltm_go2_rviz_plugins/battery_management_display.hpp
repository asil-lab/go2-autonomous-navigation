/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 21-10-2024.
 */

#ifndef LTM_GO2_RVIZ_PLUGINS__BATTERY_MANAGEMENT_DISPLAY_HPP_
#define LTM_GO2_RVIZ_PLUGINS__BATTERY_MANAGEMENT_DISPLAY_HPP_

#include <rviz_common/message_filter_display.hpp>
#include <ltm_shared_msgs/msg/battery_state.hpp>

namespace LTM
{
  class BatteryManagementDisplay : public rviz_common::MessageFilterDisplay<ltm_shared_msgs::msg::BatteryState>
  {
    Q_OBJECT

    protected:
      void processMessage(const ltm_shared_msgs::msg::BatteryState::ConstSharedPtr msg) override;

  }; // class BatteryManagementDisplay
}  // namespace LTM

#endif  // LTM_GO2_RVIZ_PLUGINS__BATTERY_MANAGEMENT_DISPLAY_HPP_

// End of file: ros2_ws/src/ltm_go2/ltm_go2_rviz_plugins/include/ltm_go2_rviz_plugins/battery_management_display.hpp