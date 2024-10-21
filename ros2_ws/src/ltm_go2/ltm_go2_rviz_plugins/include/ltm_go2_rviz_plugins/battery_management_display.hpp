/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 21-10-2024.
 */

#ifndef LTM_GO2_RVIZ_PLUGINS__BATTERY_MANAGEMENT_DISPLAY_HPP_
#define LTM_GO2_RVIZ_PLUGINS__BATTERY_MANAGEMENT_DISPLAY_HPP_

#include <rviz_common/message_filter_display.hpp>
#include <ltm_shared_msgs/msg/battery_state.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace LTM
{
  class BatteryManagementDisplay : public rviz_common::Panel
  {
    Q_OBJECT

  public:
    BatteryManagementDisplay(QWidget * parent = 0);
    ~BatteryManagementDisplay() override;
    
    virtual void save(rviz_common::Config config) const;
    virtual void load(const rviz_common::Config & config);

  public Q_SLOTS:
    
  private Q_SLOTS:
    void batteryStateCallback(const ltm_shared_msgs::msg::BatteryState::ConstSharedPtr msg);
    void updateBatteryState();

  protected:
    QLabel * m_battery_percentage_label;
    QLabel * m_battery_percentage_value;
    uint8_t m_battery_percentage;

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<ltm_shared_msgs::msg::BatteryState>::SharedPtr m_battery_state_sub;
    std::shared_ptr<std::thread> m_thread;

  }; // class BatteryManagementDisplay
}  // namespace LTM

#endif  // LTM_GO2_RVIZ_PLUGINS__BATTERY_MANAGEMENT_DISPLAY_HPP_

// End of file: ros2_ws/src/ltm_go2/ltm_go2_rviz_plugins/include/ltm_go2_rviz_plugins/battery_management_display.hpp