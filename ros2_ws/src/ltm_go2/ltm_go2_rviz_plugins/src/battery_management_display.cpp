/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 21-10-2024.
 */

#include <ltm_go2_rviz_plugins/battery_management_display.hpp>
#include <rviz_common/logging.hpp>

namespace LTM
{
  BatteryManagementDisplay::BatteryManagementDisplay(QWidget * parent)
  : rviz_common::Panel(parent)
  {
    m_battery_percentage_label = new QLabel("Battery Percentage: 0%");
    QVBoxLayout * layout = new QVBoxLayout;
    layout->addWidget(m_battery_percentage_label);
    setLayout(layout);

    m_node = rclcpp::Node::make_shared("battery_management_display");
    m_battery_state_sub = m_node->create_subscription<ltm_shared_msgs::msg::BatteryState>(
      "battery_state", 10, std::bind(&BatteryManagementDisplay::batteryStateCallback, this, std::placeholders::_1));
  }

  void BatteryManagementDisplay::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
    config.mapSetValue("BatteryPercentage", QString::number(m_battery_percentage));
  }

  void BatteryManagementDisplay::load(const rviz_common::Config & config)
  {
    rviz_common::Panel::load(config);
    QString battery_percentage;
    if (config.mapGetString("BatteryPercentage", &battery_percentage))
    {
      m_battery_percentage = battery_percentage.toUInt();
      updateBatteryState();
    }
  }

  void BatteryManagementDisplay::batteryStateCallback(const ltm_shared_msgs::msg::BatteryState::ConstSharedPtr msg)
  {
    m_battery_percentage = msg->percentage;
    updateBatteryState();
  }

  void BatteryManagementDisplay::updateBatteryState()
  {
    m_battery_percentage_label->setText("Battery Percentage: " + QString::number(m_battery_percentage) + "%");
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(LTM::BatteryManagementDisplay, rviz_common::Panel)