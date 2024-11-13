/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 11-11-2024.
 */

#ifndef LTM_GO2_RVIZ_PLUGINS__SCAN_PROCEDURE_DISPLAY_HPP_
#define LTM_GO2_RVIZ_PLUGINS__SCAN_PROCEDURE_DISPLAY_HPP_

#include <rviz_common/message_filter_display.hpp>
#include <ltm_shared_msgs/srv/display_scan_environment.hpp>

#include <QHBoxLayout>
#include <QLabel>
#include <QPalette>
#include <QColor>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace LTM
{
  class ScanProcedureDisplay : public rviz_common::Panel
  {
    Q_OBJECT

  public:
    ScanProcedureDisplay(QWidget * parent = 0);
    ~ScanProcedureDisplay() override;
    
    virtual void save(rviz_common::Config config) const;
    virtual void load(const rviz_common::Config & config);

  public Q_SLOTS:

  private Q_SLOTS:
    void displayScanEnvironmentCallback(
      const std::shared_ptr<ltm_shared_msgs::srv::DisplayScanEnvironment::Request> request,
      std::shared_ptr<ltm_shared_msgs::srv::DisplayScanEnvironment::Response> response);
    void updateScanEnvironment(bool is_scanning, bool is_moving);

  protected:
    QLabel * m_scan_environment_label;
    QPalette m_scan_environment_value_palette;
    std::string m_scan_environment;

    const double SCAN_ENVIRONMENT_UPDATE_PERIOD = 1.0;
    const QColor COLOR_GRAY = QColor(128, 128, 128);
    const QColor COLOR_RED = QColor(255, 0, 0);
    const QColor COLOR_GREEN = QColor(0, 128, 0);
    const QColor COLOR_BLUE = QColor(30, 144, 255);

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Service<ltm_shared_msgs::srv::DisplayScanEnvironment>::SharedPtr m_scan_environment_service;
    std::shared_ptr<std::thread> m_thread;

  }; // class ScanProcedureDisplay
}  // namespace LTM

#endif  // LTM_GO2_RVIZ_PLUGINS__SCAN_PROCEDURE_DISPLAY_HPP_
