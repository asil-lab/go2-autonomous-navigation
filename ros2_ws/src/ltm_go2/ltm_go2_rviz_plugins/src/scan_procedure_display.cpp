/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 11-11-2024.
 */

#include <ltm_go2_rviz_plugins/scan_procedure_display.hpp>
#include <rviz_common/logging.hpp>

namespace LTM
{
  ScanProcedureDisplay::ScanProcedureDisplay(QWidget * parent)
  : rviz_common::Panel(parent)
  {
    m_scan_environment_value_palette.setColor(QPalette::WindowText, COLOR_GRAY);

    m_scan_environment_label = new QLabel("Not Scanning");
    m_scan_environment_label->setAlignment(Qt::AlignCenter);
    m_scan_environment_label->setStyleSheet("font-size: 36px; font-weight: bold;");
    m_scan_environment_label->setPalette(m_scan_environment_value_palette);

    QHBoxLayout * layout = new QHBoxLayout;
    layout->addWidget(m_scan_environment_label);
    setLayout(layout);

    // Create a ROS node and service
    m_node = rclcpp::Node::make_shared("scan_procedure_display");
    m_scan_environment_service = m_node->create_service<ltm_shared_msgs::srv::DisplayScanEnvironment>("/display_scan_environment", 
      std::bind(&ScanProcedureDisplay::displayScanEnvironmentCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    // Start a thread to update the scan environment
    m_thread = std::make_shared<std::thread>([this]() {
      rclcpp::spin(m_node);
    });

    RVIZ_COMMON_LOG_INFO_STREAM("Scan Procedure Display Plugin Loaded");
  }

  ScanProcedureDisplay::~ScanProcedureDisplay()
  {
    // Cleanly shutdown the node and join the thread
    rclcpp::shutdown();
    if (m_thread->joinable() && m_thread)
    {
      m_thread->join();
    }
  }

  void ScanProcedureDisplay::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
  }

  void ScanProcedureDisplay::load(const rviz_common::Config & config)
  {
    rviz_common::Panel::load(config);
  }

  void ScanProcedureDisplay::displayScanEnvironmentCallback(
    const std::shared_ptr<ltm_shared_msgs::srv::DisplayScanEnvironment::Request> request,
    std::shared_ptr<ltm_shared_msgs::srv::DisplayScanEnvironment::Response> response)
  {
    updateScanEnvironment(request->is_scanning);
    (void) response;
  }

  void ScanProcedureDisplay::updateScanEnvironment(bool is_scanning)
  {
    if (is_scanning)
    {
      m_scan_environment = "Scanning...";
      m_scan_environment_value_palette.setColor(QPalette::WindowText, COLOR_RED);
    }
    else
    {
      m_scan_environment = "Not Scanning";
      m_scan_environment_value_palette.setColor(QPalette::WindowText, COLOR_GRAY);
    }

    m_scan_environment_label->setText(QString::fromStdString(m_scan_environment));
    m_scan_environment_label->setPalette(m_scan_environment_value_palette);
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(LTM::ScanProcedureDisplay, rviz_common::Panel)