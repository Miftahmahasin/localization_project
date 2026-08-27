#include "open_cr_module/open_cr_module.h"
#include <rclcpp/rclcpp.hpp>
#include <robotis_framework_common/sensor_module.h>

using namespace robotis_op;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Create instance of OpenCRModule
  auto node = OpenCRModule::getInstance();
  
  // Initialize with a default control cycle in milliseconds
  int control_cycle_ms = 8;
  node->initialize(control_cycle_ms, nullptr);
  
  RCLCPP_INFO(node->get_logger(), "OpenCR Module node started");
  
  // Tunggu hingga di-interrupt (Ctrl+C)
  rclcpp::WallRate rate(1);  // 1 Hz
  while (rclcpp::ok()) {
    rate.sleep();
  }
  
  rclcpp::shutdown();
  return 0;
}