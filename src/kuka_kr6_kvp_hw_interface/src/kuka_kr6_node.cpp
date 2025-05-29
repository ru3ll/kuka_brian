#include "rclcpp/rclcpp.hpp"
#include "kuka_kr6_kvp_hw_interface/kuka_kr6_system.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

using hardware_interface::CallbackReturn;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rclcpp::Node>("test_kvp_node", options);

  RCLCPP_INFO(node->get_logger(), "Starting test for KUKA hardware interface");

  // Create instance of your hardware interface
  auto hw_interface = std::make_shared<kuka_kr6_kvp_hw_interface::KukaKR6System>();

  // Create fake HardwareInfo object
  hardware_interface::HardwareInfo hw_info;
  hw_info.name = "kuka_kr6";
  hw_info.hardware_parameters["ip_addr"] = "172.31.1.147";
  hw_info.hardware_parameters["port"] = "7000";

  // Add 6 joints (replace with actual joint names and types as needed)
  for (int i = 0; i < 6; ++i)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint_a" + std::to_string(i + 1);
    joint.command_interfaces = {{"position"}};
    joint.state_interfaces = {{"position"}, {"velocity"}};
    hw_info.joints.push_back(joint);
  }

  // Initialize the interface
  if (hw_interface->on_init(hw_info) != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize hardware interface.");
    return 1;
  }

  // Perform a read to get current state
  hw_interface->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

//   // Print joint states
//   for (size_t i = 0; i < hw_info.joints.size(); ++i) {
//     RCLCPP_INFO(node->get_logger(), "Joint %s pos: %.3f",
//       hw_info.joints[i].name.c_str(),
//       hw_interface->hw_positions_[i]);
//   }
  hw_interface->write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

  rclcpp::shutdown();
  return 0;
}