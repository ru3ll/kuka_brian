#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/logger.hpp"
#include "pluginlib/class_list_macros.hpp"


#include <string>
#include <vector>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>

#include <cstring>
#include <iostream>

namespace kuka_kr6_kvp_hw_interface
{
  #define DEG2RAD 0.01745329252 
  #define RAD2DEG 57.295779513


class KukaKR6System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KukaKR6System)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  

private:
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  std::string kuka_ip_;
  int kuka_port_;
  int sockfd_;
  struct sockaddr_in kuka_addr_;

  bool connect_to_kuka();
  std::vector<uint8_t> send_kvp_command(const std::string& var, const std::string& val, uint16_t msg_id = 0);
  std::string get_var(const std::vector<uint8_t>& msg);
  void throw_error(int code);
  void disconect();
  std::string format_joint_states(const std::vector<double> &joint_states);
};

}  // namespace kuka_kr6_kvp_hw_interface
