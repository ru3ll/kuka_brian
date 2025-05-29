#include "kuka_kr6_kvp_hw_interface/kuka_kr6_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstring>
#include <regex>
#include <string>
#include <iostream>

namespace kuka_kr6_kvp_hw_interface
{

hardware_interface::CallbackReturn KukaKR6System::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  kuka_ip_ = info_.hardware_parameters["ip_addr"];
  kuka_port_ = std::stoi(info_.hardware_parameters["port"]);

  RCLCPP_INFO(rclcpp::get_logger("KukaKR6System"), "Trying to connect to KUKA controller at %s:%d", kuka_ip_.c_str(), kuka_port_);
  if (!connect_to_kuka())
  {
    std::cerr << "Failed to connect to KUKA controller\n";
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("KukaKR6System"), "Connected to the robot");

  // initialize the hw_positions_ with the values from the actual robot
  try {
    std::string response = get_var(send_kvp_command("$AXIS_ACT", "", 0));
    std::cout << response << std::endl;

    std::regex joint_regex(R"(A[1-6]\s(-?\d+\.?\d*))");
    std::smatch match;
    std::string::const_iterator search_start(response.cbegin());
    
    size_t joint_index = 0;
    while (std::regex_search(search_start, response.cend(), match, joint_regex) && joint_index < hw_positions_.size()) {
      hw_positions_[joint_index] = std::stod(match[1]) * DEG2RAD;
      std::cout << "Joint " << joint_index << " initialized at " << hw_positions_[joint_index] << std::endl;
      search_start = match.suffix().first;
      ++joint_index;
    }

    if (joint_index < hw_positions_.size()) {
      RCLCPP_WARN(rclcpp::get_logger("KukaKR6System"), "Only %ld joint values parsed", joint_index);
    }
    RCLCPP_INFO(rclcpp::get_logger("KukaKR6System"), "initialized joint positions");
    return CallbackReturn::SUCCESS;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("KukaKR6System"), "Failed to get joint positions");
    return CallbackReturn::FAILURE;
  }
  
}

bool KukaKR6System::connect_to_kuka()
{
  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd_ < 0)
  {
    std::cerr << "Socket creation failed\n";
    return false;
  }

  kuka_addr_.sin_family = AF_INET;
  kuka_addr_.sin_port = htons(kuka_port_);
  if (inet_pton(AF_INET, kuka_ip_.c_str(), &kuka_addr_.sin_addr) <= 0)
  {
    std::cerr << "Invalid KUKA IP\n";
    return false;
  }

  if (connect(sockfd_, (struct sockaddr *)&kuka_addr_, sizeof(kuka_addr_)) < 0)
  {
    std::cerr << "Connection to KUKA controller failed\n";
    return false;
  }

  return true;
}

void KukaKR6System::disconect(){
  close(sockfd_);
  RCLCPP_INFO(rclcpp::get_logger("KukaKR6System"), "Disconnected from the robot");
}

std::vector<uint8_t> KukaKR6System::send_kvp_command(const std::string& var, const std::string& val, uint16_t msg_id)
{
  std::vector<uint8_t> msg;
  std::vector<uint8_t> temp;

  try {
    if (!val.empty()) {
      uint16_t val_len = val.size();
      msg.push_back((val_len & 0xFF00) >> 8);
      msg.push_back(val_len & 0x00FF);
      msg.insert(msg.end(), val.begin(), val.end());
    }

    temp.push_back(val.empty() ? 0 : 1);  // read=0, write=1
    uint16_t var_len = var.size();
    temp.push_back((var_len & 0xFF00) >> 8);
    temp.push_back(var_len & 0x00FF);
    temp.insert(temp.end(), var.begin(), var.end());

    msg.insert(msg.begin(), temp.begin(), temp.end());

    std::vector<uint8_t> final_msg;
    final_msg.push_back((msg_id & 0xFF00) >> 8);
    final_msg.push_back(msg_id & 0x00FF);
    uint16_t total_len = msg.size();
    final_msg.push_back((total_len & 0xFF00) >> 8);
    final_msg.push_back(total_len & 0x00FF);
    final_msg.insert(final_msg.end(), msg.begin(), msg.end());

    ssize_t sent = sendto(sockfd_, final_msg.data(), final_msg.size(), 0, nullptr, 0);
    if (sent < 0) throw std::runtime_error("Failed to send KVP message");

    std::array<uint8_t, 1024> buffer{};
    ssize_t received = recv(sockfd_, buffer.data(), buffer.size(), 0);
    if (received <= 0) throw std::runtime_error("No response from KUKAVARPROXY");

    return std::vector<uint8_t>(buffer.begin(), buffer.begin() + received);

  } catch (...) {
    throw_error(2);
    return {};
  }
}

std::string KukaKR6System::get_var(const std::vector<uint8_t>& msg)
{
  try {
    uint8_t lsb = msg[5];
    uint8_t msb = msg[6];
    uint16_t len = (lsb << 8) | msb;
    return std::string(msg.begin() + 7, msg.begin() + 7 + len);
  } catch (...) {
    throw_error(2);
    return "";
  }
}

void KukaKR6System::throw_error(int code)
{
  // You can customize this to publish ROS logs or throw std::runtime_error
  switch (code) {
    case 1:
      throw std::runtime_error("KVP socket communication failed");
    case 2:
      throw std::runtime_error("KVP message parsing failed");
    case 3:
      throw std::runtime_error("Empty value for KVP write");
    default:
      throw std::runtime_error("Unknown KVP error");
  }
}

std::vector<hardware_interface::StateInterface> KukaKR6System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KukaKR6System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn KukaKR6System::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("KukaKR6System"), "Activating hardware interface...");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KukaKR6System::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("KukaKR6System"), "Deactivating hardware interface...");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type KukaKR6System::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    std::string response = get_var(send_kvp_command("$AXIS_ACT", "", 0));
    std::cout << response << std::endl;

    std::regex joint_regex(R"(A[1-6]\s(-?\d+\.?\d*))");
    std::smatch match;
    std::string::const_iterator search_start(response.cbegin());
    
    size_t joint_index = 0;
    while (std::regex_search(search_start, response.cend(), match, joint_regex) && joint_index < hw_positions_.size()) {
      hw_positions_[joint_index] = std::stod(match[1]) * DEG2RAD;
      search_start = match.suffix().first;
      ++joint_index;
    }

    if (joint_index < hw_positions_.size()) {
      RCLCPP_WARN(rclcpp::get_logger("KukaKR6System"), "Only %ld joint values parsed", joint_index);
    }

    return hardware_interface::return_type::OK;
  } catch (...) {
    throw_error(2);
    return hardware_interface::return_type::ERROR;
  }
  
}

std::string KukaKR6System::format_joint_states(const std::vector<double> &joint_states)
{
    if (joint_states.size() < 6)
    {
        std::cerr << "No data or insufficient joint values." << std::endl;
        return "";
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2); // Format with 6 decimal places

    oss << "{E6AXIS: "
        << "A1 " << joint_states[0] * RAD2DEG << ", "
        << "A2 " << joint_states[1] * RAD2DEG << ", "
        << "A3 " << joint_states[2] * RAD2DEG << ", "
        << "A4 " << joint_states[3] * RAD2DEG << ", "
        << "A5 " << joint_states[4] * RAD2DEG << ", "
        << "A6 " << joint_states[5] * RAD2DEG << ", "
        << "E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}";

    return oss.str();
}

hardware_interface::return_type KukaKR6System::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  try {

    // std::string myString = "{E6AXIS: A1 0, A2 -80, A3 110, A4 0, A5 0, A6 180, E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}";
    // std::ostringstream oss;
    // oss << "{E6AXIS: ";
    // for (size_t i = 0; i < hw_commands_.size(); ++i)
    // {
    //   oss << "A" << (i + 1) << " " << hw_commands_[i]<< ", ";
    // }
    // oss << "E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}\r\n";
    // RCLCPP_INFO(rclcpp::get_logger("KukaKR6System"), oss.str().c_str());

    // std::vector<double> joints = {0.0, -90.0, 90.0, 0.0, 0.0, 0.0};
    std::string response = get_var(send_kvp_command("COM_E6AXIS", format_joint_states(hw_commands_), 0));
    // RCLCPP_INFO(rclcpp::get_logger("KukaKR6System"),response.c_str());

    return hardware_interface::return_type::OK;
    }catch(...){
      RCLCPP_ERROR(rclcpp::get_logger("KukaKR6System"),"Error Writing to hardware");
      return hardware_interface::return_type::ERROR;
    }
}

}  // namespace kuka_kr6_kvp_hw_interface

PLUGINLIB_EXPORT_CLASS(kuka_kr6_kvp_hw_interface::KukaKR6System, hardware_interface::SystemInterface)
