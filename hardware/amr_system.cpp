// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "amr_system.hpp"

// for signal handling
#include <signal.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_edgebotic_amr
{
hardware_interface::CallbackReturn AMRSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // AMRSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AMRSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AMRSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AMRSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AMRSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AMRSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  device = info_.hardware_parameters["device"];
  baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  timeout = std::stoi(info_.hardware_parameters["timeout"]);
  enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  debug = std::stoi(info_.hardware_parameters["debug"]);

  // Set up the wheels
  l_wheel_.setup(info_.joints[0].name, enc_counts_per_rev);
  r_wheel_.setup(info_.joints[1].name, enc_counts_per_rev);

  // Set up the Arduino
  arduino_.setup(device, baud_rate, timeout, debug);

  RCLCPP_INFO(rclcpp::get_logger("AMRSystemHardware"), "Successfully initialized!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AMRSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AMRSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn AMRSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AMRSystemHardware"), "Activating Edgebotic AMR Controller....");

  arduino_.sendEmptyMsg();
  //arduino_.setPidValues(30, 20, 0, 100);

  RCLCPP_INFO(rclcpp::get_logger("AMRSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AMRSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AMRSystemHardware"), "Deactivating Edgebotic AMR Controller....");

  RCLCPP_INFO(rclcpp::get_logger("AMRSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AMRSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double deltaSeconds = period.seconds();

  if (!arduino_.connected())
  {
    RCLCPP_FATAL(rclcpp::get_logger("AMRSystemHardware"), "Motor controller not connected.");
    return hardware_interface::return_type::ERROR;
  }

  arduino_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

  if (debug)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("AMRSystemHardware"), "Wheel encoder counts --  Left: " << l_wheel_.enc << "  Right: " << r_wheel_.enc);
  }

  double pos_prev = l_wheel_.pos;
  l_wheel_.pos = l_wheel_.calcEncAngle();
  l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = r_wheel_.pos;
  r_wheel_.pos = r_wheel_.calcEncAngle();
  r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_edgebotic_amr ::AMRSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (debug)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("AMRSystemHardware"), "Wheel command received --  Left: " << l_wheel_.cmd << "  Right: " << r_wheel_.cmd);
  }
  if (!arduino_.connected())
  {
    RCLCPP_FATAL(rclcpp::get_logger("AMRSystemHardware"), "Motor controller not connected.");
    return hardware_interface::return_type::ERROR;
  }

  arduino_.setMotorValues(l_wheel_.cmd / l_wheel_.rads_per_count / loop_rate, r_wheel_.cmd / r_wheel_.rads_per_count / loop_rate);

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_edgebotic_amr

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_edgebotic_amr::AMRSystemHardware, hardware_interface::SystemInterface)
