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
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

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

  time_ = std::chrono::system_clock::now();

  left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  device = info_.hardware_parameters["device"];
  baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  timeout = std::stoi(info_.hardware_parameters["timeout"]);
  enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  debug = std::stoi(info_.hardware_parameters["debug"]);

  // Set up the wheels
  l_wheel_.setup(left_wheel_name, enc_counts_per_rev);
  r_wheel_.setup(right_wheel_name, enc_counts_per_rev);

  // Set up the Arduino
  arduino_.setup(device, baud_rate, timeout, debug);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AMRSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AMRSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn AMRSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AMRSystemHardware"), "Activating Edgebotic AMR Controller....");

  arduino_.sendEmptyMsg();
  arduino_.setPidValues(30, 20, 0, 100);

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

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
  /*// BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  for (std::size_t i = 0; i < hw_velocities_.size(); i++)
  {
    // Simulate AMR wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];

    RCLCPP_INFO(
      rclcpp::get_logger("AMRSystemHardware"),
      "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
      hw_velocities_[i], info_.joints[i].name.c_str());
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code*/

  // Use encoder count to calculate linear and angular position and velocity as given by:
  // linear = (right_wheel_est_vel + left_wheel_est_vel) / 2
  // angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation;

  /*// Compute wheel velocities from encoder counts
  double left_wheel_est_vel = left_wheel_encoder_count_ * 0.0344 * 2 * M_PI / 1992 / period.seconds();
  double right_wheel_est_vel = right_wheel_encoder_count_ * 0.0344 * 2 * M_PI / 1992 / period.seconds();

  // Reset encoder counts
  left_wheel_encoder_count_ = 0;
  right_wheel_encoder_count_ = 0;

  // Update linear and angular velocity
  hw_velocities_[0] = (right_wheel_est_vel + left_wheel_est_vel) / 2;
  hw_velocities_[1] = (right_wheel_est_vel - left_wheel_est_vel) / 0.26;
  
  // Update linear and angular position
  hw_positions_[0] = hw_positions_[0] + period.seconds() * hw_velocities_[0];
  hw_positions_[1] = hw_positions_[1] + period.seconds() * hw_velocities_[1];*/


  // Calculate time delta
  /*auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;*/
  double deltaSeconds = period.seconds();

  if (!arduino_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  arduino_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

  if (debug)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("AMRSystemHardware"), "Left wheel encoder: " << l_wheel_.enc);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("AMRSystemHardware"), "Right wheel encoder: " << r_wheel_.enc);
  }

  double pos_prev = l_wheel_.pos;
  hw_positions_[0] = l_wheel_.pos = l_wheel_.calcEncAngle();
  hw_velocities_[0] = l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = r_wheel_.pos;
  hw_positions_[1] = r_wheel_.pos = r_wheel_.calcEncAngle();
  hw_velocities_[1] = r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_edgebotic_amr ::AMRSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  /*// BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("AMRSystemHardware"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("AMRSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());

    hw_velocities_[i] = hw_commands_[i];
  }
  RCLCPP_INFO(rclcpp::get_logger("AMRSystemHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code*/

  // Convert linear & angular velocities to left & right wheel velocities
  // linear velocity is in m/s
  // angular velocity is in rad/s
  //double left_wheel_vel = (hw_commands_[0] + 0.13 * hw_commands_[1]) / 0.0344;
  //double right_wheel_vel = (hw_commands_[0] - 0.13 * hw_commands_[1]) / 0.0344;

  if (!arduino_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  arduino_.setMotorValues(l_wheel_.cmd / l_wheel_.rads_per_count / loop_rate, r_wheel_.cmd / r_wheel_.rads_per_count / loop_rate);

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_edgebotic_amr

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_edgebotic_amr::AMRSystemHardware, hardware_interface::SystemInterface)
