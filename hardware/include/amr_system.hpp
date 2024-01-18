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

#ifndef ROS2_CONTROL_EDGEBOTIC_AMR__AMR_SYSTEM_HPP_
#define ROS2_CONTROL_EDGEBOTIC_AMR__AMR_SYSTEM_HPP_

#include <string>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"

#include "wheel.h"
#include "arduino_comms.h"

namespace ros2_control_edgebotic_amr
{
class AMRSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AMRSystemHardware);

  ROS2_CONTROL_EDGEBOTIC_AMR_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_EDGEBOTIC_AMR_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_EDGEBOTIC_AMR_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_EDGEBOTIC_AMR_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_EDGEBOTIC_AMR_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_EDGEBOTIC_AMR_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROS2_CONTROL_EDGEBOTIC_AMR_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  ArduinoComms arduino_;

  Wheel l_wheel_;
  Wheel r_wheel_;

  // Config params
  float loop_rate = 10;
  std::string device = "/dev/ttyACM0";
  int baud_rate = 57600;
  int timeout = 500;
  int enc_counts_per_rev = 1992;
  int debug = 0;
};

}  // namespace ros2_control_edgebotic_amr

#endif  // ROS2_CONTROL_EDGEBOTIC_AMR__AMR_SYSTEM_HPP_
