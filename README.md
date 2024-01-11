# ros2_control hardware interface for the Edgebotic AMR

The Edgebotic AMR is a differential drive robot built upon the Viam Rover VR1 base. It uses an Arduino MKR Wifi 1010 as a motor controller.

This project provides a hardware interface to be used with ros2_control's diff_drive_controller. It's based on [ros2_control hardware interface examples](https://github.com/ros-controls/ros2_control_demos) and uses boilerplate directly from those projects. The serial interface code is adapted from [Josh Newans' diffdrive_arduino project](https://github.com/joshnewans/diffdrive_arduino).