cmake_minimum_required(VERSION 3.16)
project(ros2_control_edgebotic_amr LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ros2_control_edgebotic_amr
  SHARED
  hardware/amr_system.cpp
  hardware/arduino_comms.cpp
  hardware/wheel.cpp
)
target_compile_features(ros2_control_edgebotic_amr PUBLIC cxx_std_17)
target_include_directories(ros2_control_edgebotic_amr PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ros2_control_edgebotic_amr>
)
ament_target_dependencies(
  ros2_control_edgebotic_amr PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_EDGEBOTIC_AMR_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_edgebotic_amr.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_edgebotic_amr
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_edgebotic_amr
)
install(TARGETS ros2_control_edgebotic_amr
  EXPORT export_ros2_control_edgebotic_amr
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#if(BUILD_TESTING)
#  find_package(ament_cmake_pytest REQUIRED)
#
#  ament_add_pytest_test(amr_urdf_xacro test/test_urdf_xacro.py)
#  ament_add_pytest_test(view_amr_launch test/test_view_robot_launch.py)
#endif()

## EXPORTS
ament_export_targets(export_ros2_control_edgebotic_amr HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
