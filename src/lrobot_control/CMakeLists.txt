cmake_minimum_required(VERSION 3.8)
project(lrobot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  canopen_402_driver
  canopen_core
  canopen_proxy_driver
)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros2_control_test_assets)
find_package(ament_cmake_gmock REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
include/lrobot_control
)

## COMPILE
add_library(
  lrobot_control
  SHARED
  src/lrobot_system.cpp
)

target_compile_features(lrobot_control PUBLIC cxx_std_17)

target_include_directories(lrobot_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/lrobot_control>
)

ament_target_dependencies(
  lrobot_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFF_TEST_CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface lrobot_control.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/lrobot_control
)

install(
  DIRECTORY  description/ros2_control description/urdf  description/launch description/rviz description/meshes
  DESTINATION share/lrobot_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/lrobot_control
)
install(TARGETS lrobot_control
  EXPORT export_lrobot_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
