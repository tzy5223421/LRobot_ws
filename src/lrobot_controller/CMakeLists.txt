cmake_minimum_required(VERSION 3.8)
project(lrobot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  nav_msgs
  tf2
  tf2_msgs
  diagnostic_msgs
  std_srvs
  canopen_interfaces
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(backward_ros REQUIRED)
find_package(controller_manager REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
include/lrobot_controller
)

# generate parameter listener hpp file
generate_parameter_library(lrobot_controller_parameters 
 src/lrobot_controller_parameter.yaml
)

# generate lib
add_library(lrobot_controller SHARED
  src/lrobot_controller.cpp
  src/odometry.cpp
)
target_compile_features(lrobot_controller PUBLIC cxx_std_17)

target_include_directories(lrobot_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/lrobot_controller>
)

target_link_libraries(lrobot_controller PUBLIC lrobot_controller_parameters)
ament_target_dependencies(lrobot_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(lrobot_controller PRIVATE "LROBOT_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface lrobot_plugin.xml)


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

install(
  DIRECTORY include/
  DESTINATION include/lrobot_controller
)
install(TARGETS lrobot_controller lrobot_controller_parameters
  EXPORT export_lrobot_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# 为了使下游文件可以访问
ament_export_targets(export_lrobot_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
