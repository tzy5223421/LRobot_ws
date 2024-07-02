cmake_minimum_required(VERSION 3.8)
project(lrobot_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()




# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lrobot_canopen REQUIRED)

include_directories(
  include
)

set(DEPENDENCIES
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "sensor_msgs"
  "lrobot_canopen"
)
add_executable(lrobot_base_node 
    src/lrobot_base_ros.cpp
    src/lrobot_base_node.cpp
)

# add_executable(lrobot_base_node 
#     src/lrobot_base_ros.cpp
#     src/lrobot_base_node.cpp)
target_link_libraries(lrobot_base_node)

ament_target_dependencies(lrobot_base_node rclcpp tf2 tf2_ros std_msgs tf2_geometry_msgs nav_msgs sensor_msgs lrobot_canopen)

install(TARGETS lrobot_base_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

install(FILES lib/liblrobot_canopen_lib.so
  DESTINATION lib)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
