cmake_minimum_required(VERSION 3.8)
project(lrobot_canopen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(lely_core_libraries REQUIRED)

#cogen_dcf(canopen)
generate_dcf(canopen)

install(DIRECTORY
  launch 
  DESTINATION share/${PROJECT_NAME}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

find_package(Threads REQUIRED)
find_package(ament_cmake_auto REQUIRED)

set(SRC_LIST
src/async_can.cpp)

set(HEAD_LIST
include/lrobot_canopen/async_can.hpp
)

# add_executable(lrobot_canopen 
#   ${SRC_LIST}
# )
ament_auto_add_library(${PROJECT_NAME}
  ${SRC_LIST}
  ${HEAD_LIST}
)

target_link_libraries(${PROJECT_NAME})

ament_auto_package()
