cmake_minimum_required(VERSION 3.8)
project(vn100_pub)

add_definitions(-w)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

###########
## Build ##
###########
find_package(Boost 1.54 COMPONENTS log REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

add_subdirectory(vnproglib-1.2.0.0/cpp)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(vnproglib-1.2.0.0/cpp/include)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

## Declare a cpp library
## Declare a cpp executable
add_executable(vn100pub src/vn100_ros2_node.cpp)

ament_target_dependencies(vn100pub rclcpp sensor_msgs)

## Specify libraries to link a library or executable target against
target_link_libraries(vn100pub
  libvncxx
  ${Boost_LOG_LIBRARY}
)

install(TARGETS
  vn100pub
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
