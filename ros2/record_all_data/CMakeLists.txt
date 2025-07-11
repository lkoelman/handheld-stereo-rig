cmake_minimum_required(VERSION 3.8)
project(record_all_data)

#Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# EXECUTABLES
#-------------------------------
add_executable(record_all_data src/recorder/record_all_data.cpp)
ament_target_dependencies(record_all_data rclcpp rosbag2_cpp sensor_msgs)

#-------------------------------
add_executable(record_all_mock_data src/recorder/record_all_mock_data.cpp)
ament_target_dependencies(record_all_mock_data rclcpp rosbag2_cpp sensor_msgs)

#-------------------------------
add_executable(mock_imu_pub src/mock_sensor_pubs/mock_imu_pub.cpp)
ament_target_dependencies(mock_imu_pub rclcpp std_msgs sensor_msgs)

#-------------------------------
add_executable(mock_camera_pub src/mock_sensor_pubs/mock_camera_pub.cpp)
ament_target_dependencies(mock_camera_pub rclcpp std_msgs sensor_msgs)


install(TARGETS
  record_all_data
  record_all_mock_data
  mock_camera_pub
  mock_imu_pub
  DESTINATION lib/${PROJECT_NAME}
)

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
