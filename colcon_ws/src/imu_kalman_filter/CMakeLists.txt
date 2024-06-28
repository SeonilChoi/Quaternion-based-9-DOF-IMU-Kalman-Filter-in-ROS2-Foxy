cmake_minimum_required(VERSION 3.5)
project(imu_kalman_filter)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

set(PORT_HANDLER_SOURCES src/PortHandler.cpp)
add_library(port_handler_library SHARED ${PORT_HANDLER_SOURCES})
install(TARGETS port_handler_library DESTINATION lib)

set(imu_publisher_dependencies 
    "rclcpp" 
    "sensor_msgs")
add_executable(imu_publisher src/IMUPublisher.cpp)
ament_target_dependencies(imu_publisher ${imu_publisher_dependencies})
target_link_libraries(imu_publisher port_handler_library)
install(TARGETS imu_publisher DESTINATION lib/${PROJECT_NAME})

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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)

ament_package()
