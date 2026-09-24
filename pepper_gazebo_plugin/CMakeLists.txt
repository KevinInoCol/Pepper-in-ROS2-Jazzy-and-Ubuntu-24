cmake_minimum_required(VERSION 3.8)
project(pepper_gazebo_plugin)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Sonares: LaserScan de 5x5 rayos -> sensor_msgs/Range (V8: libgazebo_ros_range.so)
add_executable(sonar_to_range src/sonar_to_range.cpp)
target_link_libraries(sonar_to_range rclcpp::rclcpp ${sensor_msgs_TARGETS})
install(TARGETS sonar_to_range DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/arms_down.sh
  scripts/joy_pepper.py
  scripts/laser_publisher.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
