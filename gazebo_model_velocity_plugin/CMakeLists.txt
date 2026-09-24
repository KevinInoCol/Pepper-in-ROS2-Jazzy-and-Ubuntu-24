cmake_minimum_required(VERSION 3.8)
project(gazebo_model_velocity_plugin)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Base holonómica + odometría (equivale a libgazebo_ros_model_velocity.so del V8)
add_library(gazebo_ros_model_velocity SHARED
  src/gazebo_ros_model_velocity.cpp
  src/speed_limiter.cpp)
target_include_directories(gazebo_ros_model_velocity PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(gazebo_ros_model_velocity
  gz-sim::gz-sim gz-plugin::register
  rclcpp::rclcpp tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS})

# Ground truth (equivale a libgazebo_ros_p3d.so de gazebo_plugins)
add_library(gazebo_ros_p3d SHARED src/gazebo_ros_p3d.cpp)
target_link_libraries(gazebo_ros_p3d
  gz-sim::gz-sim gz-plugin::register
  rclcpp::rclcpp ${nav_msgs_TARGETS})

install(TARGETS gazebo_ros_model_velocity gazebo_ros_p3d LIBRARY DESTINATION lib)

# Gazebo busca los plugins en GZ_SIM_SYSTEM_PLUGIN_PATH
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
