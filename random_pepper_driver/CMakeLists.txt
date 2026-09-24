cmake_minimum_required(VERSION 3.8)
project(random_pepper_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# En el V8: add_executable(random_driver random_driver.cpp) +
#           target_link_libraries(random_driver ${catkin_LIBRARIES})
add_executable(random_driver src/random_driver.cpp)
target_link_libraries(random_driver rclcpp::rclcpp ${geometry_msgs_TARGETS})

install(TARGETS random_driver DESTINATION lib/${PROJECT_NAME})

ament_package()
