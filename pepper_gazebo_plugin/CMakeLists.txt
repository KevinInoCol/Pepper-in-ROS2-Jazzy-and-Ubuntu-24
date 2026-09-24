cmake_minimum_required(VERSION 3.8)
project(pepper_gazebo_plugin)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/arms_down.sh DESTINATION lib/${PROJECT_NAME})

ament_package()
