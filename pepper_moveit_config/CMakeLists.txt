cmake_minimum_required(VERSION 3.8)
project(pepper_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/pepper_moveit_demo.py DESTINATION lib/${PROJECT_NAME})

ament_package()
