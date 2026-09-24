cmake_minimum_required(VERSION 3.8)
project(pepper_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz urdf DESTINATION share/${PROJECT_NAME})

# Gazebo resuelve package://pepper_description/... buscando en GZ_SIM_RESOURCE_PATH
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
