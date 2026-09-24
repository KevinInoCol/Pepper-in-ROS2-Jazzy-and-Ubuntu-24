"""Pepper en RViz2 sin Gazebo (equivalente a display.launch del V8)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    fingers = LaunchConfiguration("fingers")
    pkg = FindPackageShare("pepper_description")

    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([pkg, "urdf", "pepper_robot.urdf.xacro"]),
                 " fingers:=", fingers]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="Sliders para mover las articulaciones"),
        DeclareLaunchArgument("fingers", default_value="false",
                              description="Incluir los dedos (el V8 no los usa en simulación)"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace="pepper", parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             namespace="pepper", condition=IfCondition(gui)),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             namespace="pepper", condition=UnlessCondition(gui)),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", PathJoinSubstitution([pkg, "rviz", "urdf.rviz"])]),
    ])
