"""Pepper en Gazebo Harmonic (base de los launch pepper_gazebo_plugin_*.launch del V8).

Carga el modelo, abre el mundo, hace el spawn como "pepper_MP" (igual que el V8),
arranca los controladores y baja los brazos.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")
    description = FindPackageShare("pepper_description")
    controllers = PathJoinSubstitution(
        [FindPackageShare("pepper_control"), "config", "pepper_trajectory_control.yaml"])

    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([description, "urdf", "pepper_robot.urdf.xacro"]),
                 " gazebo:=true controllers_file:=", controllers]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher", namespace="pepper",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-topic", "/pepper/robot_description", "-name", "pepper_MP",
                   "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"), "-z", "0.05"],
    )

    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    controllers_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("pepper_control"), "launch", "pepper_control_trajectory_all.launch.py"]))

    lower_arms = Node(package="pepper_gazebo_plugin", executable="arms_down.sh", output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution(
            [FindPackageShare("pepper_gazebo_plugin"), "worlds", "empty.world"])),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn,
        # Los controladores se cargan cuando el robot ya existe en Gazebo
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[controllers_launch, lower_arms])),
    ])
