"""Pepper en Gazebo Harmonic (base de los launch pepper_gazebo_plugin_*.launch del V8).

Carga el modelo, abre el mundo, hace el spawn como "pepper_MP" (igual que el V8),
arranca los controladores, baja los brazos, pasa los sensores a ROS 2 (ros_gz_bridge) y
lanza laser_publisher.py (/pepper/laser_2) y sonar_to_range (/pepper/sonar_*).
"""

from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
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

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        parameters=[{"config_file": PathJoinSubstitution(
            [FindPackageShare("pepper_gazebo_plugin"), "config", "pepper_bridge.yaml"])}],
    )

    # V8: gazebo_ros publicaba /clock a 100 Hz (pub_clock_frequency)
    clock_throttle = Node(package="pepper_gazebo_plugin", executable="clock_throttle")

    # V8: <node name="laser_publisher" pkg="pepper_gazebo_plugin" type="laser_publisher.py"/>
    laser_publisher = Node(
        package="pepper_gazebo_plugin", executable="laser_publisher.py",
        parameters=[{"use_sim_time": True}],
    )

    # V8: plugin libgazebo_ros_range.so dentro del modelo
    sonar_to_range = Node(
        package="pepper_gazebo_plugin", executable="sonar_to_range",
        parameters=[{"use_sim_time": True}],
    )

    controllers_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("pepper_control"), "launch", "pepper_control_trajectory_all.launch.py"]))

    lower_arms = Node(package="pepper_gazebo_plugin", executable="arms_down.sh", output="screen")

    # V8: el plugin openni publicaba depth/points en CameraDepth_optical_frame.
    # depth_image_proc hace lo mismo a partir de la imagen de profundidad y su camera_info.
    depth_points = Node(
        package="depth_image_proc", executable="point_cloud_xyz_node",
        remappings=[("image_rect", "/pepper/camera/depth/image_raw"),
                    ("camera_info", "/pepper/camera/depth/camera_info"),
                    ("points", "/pepper/camera/depth/points")],
        parameters=[{"use_sim_time": True}],
    )

    # V8: <env name="GAZEBO_MODEL_PATH" value="$(find pepper_gazebo_plugin)/models:..."/>
    model_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        PathJoinSubstitution([FindPackageShare("pepper_gazebo_plugin"), "models"]))

    return LaunchDescription([
        model_path,
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution(
            [FindPackageShare("pepper_gazebo_plugin"), "worlds", "empty.world"])),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        gazebo,
        bridge,
        clock_throttle,
        robot_state_publisher,
        laser_publisher,
        sonar_to_range,
        depth_points,
        spawn,
        # Los controladores se cargan cuando el robot ya existe en Gazebo
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[controllers_launch, lower_arms])),
    ])
