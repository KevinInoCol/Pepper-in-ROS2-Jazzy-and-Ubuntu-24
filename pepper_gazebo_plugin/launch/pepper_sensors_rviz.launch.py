"""RViz2 con todos los sensores de Pepper.

En el V8: rosrun rviz rviz -d `rospack find pepper_gazebo_plugin`/config/pepper_sensors.rviz
Con mapa (SLAM), como pepper_sensors_map.rviz del V8:  config:=pepper_sensors_map.rviz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="pepper_sensors.rviz"),
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("pepper_gazebo_plugin"), "config", LaunchConfiguration("config")])],
            parameters=[{"use_sim_time": True}],
        ),
    ])
