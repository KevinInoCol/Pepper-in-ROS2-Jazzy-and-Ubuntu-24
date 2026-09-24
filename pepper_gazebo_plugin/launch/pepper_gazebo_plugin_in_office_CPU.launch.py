"""Pepper en la oficina con personas (V8: pepper_gazebo_plugin_in_office_CPU.launch).

Mismo mundo (simple_office_with_people.world) y misma posición inicial que el V8
(spawn_model -x -0.5 -y 1).
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("pepper_gazebo_plugin")
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([pkg, "launch", "pepper_gazebo_plugin_empty.launch.py"]),
            launch_arguments={
                "world": PathJoinSubstitution([pkg, "worlds", "simple_office_with_people.world"]),
                "x": "-0.5",
                "y": "1.0",
            }.items(),
        ),
    ])
