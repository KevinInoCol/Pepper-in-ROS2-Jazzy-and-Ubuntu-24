"""RViz2 con todos los sensores de Pepper.

En el V8: rosrun rviz rviz -d `rospack find pepper_gazebo_plugin`/config/pepper_sensors.rviz
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("pepper_gazebo_plugin"), "config", "pepper_sensors.rviz"])],
            parameters=[{"use_sim_time": True}],
        ),
    ])
