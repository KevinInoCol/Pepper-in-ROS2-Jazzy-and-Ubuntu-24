"""Joystick para Pepper: joy_node + joy_pepper.py (en el V8: rosrun joy joy_node + joy_pepper.py).

En el V8 el mando se elegía con `rosparam set joy_node/dev "/dev/input/jsX"`. En ROS 2,
joy_node usa SDL y el mando se elige por número con device_id (ver
`ros2 run joy joy_enumerate_devices`).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("device_id", default_value="0"),
        DeclareLaunchArgument("enable_button", default_value="-1",
                              description="Botón de hombre muerto (-1 = ninguno, 4 = LB del Xbox)"),
        Node(package="joy", executable="joy_node",
             parameters=[{"device_id": LaunchConfiguration("device_id"),
                          "deadzone": 0.05, "autorepeat_rate": 20.0}]),
        Node(package="pepper_gazebo_plugin", executable="joy_pepper.py", output="screen",
             parameters=[{"enable_button": LaunchConfiguration("enable_button")}]),
    ])
