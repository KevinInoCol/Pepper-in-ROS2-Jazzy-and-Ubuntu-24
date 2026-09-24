"""Navegación de Pepper con Nav2 (en el V8: roslaunch pepper_nav amcl.launch).

Localización (AMCL) + navegación (planner, controlador MPPI omnidireccional, behaviors) +
RViz2. La simulación se lanza aparte (pepper_gazebo_plugin_in_office_CPU.launch.py).

    ros2 launch pepper_nav amcl.launch.py                               # mapa de la oficina, hokuyo
    ros2 launch pepper_nav amcl.launch.py scan:=/pepper/laser_2         # con los 3 láseres de Pepper
    ros2 launch pepper_nav amcl.launch.py map:=/ruta/a/mi_mapa.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav_share = FindPackageShare("pepper_nav")
    scan = LaunchConfiguration("scan")

    # El láser elegido se usa en AMCL (scan_topic), en los costmaps y en el collision monitor (topic)
    params = RewrittenYaml(
        source_file=PathJoinSubstitution([nav_share, "config", "nav2_params.yaml"]),
        param_rewrites={"scan_topic": scan, "topic": scan},
        convert_types=True,
    )

    bringup = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": params,
            "use_sim_time": "true",
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )

    rviz = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", PathJoinSubstitution([nav_share, "rviz", "pepper_nav.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=PathJoinSubstitution(
            [FindPackageShare("pepper_gazebo_plugin"), "map", "office.yaml"])),
        DeclareLaunchArgument("scan", default_value="/pepper/hokuyo_scan",
                              description="/pepper/hokuyo_scan o /pepper/laser_2"),
        DeclareLaunchArgument("rviz", default_value="true"),
        bringup,
        rviz,
    ])
