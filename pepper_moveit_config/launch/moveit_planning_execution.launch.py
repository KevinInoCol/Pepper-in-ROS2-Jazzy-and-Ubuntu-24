"""MoveIt 2 para Pepper en simulación (move_group + RViz2 con MotionPlanning).

Equivale al moveit_planning_execution.launch de ros-naoqi/pepper_moveit_config (ROS 1).
La simulación se lanza aparte:

    ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_in_office_CPU.launch.py
    ros2 launch pepper_moveit_config moveit_planning_execution.launch.py

MoveIt ejecuta las trayectorias con los controladores del V8 (/pepper/LeftArm_controller, ...).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    controllers_file = os.path.join(
        get_package_share_directory("pepper_control"), "config", "pepper_trajectory_control.yaml")
    moveit_config = (
        MoveItConfigsBuilder("JulietteY20MP", package_name="pepper_moveit_config")
        .robot_description(
            file_path=os.path.join(get_package_share_directory("pepper_description"),
                                   "urdf", "pepper_robot.urdf.xacro"),
            mappings={"gazebo": "true", "controllers_file": controllers_file})
        .robot_description_semantic(file_path="config/pepper.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        # Octomap en odom con la resolución de ros-naoqi (sensor_manager.launch.xml)
        parameters=[moveit_config.to_dict(), {"use_sim_time": True,
                                              "octomap_frame": "odom",
                                              "octomap_resolution": 0.025}],
        # Los estados de las articulaciones están en /pepper/joint_states (nomenclatura del V8)
        remappings=[("joint_states", "/pepper/joint_states")],
    )

    rviz = Node(
        package="rviz2", executable="rviz2", output="log",
        arguments=["-d", os.path.join(get_package_share_directory("pepper_moveit_config"),
                                      "config", "moveit.rviz")],
        # Sólo use_sim_time: RViz recibe el modelo (URDF y SRDF) por topic desde move_group.
        # Con kinematics/joint_limits como parámetros, el plugin MotionPlanning de MoveIt 2.12.4
        # no carga el modelo ("... is of type double, setting it to string is not allowed").
        # Consecuencia: en RViz no aparece el marcador para arrastrar la mano; se planifica con
        # las poses con nombre (Goal State) o la pestaña Joints, y los objetivos cartesianos se
        # mandan por código (scripts/pepper_moveit_demo.py).
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        move_group,
        rviz,
    ])
