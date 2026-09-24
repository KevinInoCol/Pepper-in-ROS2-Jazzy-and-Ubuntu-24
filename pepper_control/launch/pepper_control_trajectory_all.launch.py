"""Arranca los controladores de Pepper (equivalente a pepper_control_trajectory_all.launch del V8).

En ROS 2 el JointTrajectoryController escucha en ~/joint_trajectory; se remapea a ~/command
para conservar los topics del V8 (/pepper/LeftArm_controller/command, ...).
"""

from launch import LaunchDescription
from launch_ros.actions import Node

TRAJECTORY_CONTROLLERS = [
    "RightArm_controller",
    "LeftArm_controller",
    "Head_controller",
    "Pelvis_controller",
    "LeftHand_controller",
    "RightHand_controller",
]


def generate_launch_description():
    joint_state = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_controller", "--controller-manager", "/pepper/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    trajectory = Node(
        package="controller_manager",
        executable="spawner",
        arguments=TRAJECTORY_CONTROLLERS + [
            "--controller-manager", "/pepper/controller_manager",
            "--controller-ros-args", "-r ~/joint_trajectory:=~/command",
        ],
        parameters=[{"use_sim_time": True}],
    )
    return LaunchDescription([joint_state, trajectory])
