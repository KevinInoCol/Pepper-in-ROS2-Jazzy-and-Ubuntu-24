"""rqt_joint_trajectory_controller para Pepper (en el V8: rosrun rqt_joint_trajectory_controller ...).

El plugin de ROS 2 publica siempre en <controlador>/joint_trajectory y lee robot_description sin
namespace. Se remapea para que use los topics del V8 (/pepper/<X>_controller/command) y
/pepper/robot_description.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

CONTROLLERS = [
    "LeftArm_controller",
    "RightArm_controller",
    "Head_controller",
    "Pelvis_controller",
    "LeftHand_controller",
    "RightHand_controller",
]


def generate_launch_description():
    remappings = [("robot_description", "/pepper/robot_description")] + [
        (f"/pepper/{c}/joint_trajectory", f"/pepper/{c}/command") for c in CONTROLLERS
    ]
    return LaunchDescription([
        Node(
            package="rqt_joint_trajectory_controller",
            executable="rqt_joint_trajectory_controller",
            parameters=[{"use_sim_time": True}],
            remappings=remappings,
        ),
    ])
