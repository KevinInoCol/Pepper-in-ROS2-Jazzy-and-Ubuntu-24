"""SLAM 2D con slam_toolbox (en el V8: rosrun gmapping slam_gmapping scan:=/pepper/laser_2).

    ros2 launch pepper_gazebo_plugin pepper_slam_toolbox.launch.py                       # laser_2
    ros2 launch pepper_gazebo_plugin pepper_slam_toolbox.launch.py scan:=/pepper/hokuyo_scan max_laser_range:=20.0

En Jazzy slam_toolbox es un nodo lifecycle: hay que configurarlo y activarlo (como hace el
online_async_launch.py oficial); si no, se queda en "unconfigured" y no publica /map.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("pepper_gazebo_plugin"), "config", "pepper_slam_toolbox.yaml"])

    slam = LifecycleNode(
        package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
        namespace="", output="screen",
        parameters=[config, {"use_sim_time": True,
                             "use_lifecycle_manager": False,
                             "scan_topic": LaunchConfiguration("scan"),
                             "max_laser_range": LaunchConfiguration("max_laser_range")}],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state="configuring", goal_state="inactive",
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    return LaunchDescription([
        DeclareLaunchArgument("scan", default_value="/pepper/laser_2",
                              description="/pepper/laser_2 o /pepper/hokuyo_scan (las dos opciones del V8)"),
        DeclareLaunchArgument("max_laser_range", default_value="7.0"),
        slam,
        configure,
        activate,
    ])
