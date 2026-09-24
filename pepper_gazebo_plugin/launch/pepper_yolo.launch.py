"""Detección de objetos con YOLO sobre la cámara frontal de Pepper.

En el V8: darknet_ros (YOLOv2) con camera_reading cambiado de "/camera/rgb/image_raw" a
"/pepper/camera/front/image_raw", y `roslaunch darknet_ros darknet_ros.launch`.
En el V9: yolo_ros (ultralytics) con input_image_topic en /pepper/camera/front/image_raw.

    ros2 launch pepper_gazebo_plugin pepper_yolo.launch.py
    ros2 launch pepper_gazebo_plugin pepper_yolo.launch.py model:=yolov8m.pt threshold:=0.4

Topics (namespace /yolo): detections (yolo_msgs/DetectionArray), dbg_image (imagen con las cajas).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="yolo11m.pt"),
        DeclareLaunchArgument("device", default_value="cuda:0"),
        DeclareLaunchArgument("threshold", default_value="0.5"),
        DeclareLaunchArgument("camera", default_value="/pepper/camera/front/image_raw",
                              description="V8: /pepper/camera/front/image_raw"),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("yolo_bringup"), "launch", "yolo.launch.py"]),
            launch_arguments={
                "model": LaunchConfiguration("model"),
                "device": LaunchConfiguration("device"),
                "threshold": LaunchConfiguration("threshold"),
                "input_image_topic": LaunchConfiguration("camera"),
                "image_reliability": "1",  # reliable, como publica ros_gz_bridge
                "use_tracking": "False",
                "use_debug": "True",
            }.items(),
        ),
    ])
