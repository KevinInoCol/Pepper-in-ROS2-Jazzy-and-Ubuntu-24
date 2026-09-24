#!/bin/bash
# Baja los brazos de Pepper al arrancar (port ROS 2 de pepper_gazebo_plugin/scripts/arms_down.sh del V8).

# Esperar a que los controladores de los brazos estén escuchando
until ros2 topic info /pepper/LeftArm_controller/command 2>/dev/null | grep -q "Subscription count: [1-9]" && \
      ros2 topic info /pepper/RightArm_controller/command 2>/dev/null | grep -q "Subscription count: [1-9]"
do
  echo "Waiting for controllers to be ready..."
  sleep 1.0
done

echo "Lowering arms."
sleep 2.0

ros2 topic pub --once /pepper/LeftArm_controller/command trajectory_msgs/msg/JointTrajectory "
joint_names: [LElbowRoll, LElbowYaw, LShoulderPitch, LShoulderRoll, LWristYaw]
points:
- positions: [-0.10, 0.0, 1.45, 0.10, -1.0]
  time_from_start: {sec: 1, nanosec: 0}" > /dev/null &

ros2 topic pub --once /pepper/RightArm_controller/command trajectory_msgs/msg/JointTrajectory "
joint_names: [RElbowRoll, RElbowYaw, RShoulderPitch, RShoulderRoll, RWristYaw]
points:
- positions: [0.10, 0.0, 1.45, -0.10, 1.0]
  time_from_start: {sec: 1, nanosec: 0}" > /dev/null &

wait
