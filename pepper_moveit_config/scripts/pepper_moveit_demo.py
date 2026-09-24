#!/usr/bin/env python3
"""Demo de MoveIt 2 con Pepper: poses con nombre del SRDF y un objetivo cartesiano.

Con la simulación y moveit_planning_execution.launch.py en marcha:

    ros2 run pepper_moveit_config pepper_moveit_demo.py

Planifica y ejecuta por la acción /move_action de move_group, que usa los controladores del
V8 (/pepper/LeftArm_controller, ...).
"""

import os
import re

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, MoveItErrorCodes,
                             PositionConstraint)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

ERROR_NAMES = {v: k for k, v in MoveItErrorCodes.__dict__.items()
               if k.isupper() and isinstance(v, int)}


def named_state(srdf, group, name):
    """Valores de una <group_state> del SRDF."""
    m = re.search(rf'<group_state name="{name}" group="{group}">(.*?)</group_state>', srdf, re.S)
    return {j: float(v) for j, v in re.findall(r'<joint name="([^"]+)" value="([^"]+)"', m.group(1))}


class PepperMoveItDemo:
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, MoveGroup, "move_action")
        srdf_path = os.path.join(get_package_share_directory("pepper_moveit_config"),
                                 "config", "pepper.srdf")
        with open(srdf_path) as f:
            self.srdf = f.read()

    def _execute(self, group, constraints, label):
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.goal_constraints = [constraints]
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result)
        code = result.result().result.error_code.val
        self.node.get_logger().info(f"{label}: {ERROR_NAMES.get(code, code)}")
        return code == MoveItErrorCodes.SUCCESS

    def go_named(self, group, name):
        c = Constraints()
        c.joint_constraints = [
            JointConstraint(joint_name=j, position=v, tolerance_above=0.01,
                            tolerance_below=0.01, weight=1.0)
            for j, v in named_state(self.srdf, group, name).items()]
        return self._execute(group, c, f"{group} -> {name}")

    def go_position(self, group, link, x, y, z, frame="odom", radius=0.02):
        """Lleva `link` a (x, y, z): sólo posición (los brazos de Pepper tienen 5 articulaciones)."""
        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = link
        pc.weight = 1.0
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w = 1.0
        pc.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[radius])],
            primitive_poses=[pose])
        c = Constraints()
        c.position_constraints = [pc]
        return self._execute(group, c, f"{link} -> ({x}, {y}, {z})")


def main():
    rclpy.init()
    node = rclpy.create_node("pepper_moveit_demo")
    demo = PepperMoveItDemo(node)
    if not demo.client.wait_for_server(timeout_sec=20.0):
        node.get_logger().error("move_group no responde: ¿está lanzado moveit_planning_execution?")
        return
    demo.go_named("both_arms", "arms_down")
    demo.go_named("left_arm", "arms_forward")
    demo.go_named("left_hand", "open")
    demo.go_position("left_arm", "l_wrist", 0.25, 0.22, 0.95)
    demo.go_named("left_hand", "closed")
    demo.go_named("head", "center")
    demo.go_named("both_arms", "arms_down")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
