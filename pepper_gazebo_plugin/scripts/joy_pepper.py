#!/usr/bin/env python3
"""Mueve a Pepper con un joystick (joy_pepper.py del Tutorial V8, en rclpy).

Lee /joy (joy_node) y publica en /pepper/cmd_vel. Como la base de Pepper es holonómica,
se usan los tres ejes:

    Stick izquierdo arriba/abajo     -> avance (linear.x)
    Stick izquierdo izquierda/der.   -> lateral (linear.y)
    Stick derecho izquierda/der.     -> giro (angular.z)

Índices por defecto: mando Xbox por USB con joy_node (axes 0/1 = stick izquierdo,
axes 3 = stick derecho). Se pueden cambiar con parámetros:
    axis_linear_x, axis_linear_y, axis_angular, scale_linear, scale_angular,
    enable_button (-1 = sin botón de hombre muerto; p. ej. 4 = LB del Xbox)

Ejecutar:  ros2 launch pepper_gazebo_plugin joy_pepper.launch.py
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyPepper(Node):
    def __init__(self):
        super().__init__("joy_pepper")
        self.axis_linear_x = self.declare_parameter("axis_linear_x", 1).value
        self.axis_linear_y = self.declare_parameter("axis_linear_y", 0).value
        self.axis_angular = self.declare_parameter("axis_angular", 3).value
        # Límites de la base de Pepper (los mismos que el plugin del V8)
        self.scale_linear = self.declare_parameter("scale_linear", 0.55).value
        self.scale_angular = self.declare_parameter("scale_angular", 2.0).value
        self.enable_button = self.declare_parameter("enable_button", -1).value

        self.pub = self.create_publisher(Twist, "/pepper/cmd_vel", 1)
        self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.get_logger().info("joy_pepper: /joy -> /pepper/cmd_vel")

    @staticmethod
    def axis(msg, index):
        return msg.axes[index] if 0 <= index < len(msg.axes) else 0.0

    def joy_callback(self, msg):
        twist = Twist()
        enabled = self.enable_button < 0 or (
            self.enable_button < len(msg.buttons) and msg.buttons[self.enable_button])
        if enabled:
            twist.linear.x = self.scale_linear * self.axis(msg, self.axis_linear_x)
            twist.linear.y = self.scale_linear * self.axis(msg, self.axis_linear_y)
            twist.angular.z = self.scale_angular * self.axis(msg, self.axis_angular)
        self.pub.publish(twist)


def main():
    rclpy.init()
    node = JoyPepper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
