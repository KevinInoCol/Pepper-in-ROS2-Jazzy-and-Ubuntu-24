#!/usr/bin/env python3
"""Fusiona los 3 láseres de Pepper en /pepper/laser_2 (laser_publisher.py del V8, en rclpy).

Autor original: Sammy Pfeiffer (pepper_virtual). Port a ROS 2 para el Pepper Tutorial V9.

Igual que en el V8:
  - Sincroniza /pepper/scan_right, /pepper/scan_front y /pepper/scan_left.
  - Proyecta cada scan a puntos y los lleva a base_footprint con la TF.
  - Construye un LaserScan en base_footprint de ±half_max_angle (120°) con 61*8 = 488 rayos:
    cada punto cae en el rayo de su ángulo y guarda su distancia a base_footprint. Los rayos
    sin punto quedan en NaN.
  - Publica también las nubes que publicaba el V8: /cloud (los 3 láseres en base_footprint),
    /cloudl y /cloudr (láser izquierdo y derecho en su frame), /cloud_redone (laser_2
    reproyectado) y /cloud_rereprojected (depuración del V8).

Cambios respecto al V8: los parámetros de ddynamic_reconfigure (half_max_angle) son
parámetros de ROS 2; el sincronizador es aproximado (las tres medidas llegan en el mismo paso
de simulación, pero así no se pierde ninguna); sin los print de depuración.
"""

import math

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

BASE_FRAME = "base_footprint"
NUM_RAYS = 61 * 8
NAN_GAP = 8  # puntos NaN entre láseres en /cloud, como el V8


def scan_to_points(scan):
    """Puntos (N, 3) de un LaserScan en su propio frame (incluye inf/NaN, como el V8)."""
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    with np.errstate(invalid="ignore"):
        return np.column_stack(
            [ranges * np.cos(angles), ranges * np.sin(angles), np.zeros_like(ranges)])


def transform_matrix(tf):
    """Matriz 4x4 de un geometry_msgs/TransformStamped."""
    t = tf.transform.translation
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t.x],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t.y],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t.z],
        [0.0, 0.0, 0.0, 1.0],
    ])


class LaserPublisher(Node):
    def __init__(self):
        super().__init__("laser_publisher")
        self.half_max_angle = self.declare_parameter("half_max_angle", 120.0).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.all_laser_pub = self.create_publisher(LaserScan, "/pepper/laser_2", 1)
        self.pc_pub = self.create_publisher(PointCloud2, "/cloud", 1)
        self.pcl_pub = self.create_publisher(PointCloud2, "/cloudl", 1)
        self.pcr_pub = self.create_publisher(PointCloud2, "/cloudr", 1)
        self.pc_redone_pub = self.create_publisher(PointCloud2, "/cloud_redone", 1)
        self.pc_rereprojected_pub = self.create_publisher(PointCloud2, "/cloud_rereprojected", 1)

        subs = [Subscriber(self, LaserScan, topic, qos_profile=qos_profile_sensor_data)
                for topic in ("/pepper/scan_left", "/pepper/scan_front", "/pepper/scan_right")]
        self.ts = ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.05)
        self.ts.registerCallback(self.scan_cb)
        self.get_logger().info("laser_publisher: scan_left/front/right -> /pepper/laser_2")

    def to_base(self, scan):
        """Puntos del scan en base_footprint, o None si aún no hay TF."""
        try:
            tf = self.tf_buffer.lookup_transform(BASE_FRAME, scan.header.frame_id, Time())
        except TransformException as ex:
            self.get_logger().warn(f"Sin TF {BASE_FRAME} <- {scan.header.frame_id}: {ex}",
                                   throttle_duration_sec=5.0)
            return None
        points = scan_to_points(scan)
        homogeneous = np.column_stack([points, np.ones(len(points))])
        with np.errstate(invalid="ignore"):
            return (transform_matrix(tf) @ homogeneous.T).T[:, :3]

    def scan_cb(self, left, front, right):
        header = front.header
        self.pcl_pub.publish(point_cloud2.create_cloud_xyz32(left.header, scan_to_points(left)))
        self.pcr_pub.publish(point_cloud2.create_cloud_xyz32(right.header, scan_to_points(right)))

        parts = [self.to_base(right), self.to_base(front), self.to_base(left)]
        if any(p is None for p in parts):
            return
        gap = np.full((NAN_GAP, 3), np.nan)
        cloud = np.vstack([parts[0], gap, parts[1], gap, parts[2]])

        header.frame_id = BASE_FRAME
        self.pc_pub.publish(point_cloud2.create_cloud_xyz32(header, cloud))

        ranges, angle_min, angle_max, angle_increment = self.pc_to_laser(cloud, header)
        msg = LaserScan()
        msg.header = header
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = angle_increment
        msg.range_min = 0.1
        msg.range_max = 7.0
        msg.ranges = ranges
        self.all_laser_pub.publish(msg)

        self.pc_redone_pub.publish(point_cloud2.create_cloud_xyz32(header, scan_to_points(msg)))

    def pc_to_laser(self, cloud, header):
        """Rayos de laser_2 a partir de la nube en base_footprint (misma lógica que el V8)."""
        min_angle = -math.radians(self.half_max_angle)
        max_angle = math.radians(self.half_max_angle)
        angle_increment = (math.radians(self.half_max_angle) * 2.0) / float(NUM_RAYS)
        ranges = [float("nan")] * NUM_RAYS
        rereprojected = []
        for idx, (px, py, _) in enumerate(cloud):
            dist = math.hypot(px, py)
            # Depuración del V8: cada punto colocado según su índice, no según su ángulo
            a = idx * angle_increment + min_angle
            rereprojected.append((dist * math.cos(a), dist * math.sin(a), 0.0))
            angle = math.atan2(py, px)
            if math.isnan(angle):
                continue
            closest = int((angle - min_angle) / angle_increment)
            closest = min(max(closest, 0), NUM_RAYS - 1)
            ranges[closest] = dist
        self.pc_rereprojected_pub.publish(point_cloud2.create_cloud_xyz32(header, rereprojected))
        return ranges, min_angle, max_angle, angle_increment


def main():
    rclpy.init()
    node = LaserPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
