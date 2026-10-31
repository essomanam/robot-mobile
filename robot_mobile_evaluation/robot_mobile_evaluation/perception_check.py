"""Mesure la détection de la cible (E3) : fréquence, taux de détection, latence, distance.

Compte, pendant `duration` secondes (horloge murale), les images couleur reçues et les messages
de /target/detections ; pour chaque message, la latence est l'écart entre l'instant de l'image
(en-tête) et l'instant de réception (temps simulé). À lancer robot immobile, avec la simulation
et target_detector déjà démarrés :
    ros2 run robot_mobile_evaluation perception_check --ros-args -p use_sim_time:=true \
        -p output:=<fichier.json>

Avec la cible en vue, on attend 100 % de détections ; cible hors du champ, 0 %.
"""

import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray


class PerceptionCheck(Node):

    def __init__(self):
        super().__init__('perception_check')
        self.duration = self.declare_parameter('duration', 30.0).value
        self.output = self.declare_parameter('output', '').value
        self.images = 0
        self.messages = 0
        self.latencies, self.distances = [], []
        self.start = None
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image, 10)
        self.create_subscription(Detection3DArray, '/target/detections', self.on_detections, 10)
        self.create_timer(0.5, self.check_end)
        self.done = False

    def on_image(self, msg):
        if self.start is None:
            self.start = time.monotonic()
        self.images += 1

    def on_detections(self, msg):
        if self.start is None:
            return
        self.messages += 1
        stamp = Time.from_msg(msg.header.stamp)
        self.latencies.append((self.get_clock().now() - stamp).nanoseconds * 1e-6)
        if msg.detections:
            self.distances.append(msg.detections[0].bbox.center.position.z)

    def check_end(self):
        if self.start is None or time.monotonic() - self.start < self.duration or self.done:
            return
        self.done = True
        elapsed = time.monotonic() - self.start
        lat = np.array(self.latencies) if self.latencies else np.array([float('nan')])
        result = {
            'duration_s': elapsed,
            'image_rate_hz': self.images / elapsed,
            'detection_message_rate_hz': self.messages / elapsed,
            'messages_per_image': self.messages / max(self.images, 1),
            'detection_ratio': len(self.distances) / max(self.messages, 1),
            'latency_ms_mean': float(np.mean(lat)),
            'latency_ms_p95': float(np.percentile(lat, 95)),
            'latency_ms_max': float(np.max(lat)),
            'distance_m_mean': float(np.mean(self.distances)) if self.distances else None,
            'distance_m_std': float(np.std(self.distances)) if self.distances else None,
        }
        for key, value in result.items():
            self.get_logger().info(f'{key} = {value:.3f}' if value is not None else f'{key} = -')
        if self.output:
            Path(self.output).expanduser().write_text(json.dumps(result, indent=2) + '\n')
        raise SystemExit


def main():
    rclpy.init()
    node = PerceptionCheck()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
