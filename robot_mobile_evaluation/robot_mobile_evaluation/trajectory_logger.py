"""Enregistre les trajectoires et les sorties d'un SLAM pendant le rejeu d'un jeu de données.

Fichiers écrits dans `output_dir` à l'arrêt du nœud :
  ground_truth.tum   vérité terrain (/ground_truth/odom), repère du monde
  odometry.tum       odométrie des roues (/odom), repère odom
  slam.tum           estimation EN LIGNE du SLAM : TF map -> base_footprint, telle que le
                     robot la connaît à chaque instant (c'est elle que la navigation utilise)
  map.png, map.yaml  dernière carte d'occupation reçue (/map), convention map_server
  stats.json         étiquette, fermetures de boucle et temps de calcul (RTAB-Map)

Le graphe optimisé de RTAB-Map est exporté après l'essai depuis sa base de données
(evaluate_slam), car seule la base conserve l'instant de chaque nœud.

    ros2 run robot_mobile_evaluation trajectory_logger --ros-args \
        -p output_dir:=/tmp/essai -p label:=rtabmap_rgbd -p use_sim_time:=true
"""

import json
import math
from pathlib import Path

import cv2
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from robot_mobile_evaluation.trajectory import write_tum, yaw_from_quaternion
from tf2_ros import Buffer, TransformException, TransformListener

try:  # messages RTAB-Map : facultatifs (absents si l'on évalue slam_toolbox seul)
    from rtabmap_msgs.msg import Info
except ImportError:
    Info = None


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_row(stamp, position, orientation):
    q = orientation
    return (stamp_seconds(stamp), position.x, position.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w))


class TrajectoryLogger(Node):

    def __init__(self):
        super().__init__('trajectory_logger')
        self.output = Path(self.declare_parameter('output_dir', '').value)
        self.label = self.declare_parameter('label', '').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        rate = self.declare_parameter('rate_hz', 20.0).value
        if not str(self.output):
            raise ValueError('le paramètre output_dir est obligatoire')

        self.truth, self.odom, self.slam = [], [], []
        self.last_map = None
        # fermetures de boucle acceptées : (nœud, nœud) -> nombre de points cohérents (inliers)
        self.loop_closures, self.proximities, self.timings = {}, set(), []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, 50)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        if Info is not None:
            self.create_subscription(Info, '/rtabmap/info', self.on_info, 10)
        self.create_timer(1.0 / rate, self.sample_slam)

    def on_truth(self, msg):
        self.truth.append(pose_row(msg.header.stamp, msg.pose.pose.position,
                                   msg.pose.pose.orientation))

    def on_odom(self, msg):
        self.odom.append(pose_row(msg.header.stamp, msg.pose.pose.position,
                                  msg.pose.pose.orientation))

    def on_map(self, msg):
        self.last_map = msg

    def on_info(self, msg):
        stats = dict(zip(msg.stats_keys, msg.stats_values))
        if msg.loop_closure_id > 0:
            self.loop_closures[(msg.ref_id, msg.loop_closure_id)] = \
                stats.get('Loop/Visual_inliers/')
        if msg.proximity_detection_id > 0:
            self.proximities.add((msg.ref_id, msg.proximity_detection_id))
        if 'Timing/Total/ms' in stats:
            self.timings.append(stats['Timing/Total/ms'])

    def sample_slam(self):
        """Dernière pose map -> base_footprint connue ; nouvelle ligne si l'instant a changé."""
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return
        row = pose_row(tf.header.stamp, tf.transform.translation, tf.transform.rotation)
        if not self.slam or row[0] > self.slam[-1][0]:
            self.slam.append(row)

    def save(self):
        self.output.mkdir(parents=True, exist_ok=True)
        write_tum(self.output / 'ground_truth.tum', self.truth)
        write_tum(self.output / 'odometry.tum', self.odom)
        write_tum(self.output / 'slam.tum', self.slam)
        if self.last_map is not None:
            self.save_map(self.last_map)
        stats = {
            'label': self.label,
            'samples': {'ground_truth': len(self.truth), 'odometry': len(self.odom),
                        'slam': len(self.slam)},
            'loop_closures': len(self.loop_closures),
            'loop_closure_inliers': sorted(
                int(v) for v in self.loop_closures.values() if v is not None),
            'proximity_detections': len(self.proximities),
            'timing_ms': ({'mean': float(np.mean(self.timings)),
                           'max': float(np.max(self.timings)),
                           'updates': len(self.timings)} if self.timings else None),
        }
        (self.output / 'stats.json').write_text(json.dumps(stats, indent=2))

    def save_map(self, grid):
        """Carte au format map_server : libre = blanc, occupé = noir, inconnu = gris."""
        info = grid.info
        data = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width)
        image = np.full(data.shape, 205, np.uint8)
        image[data == 0] = 254
        image[data >= 65] = 0
        cv2.imwrite(str(self.output / 'map.png'), np.flipud(image))
        o = info.origin
        yaw = yaw_from_quaternion(o.orientation.x, o.orientation.y,
                                  o.orientation.z, o.orientation.w)
        (self.output / 'map.yaml').write_text(
            'image: map.png\nmode: trinary\n'
            f'resolution: {info.resolution}\n'
            f'origin: [{o.position.x}, {o.position.y}, {yaw if math.isfinite(yaw) else 0.0}]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


def main():
    rclpy.init()
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.get_logger().info(
            f'{node.label} : {len(node.slam)} poses SLAM, {len(node.truth)} vérité terrain '
            f'-> {node.output}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
