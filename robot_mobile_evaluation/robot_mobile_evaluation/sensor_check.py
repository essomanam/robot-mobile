"""Vérifie que les capteurs simulés mesurent ce que la géométrie de la salle prévoit.

Principe : la salle est un rectangle connu et le simulateur donne la position exacte
du robot (vérité terrain). Pour quelques rayons du LiDAR et pour le pixel central de
l'image de profondeur, on calcule la distance attendue jusqu'au mur et on la compare
à la mesure, moyennée sur plusieurs acquisitions : la moyenne de l'écart est le
**biais**, son écart-type le **bruit**. Les positions des capteurs sur le robot sont
lues dans TF : un biais révèle donc aussi une erreur de description (URDF / SDF).

À lancer pendant la simulation, robot immobile, avec les rayons testés dégagés
(c'est le cas à la position de départ par défaut) :
    ros2 run robot_mobile_evaluation sensor_check --ros-args -p use_sim_time:=true
"""

import math

from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from robot_mobile_evaluation.room import ray_to_wall
from robot_mobile_evaluation.trajectory import yaw_from_quaternion
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return yaw_from_quaternion(q.x, q.y, q.z, q.w)


def apply_yaw(x, y, yaw):
    return x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw)


class SensorCheck(Node):

    def __init__(self):
        super().__init__('sensor_check')
        # Champ horizontal déclaré dans le modèle SDF (rad), pour vérifier la calibration
        self.hfov = self.declare_parameter('camera_hfov', 1.2112).value
        # Nombre d'acquisitions moyennées pour chaque capteur
        self.samples = self.declare_parameter('samples', 50).value

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scans, self.depths = [], []
        self.info = self.pose = None

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_raw', self.on_depth, 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.on_info, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_pose, 10)
        self.timer = self.create_timer(1.0, self.try_check)
        self.done = False

    def on_scan(self, msg):
        if len(self.scans) < self.samples:
            self.scans.append(msg)

    def on_depth(self, msg):
        if len(self.depths) < self.samples:
            self.depths.append(msg)

    def on_info(self, msg):
        self.info = msg

    def on_pose(self, msg):
        self.pose = msg

    def sensor_pose(self, frame):
        """Position (x, y) et cap du capteur dans le monde, via TF + vérité terrain."""
        tf = self.tf_buffer.lookup_transform('base_footprint', frame, Time())
        p = self.pose.pose.pose
        yaw = yaw_of(p.orientation)
        ox, oy = apply_yaw(tf.transform.translation.x, tf.transform.translation.y, yaw)
        return p.position.x + ox, p.position.y + oy, yaw, tf

    def try_check(self):
        if self.done or self.info is None or self.pose is None:
            return
        if len(self.scans) < self.samples or len(self.depths) < self.samples:
            return
        scan, depth_msg = self.scans[0], self.depths[0]
        if not self.tf_buffer.can_transform('base_footprint', scan.header.frame_id, Time()):
            return
        self.done = True
        log = self.get_logger().info
        p = self.pose.pose.pose
        log(f'Pose vérité terrain : x={p.position.x:.3f} m  y={p.position.y:.3f} m  '
            f'cap={math.degrees(yaw_of(p.orientation)):.1f}°')
        log(f'Repères : scan={scan.header.frame_id}  profondeur={depth_msg.header.frame_id}  '
            f'camera_info={self.info.header.frame_id}')
        log(f'Moyennes sur {self.samples} acquisitions par capteur')

        # LiDAR : rayons avant, gauche, arrière, droite
        lx, ly, yaw, _ = self.sensor_pose(scan.header.frame_id)
        log('LiDAR — rayon : attendu / moyenne mesurée / biais / bruit (écart-type)')
        for name, beam in (('avant', 0.0), ('gauche', math.pi / 2),
                           ('arrière', math.pi), ('droite', 3 * math.pi / 2)):
            i = round((beam - scan.angle_min) / scan.angle_increment)
            angle = scan.angle_min + i * scan.angle_increment
            expected = ray_to_wall(lx, ly, yaw + angle)
            values = np.array([s.ranges[i] for s in self.scans])
            if expected > scan.range_max:
                log(f'  {name:8s}: > {scan.range_max:.2f} m (hors portée) / '
                    f'{np.isinf(values).sum()}/{len(values)} mesures infinies')
            else:
                log(f'  {name:8s}: {expected:.3f} / {values.mean():.3f} / '
                    f'{values.mean() - expected:+.4f} / {values.std():.4f} m')

        # Caméra : calibration, puis profondeur du pixel central (axe optique)
        k = self.info.k
        fx_theory = (self.info.width / 2) / math.tan(self.hfov / 2)
        log(f'Caméra : {self.info.width}x{self.info.height}  fx={k[0]:.1f} (théorique '
            f'{fx_theory:.1f})  fy={k[4]:.1f}  cx={k[2]:.1f}  cy={k[5]:.1f}')
        cx, cy, yaw, tf = self.sensor_pose(depth_msg.header.frame_id)
        u, v = int(round(k[2])), int(round(k[5]))
        values = np.array([
            float(self.bridge.imgmsg_to_cv2(d)[v, u]) for d in self.depths])
        # direction de l'axe optique (z du repère optique) exprimée dans base_footprint
        q = tf.transform.rotation
        zx = 2.0 * (q.x * q.z + q.w * q.y)
        zy = 2.0 * (q.y * q.z - q.w * q.x)
        axis_yaw = yaw + math.atan2(zy, zx)
        expected = ray_to_wall(cx, cy, axis_yaw)
        log(f'Profondeur au centre ({depth_msg.encoding}) : attendu {expected:.3f} / '
            f'moyenne {values.mean():.3f} / biais {values.mean() - expected:+.4f} / '
            f'bruit {values.std():.4f} m')
        raise SystemExit


def main():
    rclpy.init()
    node = SensorCheck()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
