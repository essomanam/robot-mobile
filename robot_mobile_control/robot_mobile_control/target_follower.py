"""Nœud de suivi de cible : détections de la caméra -> commandes de vitesse.

Entrées : /target/detections (vision_msgs/Detection3DArray, repère optique de la caméra),
          /scan (sécurité), TF (caméra -> base_footprint)
Sorties : /cmd_vel (TwistStamped), /target_follower/state (std_msgs/String)

Sûreté : sans détections ni scan récents, le robot s'arrête ; un obstacle trop proche
devant annule l'avance ; un arrêt est envoyé à la fermeture du nœud.

    ros2 run robot_mobile_control target_follower --ros-args \
        --params-file <bringup>/config/control/target_follower.yaml
"""

import math

from geometry_msgs.msg import PointStamped, TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_mobile_control.approach import ApproachController, ApproachParams
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray


class TargetFollower(Node):

    def __init__(self):
        super().__init__('target_follower')
        d = ApproachParams()
        params = ApproachParams(
            desired_distance=self.declare_parameter('desired_distance', d.desired_distance).value,
            distance_tolerance=self.declare_parameter(
                'distance_tolerance', d.distance_tolerance).value,
            align_angle=math.radians(self.declare_parameter('align_angle_deg', 20.0).value),
            arrive_angle=math.radians(self.declare_parameter('arrive_angle_deg', 5.0).value),
            k_v=self.declare_parameter('k_v', d.k_v).value,
            k_w=self.declare_parameter('k_w', d.k_w).value,
            max_v=self.declare_parameter('max_v', d.max_v).value,
            max_w=self.declare_parameter('max_w', d.max_w).value,
            search_w=self.declare_parameter('search_w', d.search_w).value,
            lost_timeout=self.declare_parameter('lost_timeout', d.lost_timeout).value)
        self.class_id = self.declare_parameter('class_id', 'red_target').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.safety_distance = self.declare_parameter('safety_distance', 0.30).value
        self.safety_angle = math.radians(self.declare_parameter('safety_angle_deg', 30.0).value)
        self.timeout = self.declare_parameter('sensor_timeout', 0.5).value
        rate = self.declare_parameter('rate_hz', 20.0).value

        self.controller = ApproachController(params)
        self.target = None            # (angle, distance) de la dernière détection
        self.last_detection = None    # instant de la dernière liste de détections reçue
        self.front_distance = math.inf
        self.last_scan = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Detection3DArray, '/target/detections', self.on_detections, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/target_follower/state', 10)
        self.create_timer(1.0 / rate, self.step)

    def on_detections(self, msg):
        self.last_detection = self.get_clock().now()
        self.target = None
        for det in msg.detections:
            if det.id != self.class_id:
                continue
            point = PointStamped()
            point.header = msg.header
            point.point = det.bbox.center.position
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, msg.header.frame_id, msg.header.stamp,
                    timeout=Duration(seconds=0.1))
            except TransformException as exc:
                self.get_logger().warn(f'TF indisponible : {exc}', throttle_duration_sec=2.0)
                return
            p = do_transform_point(point, tf).point
            self.target = (math.atan2(p.y, p.x), math.hypot(p.x, p.y))
            return

    def on_scan(self, msg):
        front = [r for i, r in enumerate(msg.ranges)
                 if msg.range_min <= r <= msg.range_max and math.isfinite(r)
                 and abs(math.remainder(msg.angle_min + i * msg.angle_increment,
                                        2 * math.pi)) < self.safety_angle]
        self.front_distance = min(front) if front else math.inf
        self.last_scan = self.get_clock().now()

    def fresh(self, stamp, now):
        return stamp is not None and (now - stamp).nanoseconds * 1e-9 < self.timeout

    def step(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:  # temps simulé pas encore reçu
            return
        if not (self.fresh(self.last_detection, now) and self.fresh(self.last_scan, now)):
            self.publish(0.0, 0.0, 'CAPTEURS ABSENTS')
            return
        v, w = self.controller.update(now.nanoseconds * 1e-9, self.target,
                                      obstacle_ahead=self.front_distance < self.safety_distance)
        self.publish(v, w, self.controller.state)

    def publish(self, v, w, state):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.twist.linear.x, cmd.twist.angular.z = v, w
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=state))


def main():
    rclpy.init()
    node = TargetFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish(0.0, 0.0, 'ARRET')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
