"""Observe un essai d'approche de cible (E3) et en mesure le résultat avec la vérité terrain.

L'essai réussit quand le comportement reste dans l'état ARRIVEE pendant `arrive_hold`
secondes ; il échoue au bout de `timeout` secondes (temps simulé). À la fin, le nœud écrit
`result.json` et la trajectoire vraie (`ground_truth.tum`) dans `output_dir`, puis s'arrête
(ce qui arrête aussi le lancement de l'essai).

Mesures : réussite, durée, temps passé en recherche, distance finale du centre du robot à la
face de la cible le long de la ligne de visée (comme la mesure le robot ; comparée à la distance
voulue), erreur d'orientation finale vers le centre de la cible, longueur du trajet, marge
minimale aux obstacles (négative = collision).
"""

import json
import math
from pathlib import Path

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_mobile_evaluation.room import clearance, distance_to_box, sight_distance_to_box
from robot_mobile_evaluation.trajectory import write_tum, yaw_from_quaternion
from std_msgs.msg import String


class TrialMonitor(Node):

    def __init__(self):
        super().__init__('trial_monitor')
        self.output = Path(self.declare_parameter('output_dir', '').value)
        self.scenario = self.declare_parameter('scenario', '').value
        self.timeout = self.declare_parameter('timeout', 120.0).value
        self.arrive_hold = self.declare_parameter('arrive_hold', 2.0).value
        self.target = (self.declare_parameter('target_x', 1.7).value,
                       self.declare_parameter('target_y', -0.3).value,
                       self.declare_parameter('target_half_size', 0.2).value)
        self.desired = self.declare_parameter('desired_distance', 0.6).value
        self.robot_radius = self.declare_parameter('robot_radius', 0.21).value
        if not str(self.output):
            raise ValueError('le paramètre output_dir est obligatoire')

        self.truth, self.states = [], []   # states : (instant du changement, état)
        self.start = self.arrived_since = self.first_arrival = None
        self.state = ''
        self.done = False
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, 50)
        self.create_subscription(String, '/target_follower/state', self.on_state, 50)
        self.create_timer(0.1, self.check)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.truth.append((t, p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)))

    def on_state(self, msg):
        t = self.now()
        if self.start is None and msg.data != 'CAPTEURS ABSENTS':
            self.start = t                       # le comportement a démarré
        if msg.data != self.state:
            self.states.append((t, msg.data))
        self.state = msg.data
        if msg.data == 'ARRIVEE':
            if self.arrived_since is None:
                self.arrived_since = t
                self.first_arrival = self.first_arrival or t
        else:
            self.arrived_since = None

    def check(self):
        if self.done or self.start is None:
            return
        t = self.now()
        success = self.arrived_since is not None and t - self.arrived_since >= self.arrive_hold
        if success or t - self.start > self.timeout:
            self.done = True
            self.save(success, t)
            raise SystemExit

    def save(self, success, t_end):
        self.output.mkdir(parents=True, exist_ok=True)
        write_tum(self.output / 'ground_truth.tum', self.truth)
        tx, ty, half = self.target
        _, x, y, yaw = self.truth[-1]
        face = sight_distance_to_box(x, y, tx, ty, half, half)
        # durée passée dans chaque état, d'après la chronologie des changements
        times = [t for t, _ in self.states[1:]] + [t_end]
        search = sum(end - t for (t, s), end in zip(self.states, times) if s == 'RECHERCHE')
        bearing = math.degrees(math.remainder(math.atan2(ty - y, tx - x) - yaw, 2 * math.pi))
        path = sum(math.hypot(q[1] - p[1], q[2] - p[2])
                   for p, q in zip(self.truth, self.truth[1:]))
        margin = min(clearance(px, py) for _, px, py, _ in self.truth) - self.robot_radius
        target_margin = min(distance_to_box(px, py, tx, ty, half, half)
                            for _, px, py, _ in self.truth) - self.robot_radius
        result = {
            'scenario': self.scenario,
            'success': bool(success),
            'time_to_arrival': (self.first_arrival - self.start) if self.first_arrival else None,
            'duration': t_end - self.start,
            'search_time': search,
            'start': list(self.truth[0][1:]),
            'final_face_distance': face,
            'distance_error': face - self.desired,
            'final_bearing_error_deg': bearing,
            'path_length': path,
            'min_obstacle_margin': margin,
            'min_target_margin': target_margin,
            'states': [s for _, s in self.states],
            'state_changes': [[t - self.start, s] for t, s in self.states],
        }
        (self.output / 'result.json').write_text(json.dumps(result, indent=2))
        self.get_logger().info(
            f'{self.scenario} : {"RÉUSSI" if success else "ÉCHEC"} en {result["duration"]:.1f} s, '
            f'distance finale {face:.3f} m, cap {bearing:+.1f}°, marge {margin:.2f} m')


def main():
    rclpy.init()
    node = TrialMonitor()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
