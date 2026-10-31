"""Un essai de navigation (E4) : envoie un but à Nav2 et mesure le résultat (vérité terrain).

Le but est donné dans le repère du monde ; il est converti dans le repère de la carte grâce à
`map_origin` (pose du robot au début de la cartographie). L'essai réussit si Nav2 annonce le
but atteint ET que le robot est vraiment à moins de `goal_tolerance` du but. Mesures : durée,
longueur du trajet, marge minimale aux obstacles de la salle et aux obstacles ajoutés (non
cartographiés) ; une marge négative signifie un contact (robot assimilé à un disque).

Écrit result.json et ground_truth.tum dans output_dir, puis s'arrête.
"""

import json
import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from robot_mobile_evaluation.room import clearance, distance_to_box
from robot_mobile_evaluation.trajectory import write_tum, yaw_from_quaternion
from tf2_ros import Buffer, TransformListener

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'atteint', GoalStatus.STATUS_ABORTED: 'abandonné',
          GoalStatus.STATUS_CANCELED: 'annulé'}


class NavigationTrial(Node):

    def __init__(self):
        super().__init__('navigation_trial')
        self.output = Path(self.declare_parameter('output_dir', '').value)
        self.scenario = self.declare_parameter('scenario', '').value
        self.goal = self.declare_parameter('goal', [0.0, 0.0, 0.0]).value   # x, y, cap (monde)
        self.origin = self.declare_parameter('map_origin', [0.0, 0.0, 0.0]).value
        self.timeout = self.declare_parameter('timeout', 180.0).value
        # délai pour que Nav2 soit prêt et le robot localisé, avant d'abandonner l'essai
        self.startup_timeout = self.declare_parameter('startup_timeout', 90.0).value
        self.tolerance = self.declare_parameter('goal_tolerance', 0.30).value
        self.radius = self.declare_parameter('robot_radius', 0.21).value
        # obstacles ajoutés : liste à plat [x, y, demi-largeur x, demi-largeur y, ...]
        extra = self.declare_parameter('extra_obstacles', [0.0]).value
        self.extra = [tuple(extra[i:i + 4]) for i in range(0, len(extra) - 3, 4)]
        if not str(self.output):
            raise ValueError('le paramètre output_dir est obligatoire')

        self.truth = []
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, 50)
        self.phase, self.start, self.status, self.goal_handle = 'wait', None, None, None
        self.created = None
        self.create_timer(0.2, self.step)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, msg):
        if self.start is None:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.truth.append((t, p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)))

    def goal_in_map(self):
        ox, oy, oyaw = self.origin
        gx, gy, gyaw = self.goal
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        return c * (gx - ox) - s * (gy - oy), s * (gx - ox) + c * (gy - oy), gyaw - oyaw

    def step(self):
        if self.phase == 'wait':
            self.created = self.created or self.now()
            # Nav2 prêt et robot localisé (transformation map -> base_footprint disponible)
            if not self.client.server_is_ready() or not self.tf_buffer.can_transform(
                    'map', 'base_footprint', Time()):
                if self.now() - self.created > self.startup_timeout:
                    self.start, self.status = self.created, 'Nav2 non prêt'
                    self.finish()
                return
            x, y, yaw = self.goal_in_map()
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = (
                math.sin(yaw / 2), math.cos(yaw / 2))
            self.start = self.now()
            self.client.send_goal_async(goal).add_done_callback(self.on_accepted)
            self.phase = 'running'
            self.get_logger().info(
                f'{self.scenario} : but envoyé ({x:.2f}, {y:.2f}) dans la carte')
        elif self.phase == 'running' and self.now() - self.start > self.timeout:
            self.status = 'délai dépassé'
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.finish()

    def on_accepted(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.status = 'refusé'
            self.finish()
            return
        self.goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        if self.phase == 'running':
            self.status = STATUS.get(future.result().status, str(future.result().status))
            self.finish()

    def finish(self):
        self.phase = 'done'
        self.output.mkdir(parents=True, exist_ok=True)
        write_tum(self.output / 'ground_truth.tum', self.truth)
        _, x, y, _ = self.truth[-1] if self.truth else (0, math.nan, math.nan, 0)
        error = math.hypot(x - self.goal[0], y - self.goal[1])
        room_margin = min((clearance(px, py) for _, px, py, _ in self.truth),
                          default=math.nan) - self.radius
        extra_margin = min((distance_to_box(px, py, *box) for _, px, py, _ in self.truth
                            for box in self.extra), default=math.nan) - self.radius
        path = sum(math.hypot(q[1] - p[1], q[2] - p[2])
                   for p, q in zip(self.truth, self.truth[1:]))
        result = {
            'scenario': self.scenario,
            'nav2_status': self.status,
            'success': self.status == 'atteint' and error < self.tolerance,
            'duration': self.now() - self.start,
            'final_goal_error': error,
            'path_length': path,
            'min_room_margin': room_margin,
            'min_extra_obstacle_margin': extra_margin,
            'collision': bool(room_margin < 0.0 or extra_margin < 0.0),
            'start': list(self.truth[0][1:]) if self.truth else None,
            'goal': list(self.goal),
        }
        (self.output / 'result.json').write_text(json.dumps(result, indent=2))
        self.get_logger().info(
            f'{self.scenario} : {self.status}, écart au but {error:.2f} m, '
            f'marge obstacles ajoutés {extra_margin:.2f} m, {result["duration"]:.1f} s')
        raise SystemExit


def main():
    rclpy.init()
    node = NavigationTrial()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
