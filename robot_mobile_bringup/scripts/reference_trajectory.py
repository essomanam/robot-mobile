#!/usr/bin/env python3
"""Fait parcourir au robot une trajectoire de référence définie par paramètres.

La trajectoire est une suite de segments (vitesse linéaire, vitesse angulaire, durée)
exécutés en boucle ouverte, en temps simulé. Elle n'a pas besoin d'être parfaite :
la vraie trajectoire est enregistrée grâce à la vérité terrain du simulateur. Son rôle
est d'être **identique d'un essai à l'autre**, ce qu'une téléopération à la main ne
permet pas.

    ros2 run robot_mobile_bringup reference_trajectory.py --ros-args \
        --params-file <bringup>/config/reference_trajectory.yaml
"""

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node


class ReferenceTrajectory(Node):

    def __init__(self):
        super().__init__('reference_trajectory')
        self.linear = self.declare_parameter('linear_speeds', [0.0]).value
        self.angular = self.declare_parameter('angular_speeds', [0.0]).value
        self.durations = self.declare_parameter('durations', [0.0]).value
        self.start_delay = self.declare_parameter('start_delay', 2.0).value
        rate = self.declare_parameter('rate_hz', 20.0).value
        if not len(self.linear) == len(self.angular) == len(self.durations):
            raise ValueError(
                'linear_speeds, angular_speeds et durations doivent avoir la même taille')

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.start_time = None
        self.segment = -1
        self.finished = False
        self.timer = self.create_timer(1.0 / rate, self.step)

    def publish(self, v, w):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_footprint'
        cmd.twist.linear.x = v
        cmd.twist.angular.z = w
        self.publisher.publish(cmd)

    def step(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:  # horloge simulée pas encore reçue
            return
        if self.start_time is None:
            self.start_time = now
            self.get_logger().info(
                f'{len(self.durations)} segments, {sum(self.durations):.1f} s, '
                f'départ dans {self.start_delay:.1f} s')
        elapsed = (now - self.start_time).nanoseconds * 1e-9 - self.start_delay
        if elapsed < 0:
            self.publish(0.0, 0.0)
            return

        # segment courant d'après le temps écoulé
        t, index = 0.0, None
        for i, d in enumerate(self.durations):
            if elapsed < t + d:
                index = i
                break
            t += d
        if index is None:
            self.publish(0.0, 0.0)
            self.get_logger().info('Trajectoire terminée')
            self.finished = True
            raise SystemExit
        if index != self.segment:
            self.segment = index
            self.get_logger().info(
                f'Segment {index + 1}/{len(self.durations)} : v={self.linear[index]:.2f} m/s, '
                f'w={self.angular[index]:.3f} rad/s, {self.durations[index]:.2f} s')
        self.publish(self.linear[index], self.angular[index])


def main():
    rclpy.init()
    node = ReferenceTrajectory()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
