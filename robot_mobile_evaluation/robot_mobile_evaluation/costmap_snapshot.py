"""Image de la carte de coûts globale de Nav2 (E4), superposée au plan de la salle.

Attend `delay` secondes (horloge murale) après la première carte reçue, puis dessine la
dernière dans le repère du monde (grâce à `map_origin`), avec la position vraie du robot, et
s'arrête. À lancer pendant la navigation :
    ros2 run robot_mobile_evaluation costmap_snapshot --ros-args -p use_sim_time:=true \
        -p output:=<image.png> -p title:="LiDAR + profondeur"
"""

import math
import time

import matplotlib
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_mobile_evaluation.evaluate_navigation_trials import EXTRA
from robot_mobile_evaluation.room import draw_room
from robot_mobile_evaluation.trajectory import yaw_from_quaternion

matplotlib.use('Agg')


class CostmapSnapshot(Node):

    def __init__(self):
        super().__init__('costmap_snapshot')
        self.output = self.declare_parameter('output', 'costmap.png').value
        self.title = self.declare_parameter('title', 'Carte de coûts globale').value
        self.delay = self.declare_parameter('delay', 30.0).value
        self.origin = self.declare_parameter('map_origin', [-2.500876, -1.500003, 0.0]).value
        # zone affichée (monde) : [x_min, x_max, y_min, y_max] ; vide = toute la salle
        self.zoom = self.declare_parameter('zoom', [0.0]).value
        self.grid, self.pose, self.first = None, None, None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_grid, latched)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_pose, 10)
        self.create_timer(0.5, self.check_end)

    def on_grid(self, msg):
        self.grid = msg
        self.first = self.first or time.monotonic()

    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def check_end(self):
        if self.first is None or time.monotonic() - self.first < self.delay:
            return
        self.plot()
        raise SystemExit

    def plot(self):
        info = self.grid.info
        cost = np.array(self.grid.data, dtype=float).reshape(info.height, info.width)
        cost[cost < 0] = np.nan                       # inconnu
        # coin de la grille : repère de la carte -> repère du monde (carte sans rotation, D-015)
        x0 = info.origin.position.x + self.origin[0]
        y0 = info.origin.position.y + self.origin[1]
        extent = (x0, x0 + info.width * info.resolution, y0, y0 + info.height * info.resolution)
        fig, ax = plt.subplots(figsize=(8, 6))
        colors = plt.get_cmap('Greys')(np.linspace(0.05, 0.9, 256))
        colors[-1] = (0.55, 0.0, 0.55, 1.0)           # 100 = obstacle (« létal »)
        image = ax.imshow(cost, origin='lower', extent=extent, cmap=ListedColormap(colors),
                          vmin=0, vmax=100, interpolation='nearest')
        draw_room(ax, labels=False)
        for name, x, y, hx, hy in EXTRA:
            ax.add_patch(Rectangle((x - hx, y - hy), 2 * hx, 2 * hy, fill=False, lw=2,
                                   ls='--', color='tab:orange'))
            ax.annotate(name, (x, y + hy + 0.08), ha='center', fontsize=8, color='tab:orange')
        if self.pose is not None:
            x, y, yaw = self.pose
            ax.plot(x, y, 'o', color='tab:blue', ms=8)
            ax.annotate('', xy=(x + 0.4 * math.cos(yaw), y + 0.4 * math.sin(yaw)), xytext=(x, y),
                        arrowprops={'arrowstyle': '->', 'color': 'tab:blue', 'lw': 2})
        if len(self.zoom) == 4:
            ax.set_xlim(self.zoom[0], self.zoom[1])
            ax.set_ylim(self.zoom[2], self.zoom[3])
        fig.colorbar(image, ax=ax, shrink=0.7, label='coût (0 = libre, 100 = obstacle)')
        ax.set_title(self.title, fontsize=10)
        fig.savefig(self.output, dpi=100, bbox_inches='tight')
        self.get_logger().info(f'carte de coûts enregistrée : {self.output}')


def main():
    rclpy.init()
    node = CostmapSnapshot()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
