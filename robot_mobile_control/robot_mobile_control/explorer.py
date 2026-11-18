"""Nœud d'exploration autonome par frontières : carte en construction -> buts de navigation.

Boucle (toutes les `update_period` secondes) :
  1. lire la dernière carte d'occupation (/map, publiée par RTAB-Map en cartographie) ;
  2. trouver les zones de frontière (frontiers.find_frontiers), en masquant les endroits déjà
     essayés (liste noire) ;
  3. sans but en cours : choisir une zone (frontiers.choose_frontier) et l'envoyer à Nav2
     (action NavigateToPose), orientée du robot vers la frontière pour que la caméra regarde
     l'inconnu ;
  4. avec un but en cours : passer au suivant s'il ne reste plus aucune case frontière près du
     but (exploré en route), l'abandonner s'il dure plus de `goal_timeout` secondes ;
  5. plus aucune frontière : exploration terminée.

Un but atteint, échoué ou trop long est mis sur liste noire : les cases libres à moins de
`blacklist_radius` de cet endroit sont traitées comme occupées dans une COPIE de la carte avant de
chercher les frontières. Sinon le robot pourrait tourner en rond devant une frontière qu'il ne peut
pas « effacer » (vitre, trou du sol, but inaccessible). On ne masque qu'un disque, pas une zone
entière : une grande frontière (tout le pourtour d'une pièce) reste visée ailleurs.

Entrées : /map (nav_msgs/OccupancyGrid), TF map -> base_footprint, action /navigate_to_pose
Sorties : /exploration/state (std_msgs/String), /exploration/frontiers (MarkerArray, pour RViz)

    ros2 run robot_mobile_control explorer --ros-args \
        --params-file <bringup>/config/control/explorer.yaml
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from robot_mobile_control.frontiers import choose_frontier, find_frontiers, FREE
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

EXPLORING, DONE = 'EXPLORATION', 'TERMINEE'


class Explorer(Node):

    def __init__(self):
        super().__init__('explorer')
        self.min_size = self.declare_parameter('min_frontier_size', 10).value
        self.min_distance = self.declare_parameter('min_distance', 0.5).value
        self.goal_timeout = self.declare_parameter('goal_timeout', 60.0).value
        self.blacklist_radius = self.declare_parameter('blacklist_radius', 0.5).value
        self.min_known_area = self.declare_parameter('min_known_area', 1.0).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_footprint').value
        period = self.declare_parameter('update_period', 1.0).value

        self.grid = None              # dernière carte (tableau ligne, colonne)
        self.info = None              # résolution et origine de la carte
        self.goal = None              # (x, y) du but en cours, repère carte
        self.goal_handle = None
        self.goal_sent_at = None
        self.blacklist = []           # (x, y) des buts déjà essayés
        self.goals_sent = 0
        self.started_at = None
        self.no_frontier_steps = 0    # constats successifs « plus de frontière »
        self.state = EXPLORING

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # « Transient local », comme la couche statique de Nav2 : RTAB-Map publie la carte en
        # mode « latch » et ne la republie qu'à sa prochaine mise à jour, c'est-à-dire quand le
        # robot bouge. Un abonné « volatile » arrivé après la première carte ne recevrait rien,
        # et le robot attendrait indéfiniment (constaté au premier essai).
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.state_pub = self.create_publisher(String, '/exploration/state', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', 1)
        self.create_timer(period, self.step)

    # ------------------------------------------------------------------ entrées

    def on_map(self, msg):
        self.info = msg.info
        self.grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)

    def robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
        except TransformException:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    # Conversions case <-> carte (origine de la grille sans rotation, comme celle de RTAB-Map)
    def cell_to_xy(self, cell):
        o, r = self.info.origin.position, self.info.resolution
        return o.x + (cell[1] + 0.5) * r, o.y + (cell[0] + 0.5) * r

    def xy_to_cell(self, x, y):
        o, r = self.info.origin.position, self.info.resolution
        return int((y - o.y) // r), int((x - o.x) // r)

    # ------------------------------------------------------------------ boucle

    def step(self):
        self.state_pub.publish(String(data=self.state))
        if self.state == DONE or self.grid is None:
            return
        robot = self.robot_xy()
        if robot is None:
            self.get_logger().info('En attente de la position du robot (TF map -> base)',
                                   throttle_duration_sec=5.0)
            return
        if not self.nav.server_is_ready():
            self.get_logger().info('En attente de Nav2…', throttle_duration_sec=5.0)
            return
        now = self.get_clock().now()
        if self.started_at is None:
            self.started_at = now

        zones = find_frontiers(self.masked_grid(), self.min_size)
        self.publish_markers(zones)

        if self.goal is not None:
            if now - self.goal_sent_at > Duration(seconds=self.goal_timeout):
                self.get_logger().warning('But trop long : abandonné')
                self.drop_goal(blacklist=True)
            elif not self.near_frontier(self.goal, zones):
                self.get_logger().info('Frontière explorée en route : but suivant')
                self.drop_goal(blacklist=False)
            return

        if not zones:
            # Une carte encore vide (démarrage) n'a pas de frontière non plus : on attend d'avoir
            # au moins min_known_area m² de libre, et 3 constats de suite avant de conclure.
            free_area = np.count_nonzero(self.grid == FREE) * self.info.resolution ** 2
            if free_area >= self.min_known_area:
                self.no_frontier_steps += 1
                if self.no_frontier_steps >= 3:
                    self.finish(now)
            return
        self.no_frontier_steps = 0
        best = choose_frontier(zones, self.xy_to_cell(*robot), self.info.resolution,
                               self.min_distance)
        self.send_goal(self.cell_to_xy(best.goal), robot, best.size)

    def masked_grid(self):
        """Renvoie une copie de la carte où les abords des buts déjà essayés sont occupés."""
        if not self.blacklist:
            return self.grid
        grid = self.grid.copy()
        rows, cols = np.ogrid[:grid.shape[0], :grid.shape[1]]
        radius = self.blacklist_radius / self.info.resolution
        for xy in self.blacklist:
            r, c = self.xy_to_cell(*xy)
            disc = (rows - r) ** 2 + (cols - c) ** 2 <= radius ** 2
            grid[disc & (grid == FREE)] = 100
        return grid

    def near_frontier(self, xy, zones):
        """Indique s'il reste une case frontière à moins de blacklist_radius du point xy."""
        r, c = self.xy_to_cell(*xy)
        radius = self.blacklist_radius / self.info.resolution
        return any((((z.cells[:, 0] - r) ** 2 + (z.cells[:, 1] - c) ** 2) <= radius ** 2).any()
                   for z in zones)

    # ------------------------------------------------------------------ Nav2

    def send_goal(self, goal_xy, robot_xy, size):
        yaw = math.atan2(goal_xy[1] - robot_xy[1], goal_xy[0] - robot_xy[0])
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = goal_xy
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal, self.goal_sent_at = goal_xy, self.get_clock().now()
        self.goals_sent += 1
        self.get_logger().info(
            f'But {self.goals_sent} : frontière de {size} cases en ({goal_xy[0]:.2f} ; '
            f'{goal_xy[1]:.2f}), à {math.dist(goal_xy, robot_xy):.2f} m')
        future = self.nav.send_goal_async(NavigateToPose.Goal(pose=pose))
        future.add_done_callback(lambda f, g=goal_xy: self.on_goal_response(f, g))

    def on_goal_response(self, future, goal_xy):
        handle = future.result()
        if goal_xy is not self.goal:          # but abandonné avant d'avoir été accepté
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            # Refus = Nav2 inactif (démarrage raté, par exemple), pas un mauvais but :
            # pas de liste noire, on réessaiera au prochain cycle.
            self.get_logger().warning('But refusé par Nav2 (Nav2 inactif ?) : nouvel essai',
                                      throttle_duration_sec=5.0)
            self.drop_goal(blacklist=False, cancel=False)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, h=handle: self.on_result(h, f.result().status))

    def on_result(self, handle, status):
        if handle is not self.goal_handle:   # ancien but, déjà abandonné
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            # atteint : si une frontière subsiste ici, le robot ne peut pas l'effacer
            self.get_logger().info('But atteint')
        elif status != GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(f'But échoué (statut {status})')
        self.drop_goal(blacklist=True, cancel=False)

    def drop_goal(self, blacklist, cancel=True):
        """Oublie le but en cours (en l'annulant auprès de Nav2), avec ou sans liste noire."""
        if cancel and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if blacklist and self.goal is not None:
            self.blacklist.append(self.goal)
        self.goal, self.goal_handle = None, None

    # ------------------------------------------------------------------ fin et affichage

    def finish(self, now):
        self.state = DONE
        free_area = np.count_nonzero(self.grid == FREE) * self.info.resolution ** 2
        duration = (now - self.started_at).nanoseconds * 1e-9
        self.get_logger().info(
            f'Exploration terminée : {self.goals_sent} buts, {duration:.0f} s, '
            f'surface libre cartographiée {free_area:.1f} m²')
        self.state_pub.publish(String(data=self.state))

    def publish_markers(self, zones):
        cells = Marker(type=Marker.POINTS, ns='frontieres', id=0, action=Marker.ADD)
        cells.header.frame_id = self.map_frame
        cells.scale.x = cells.scale.y = self.info.resolution
        cells.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        cells.pose.orientation.w = 1.0
        for z in zones:
            for c in z.cells:
                x, y = self.cell_to_xy(c)
                cells.points.append(Point(x=x, y=y, z=0.05))
        goal = Marker(type=Marker.SPHERE, ns='but', id=1)
        goal.header.frame_id = self.map_frame
        goal.pose.orientation.w = 1.0
        goal.scale.x = goal.scale.y = goal.scale.z = 0.25
        goal.color = ColorRGBA(r=0.0, g=0.8, b=0.2, a=1.0)
        if self.goal is not None:
            goal.action = Marker.ADD
            goal.pose.position.x, goal.pose.position.y = self.goal
        else:
            goal.action = Marker.DELETE
        self.marker_pub.publish(MarkerArray(markers=[cells, goal]))


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Le signal d'arrêt du launch peut fermer le contexte ROS pendant l'attente des messages
        # (RCLError « context is not valid ») : sans gravité. Toute autre erreur est remontée.
        if rclpy.ok():
            raise
    finally:
        if node.goal_handle is not None and rclpy.ok():
            node.goal_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
