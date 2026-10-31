"""Navigation autonome (E4) : localisation RTAB-Map + obstacles vus par la profondeur + Nav2.

    ros2 launch robot_mobile_bringup simulation.launch.py \
        world:=<bringup>/worlds/as52_room_obstacles.sdf
    ros2 launch robot_mobile_bringup navigation.launch.py map_database:=<carte>.db rviz:=true

Puis, dans RViz : « 2D Goal Pose » pour envoyer un but. Arguments utiles :
  use_depth:=false     LiDAR seul (sans le nuage de points de la caméra de profondeur)
  initial_pose:="x y z roulis tangage lacet"   pose de départ dans la carte (sinon RTAB-Map
                       se relocalise seul, grâce à la carte faite dans les deux sens, D-015)
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Nœuds lancés par nav2_bringup/navigation_launch.py. Sous Lyrical, ce fichier ne lance pas de
# gestionnaire de cycle de vie (c'est bringup_launch.py qui le fait) : sans lui, les nœuds restent
# « non configurés » (constaté au premier essai E4). On lance donc le nôtre.
NAVIGATION_NODES = ['controller_server', 'smoother_server', 'planner_server', 'route_server',
                    'behavior_server', 'velocity_smoother', 'collision_monitor', 'bt_navigator',
                    'waypoint_follower', 'docking_server', 'following_server']


def nav2_params_file(bringup, use_depth):
    """Fichier de paramètres Nav2 ; sans profondeur, variante où la source depth_cloud est retirée.

    Il ne suffit pas d'arrêter le nuage de points : le collision monitor attendrait cette source
    et arrêterait le robot (« invalid source »), constaté au premier essai « LiDAR seul ».
    """
    path = os.path.join(bringup, 'config', 'navigation', 'nav2_params.yaml')
    if use_depth:
        return path
    import yaml
    with open(path) as f:
        params = yaml.safe_load(f)
    for costmap, layer in (('local_costmap', 'voxel_layer'), ('global_costmap', 'obstacle_layer')):
        source = params[costmap][costmap]['ros__parameters'][layer]
        source['observation_sources'] = 'scan'
        source.pop('depth_cloud', None)
    monitor = params['collision_monitor']['ros__parameters']
    monitor['observation_sources'] = ['scan']
    monitor.pop('depth_cloud', None)
    out = tempfile.NamedTemporaryFile('w', prefix='nav2_lidar_seul_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, out)
    out.close()
    return out.name


def launch_setup(context):
    bringup = get_package_share_directory('robot_mobile_bringup')
    nav2 = get_package_share_directory('nav2_bringup')
    use_depth = LaunchConfiguration('use_depth').perform(context).lower() == 'true'

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'slam.launch.py')),
        launch_arguments={'method': 'rtabmap_rgbd_scan', 'localization': 'true',
                          'database_path': LaunchConfiguration('map_database'),
                          'initial_pose': LaunchConfiguration('initial_pose')}.items())

    # Image de profondeur -> nuage de points (source d'obstacles pour Nav2)
    depth_cloud = Node(
        package='rtabmap_util', executable='point_cloud_xyz', name='depth_to_cloud',
        output='screen', condition=IfCondition(LaunchConfiguration('use_depth')),
        parameters=[{'use_sim_time': True,
                     'decimation': 4,      # 1 pixel sur 4 : 160 x 120 points
                     'voxel_size': 0.05,   # m : un point par cube de 5 cm
                     'max_depth': 3.0}],   # m : au-delà, profondeur jugée trop bruitée
        remappings=[('depth/image', '/camera/depth/image_raw'),
                    ('depth/camera_info', '/camera/depth/camera_info'),
                    ('cloud', '/camera/points')])

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params_file(bringup, use_depth),
            'use_sim_time': 'true', 'autostart': 'true'}.items())

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': NAVIGATION_NODES}])

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(nav2, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')))

    return [localization, depth_cloud, navigation, lifecycle_manager, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_database',
                              description='Carte RTAB-Map (.db), modifiée sur place'),
        DeclareLaunchArgument('use_depth', default_value='true',
                              description='Obstacles vus par la caméra de profondeur'),
        DeclareLaunchArgument('initial_pose', default_value='',
                              description='Pose de départ dans la carte (vide = relocalisation)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
