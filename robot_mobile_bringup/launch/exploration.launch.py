"""Exploration autonome d'un lieu inconnu.

Simulation + RTAB-Map (cartographie) + Nav2 + explorateur.

Le robot démarre sans carte ; il cartographie en naviguant, va de frontière en frontière (limites
entre le connu et l'inconnu) et s'arrête quand il n'en reste plus.

    ros2 launch robot_mobile_bringup exploration.launch.py world:=arena
    ros2 launch robot_mobile_bringup exploration.launch.py world:=house gui:=false

Mondes (argument world) :
    as52_room   la salle du projet (7 x 5 m), départ (-2,5 ; -1,5)
    arena       l'arène TurtleBot3 (turtlebot3_world, ~4 m), départ (-2,0 ; -0,5)
    house       la maison TurtleBot3 (15 x 10,5 m, plusieurs pièces, cour close),
                départ (-2,0 ; 1,0)
    ou le chemin d'un fichier .sdf (donner alors x, y, yaw)
La carte est écrite dans <output>/rtabmap.db, ÉCRASÉE à chaque lancement
(par défaut ~/as52_data/exploration/<monde>/).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, \
    TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# monde -> (fichier, x, y, cap de départ)
WORLDS = {
    'as52_room': ('as52_room.sdf', '-2.5', '-1.5', '0.0'),
    'arena': ('tb3_arena.sdf', '-2.0', '-0.5', '0.0'),
    'house': ('tb3_house.sdf', '-2.0', '1.0', '0.0'),
}


def launch_setup(context):
    bringup = get_package_share_directory('robot_mobile_bringup')
    world = LaunchConfiguration('world').perform(context)
    if world in WORLDS:
        world_file, *start = WORLDS[world]
        world_path = os.path.join(bringup, 'worlds', world_file)
        name = world
    else:
        world_path = os.path.expanduser(world)
        start = ['0.0', '0.0', '0.0']
        name = os.path.splitext(os.path.basename(world_path))[0]
    x, y, yaw = (LaunchConfiguration(arg).perform(context) or default
                 for arg, default in zip(('x', 'y', 'yaw'), start))
    output = os.path.expanduser(LaunchConfiguration('output').perform(context)
                                or os.path.join('~', 'as52_data', 'exploration', name))
    os.makedirs(output, exist_ok=True)

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'simulation.launch.py')),
        launch_arguments={'world': world_path, 'x': x, 'y': y, 'yaw': yaw,
                          'gui': LaunchConfiguration('gui'), 'rviz': 'false'}.items())

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'slam_mode': 'mapping',
            'map_database': os.path.join(output, 'rtabmap.db'),
            'use_depth': LaunchConfiguration('use_depth'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': os.path.join(bringup, 'rviz', 'exploration.rviz')}.items())

    explorer = Node(
        package='robot_mobile_control', executable='explorer', output='screen',
        parameters=[os.path.join(bringup, 'config', 'control', 'explorer.yaml')])

    # Gazebo d'abord, puis RTAB-Map et Nav2, puis l'explorateur (il attend aussi Nav2 lui-même)
    return [simulation,
            TimerAction(period=8.0, actions=[navigation]),
            TimerAction(period=15.0, actions=[explorer])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='arena',
                              description="as52_room, arena, house ou chemin d'un .sdf"),
        DeclareLaunchArgument('x', default_value='',
                              description='Départ x (m), sinon celui du monde'),
        DeclareLaunchArgument('y', default_value='', description='Départ y (m)'),
        DeclareLaunchArgument('yaw', default_value='', description='Cap de départ (rad)'),
        DeclareLaunchArgument('output', default_value='',
                              description='Dossier de la carte '
                                          '(défaut : ~/as52_data/exploration/<monde>)'),
        DeclareLaunchArgument('use_depth', default_value='true',
                              description='Obstacles vus par la caméra de profondeur'),
        DeclareLaunchArgument('gui', default_value='true', description='Afficher Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Afficher RViz'),
        OpaqueFunction(function=launch_setup),
    ])
