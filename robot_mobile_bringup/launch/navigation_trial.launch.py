"""Un essai de navigation (E4) : simulation avec obstacles + localisation + Nav2 + mesure.

    ros2 launch robot_mobile_bringup navigation_trial.launch.py scenario:=carton \
        map_database:=<carte>.db x:=-2.5 y:=-1.5 yaw:=0.0 goal:="[2.5, -1.5, 0.0]" \
        use_depth:=true output:=~/as52_data/nav_trials/carton_profondeur

La carte est copiée dans `output` (l'originale n'est pas modifiée) ; la pose de départ est
donnée à RTAB-Map, convertie dans le repère de la carte grâce à `map_origin`.
"""

import math
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Obstacles ajoutés dans worlds/as52_room_obstacles.sdf : x, y, demi-largeur x, demi-largeur y
EXTRA_OBSTACLES = [0.0, -1.5, 0.2, 0.2,     # carton
                   0.0, 1.5, 0.3, 0.3]      # obstacle bas


def launch_setup(context):
    bringup = get_package_share_directory('robot_mobile_bringup')
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    output = os.path.expanduser(value('output'))
    os.makedirs(output, exist_ok=True)
    database = os.path.join(output, 'rtabmap.db')
    shutil.copyfile(os.path.expanduser(value('map_database')), database)

    # pose de départ (monde) -> repère de la carte
    x, y, yaw = (float(value(n)) for n in ('x', 'y', 'yaw'))
    ox, oy, oyaw = (float(v) for v in value('map_origin').split())
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    initial = (f'{c * (x - ox) - s * (y - oy):.4f} {s * (x - ox) + c * (y - oy):.4f} 0 0 0 '
               f'{math.remainder(yaw - oyaw, 2 * math.pi):.4f}')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'simulation.launch.py')),
        launch_arguments={'world': os.path.join(bringup, 'worlds', 'as52_room_obstacles.sdf'),
                          'gui': value('gui'), 'rviz': 'false',
                          'x': str(x), 'y': str(y), 'yaw': str(yaw)}.items())
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'navigation.launch.py')),
        launch_arguments={'map_database': database, 'use_depth': value('use_depth'),
                          'initial_pose': initial, 'rviz': value('rviz')}.items())
    trial = Node(
        package='robot_mobile_evaluation', executable='navigation_trial', output='screen',
        parameters=[{'output_dir': output, 'scenario': value('scenario'), 'use_sim_time': True,
                     'goal': [float(v) for v in value('goal').strip('[]').split(',')],
                     'map_origin': [ox, oy, oyaw], 'timeout': float(value('timeout')),
                     'extra_obstacles': EXTRA_OBSTACLES}])
    stop = RegisterEventHandler(OnProcessExit(
        target_action=trial, on_exit=[EmitEvent(event=Shutdown(reason="fin de l'essai"))]))
    return [simulation, TimerAction(period=8.0, actions=[navigation]),
            TimerAction(period=20.0, actions=[trial]), stop]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output', description="Dossier des résultats de l'essai"),
        DeclareLaunchArgument('map_database', description='Carte RTAB-Map (.db), copiée'),
        DeclareLaunchArgument('map_origin', default_value='-2.500876 -1.500003 0.0',
                              description='Pose (monde) du robot au début de la cartographie'),
        DeclareLaunchArgument('scenario', default_value='essai'),
        DeclareLaunchArgument('goal', default_value='[2.5, -1.5, 0.0]',
                              description='But (monde) : [x, y, cap]'),
        DeclareLaunchArgument('use_depth', default_value='true'),
        DeclareLaunchArgument('timeout', default_value='180.0'),
        DeclareLaunchArgument('x', default_value='-2.5'),
        DeclareLaunchArgument('y', default_value='-1.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
