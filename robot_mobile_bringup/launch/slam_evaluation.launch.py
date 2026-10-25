"""Évalue un SLAM sur un rosbag : rejeu + SLAM + enregistrement des trajectoires.

Une commande par essai ; tout s'arrête à la fin du rosbag. Les résultats (trajectoires
TUM, carte, statistiques, base RTAB-Map) sont écrits dans `output`, puis comparés avec :
    ros2 run robot_mobile_evaluation evaluate_slam <dossier des essais> <dossier des figures>

Exemple :
    ros2 launch robot_mobile_bringup slam_evaluation.launch.py method:=rtabmap_rgbd \
        bag:=~/as52_data/rosbags/E1_reference_20261024 output:=~/as52_data/slam_runs/rtabmap_rgbd_1

Localisation sur une carte existante (RTAB-Map) : la carte est copiée dans `output`
(l'originale n'est jamais modifiée) puis RTAB-Map est lancé en mode localisation :
    ros2 launch robot_mobile_bringup slam_evaluation.launch.py localization:=true \
        map_database:=<essai de cartographie>/rtabmap.db bag:=<autre parcours> output:=...

Attention : aucune simulation ne doit tourner sur le même domaine ROS (elle publierait
aussi /clock, /tf, /odom...). Sinon, isoler l'essai : ROS_DOMAIN_ID=42 ros2 launch ...
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    bringup = get_package_share_directory('robot_mobile_bringup')
    method = LaunchConfiguration('method').perform(context)
    bag = os.path.expanduser(LaunchConfiguration('bag').perform(context))
    output = os.path.expanduser(LaunchConfiguration('output').perform(context))
    rate = LaunchConfiguration('rate').perform(context)
    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    os.makedirs(output, exist_ok=True)  # RTAB-Map y crée sa base de données dès le démarrage
    database = os.path.join(output, 'rtabmap.db')
    if localization:
        if method.startswith('slam_toolbox'):
            raise RuntimeError('localisation évaluée seulement pour RTAB-Map')
        source = os.path.expanduser(LaunchConfiguration('map_database').perform(context))
        if not os.path.isfile(source):
            raise RuntimeError(f'carte introuvable : {source}')
        shutil.copyfile(source, database)

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'slam.launch.py')),
        launch_arguments={'method': method,
                          'database_path': database,
                          'localization': str(localization).lower(),
                          'initial_pose': LaunchConfiguration('initial_pose'),
                          'extra_params': LaunchConfiguration('extra_params')}.items())

    logger = Node(
        package='robot_mobile_evaluation', executable='trajectory_logger', output='screen',
        parameters=[{'output_dir': output, 'label': method, 'use_sim_time': True}])

    # Le rosbag contient l'horloge simulée (/clock) : elle est rejouée telle quelle.
    play = ['ros2', 'bag', 'play', bag, '--rate', rate, '--disable-keyboard-controls']
    if method == 'rtabmap_rgbd_vo':
        # sans les roues : on retire odom -> base_footprint enregistré, l'odométrie
        # visuelle le recalcule à partir des images
        play += ['--exclude-topics', '/tf']
    player = ExecuteProcess(cmd=play, output='screen')

    stop_after_bag = RegisterEventHandler(OnProcessExit(
        target_action=player,
        # quelques secondes pour que le SLAM traite les dernières données
        on_exit=[TimerAction(period=3.0, actions=[
            EmitEvent(event=Shutdown(reason='fin du rosbag'))])]))

    return [slam, logger, TimerAction(period=5.0, actions=[player]), stop_after_bag]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='rtabmap_rgbd_scan',
                              description='Méthode de SLAM (voir slam.launch.py)'),
        DeclareLaunchArgument(
            'bag', default_value='~/as52_data/rosbags/E1_reference_20261024',
            description='Rosbag à rejouer'),
        DeclareLaunchArgument('output', description='Dossier des résultats de cet essai'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='Vitesse de rejeu (1.0 = temps réel)'),
        DeclareLaunchArgument('extra_params', default_value='',
                              description='RTAB-Map : fichier YAML de réglage chargé en dernier'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='RTAB-Map : localisation sur la carte map_database'),
        DeclareLaunchArgument('map_database', default_value='',
                              description='Carte RTAB-Map (.db) à utiliser en localisation'),
        DeclareLaunchArgument('initial_pose', default_value='',
                              description='Localisation : pose de départ connue dans la carte'),
        OpaqueFunction(function=launch_setup),
    ])
