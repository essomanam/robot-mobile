"""Enregistre le jeu de données de référence : simulation + trajectoire scriptée + rosbag.

Une seule commande produit un enregistrement réutilisable pour les étapes suivantes
(SLAM, perception) sans relancer la simulation :

    ros2 launch robot_mobile_bringup reference_dataset.launch.py
    ros2 launch robot_mobile_bringup reference_dataset.launch.py output:=/chemin/E1_reference

Tout s'arrête automatiquement à la fin de la trajectoire.
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Topics enregistrés : tout ce qu'il faut pour rejouer la perception et le SLAM,
# plus la vérité terrain pour évaluer.
TOPICS = [
    '/clock', '/tf', '/tf_static', '/joint_states', '/robot_description',
    '/odom', '/imu', '/scan',
    '/camera/color/image_raw', '/camera/color/camera_info',
    '/camera/depth/image_raw', '/camera/depth/camera_info',
    '/ground_truth/odom', '/cmd_vel',
]


def generate_launch_description():
    bringup = get_package_share_directory('robot_mobile_bringup')
    default_output = os.path.join(
        os.path.expanduser('~'), 'as52_data', 'rosbags',
        'E1_reference_' + time.strftime('%Y%m%d_%H%M%S'))

    output = LaunchConfiguration('output')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'simulation.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui'),
                          'rviz': LaunchConfiguration('rviz')}.items())

    # Compression zstd par message : réduit fortement la taille de la profondeur simulée
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--use-sim-time', '-s', 'mcap',
             '--storage-preset-profile', 'zstd_fast', '-o', output, '--topics'] + TOPICS,
        output='screen')

    trajectory = Node(
        package='robot_mobile_evaluation',
        executable='reference_trajectory',
        parameters=[os.path.join(bringup, 'config', 'reference_trajectory.yaml')],
        output='screen')

    stop_at_end = RegisterEventHandler(OnProcessExit(
        target_action=trajectory,
        on_exit=[EmitEvent(event=Shutdown(reason='trajectoire de référence terminée'))]))

    return LaunchDescription([
        DeclareLaunchArgument('output', default_value=default_output,
                              description='Dossier de sortie du rosbag (ne doit pas exister)'),
        DeclareLaunchArgument('gui', default_value='false', description='Afficher Gazebo'),
        DeclareLaunchArgument('rviz', default_value='false', description='Afficher RViz'),
        simulation,
        # laisser le temps au robot d'apparaître et aux capteurs de démarrer
        TimerAction(period=8.0, actions=[recorder]),
        TimerAction(period=10.0, actions=[trajectory]),
        stop_at_end,
    ])
