"""Un essai d'approche de cible (E3) : simulation + approche + mesure, arrêt automatique.

    ros2 launch robot_mobile_bringup target_trial.launch.py scenario:=face_proche \
        x:=3.0 y:=-0.3 yaw:=3.1416 output:=~/as52_data/target_trials/face_proche

Le nœud trial_monitor arrête l'essai quand la cible est atteinte (ou au bout du délai) et écrit
result.json dans `output`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    bringup = get_package_share_directory('robot_mobile_bringup')
    output = os.path.expanduser(LaunchConfiguration('output').perform(context))

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'simulation.launch.py')),
        launch_arguments={name: LaunchConfiguration(name)
                          for name in ('gui', 'rviz', 'x', 'y', 'yaw')}.items())
    approach = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(bringup, 'launch', 'target_approach.launch.py')))
    monitor = Node(
        package='robot_mobile_evaluation', executable='trial_monitor', output='screen',
        parameters=[{'output_dir': output, 'use_sim_time': True,
                     'scenario': LaunchConfiguration('scenario'),
                     'timeout': LaunchConfiguration('timeout')}])
    stop = RegisterEventHandler(OnProcessExit(
        target_action=monitor,
        on_exit=[EmitEvent(event=Shutdown(reason="fin de l'essai"))]))
    # le comportement démarre une fois le robot apparu et les capteurs actifs
    return [simulation, TimerAction(period=8.0, actions=[approach, monitor]), stop]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output', description="Dossier des résultats de l'essai"),
        DeclareLaunchArgument('scenario', default_value='essai'),
        DeclareLaunchArgument('timeout', default_value='120.0',
                              description='Durée maximale (s, temps simulé)'),
        DeclareLaunchArgument('x', default_value='-2.5'),
        DeclareLaunchArgument('y', default_value='-1.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
