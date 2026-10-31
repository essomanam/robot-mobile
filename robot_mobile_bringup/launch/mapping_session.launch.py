"""Cartographie en direct : simulation + SLAM + trajectoire scriptée + enregistrement léger.

Contrairement à reference_dataset.launch.py, les images ne sont pas enregistrées (plusieurs Go) :
le SLAM tourne pendant la simulation et produit directement la carte (base RTAB-Map), tandis que
trajectory_logger enregistre les trajectoires (vérité terrain, odométrie, SLAM en ligne).

    ros2 launch robot_mobile_bringup mapping_session.launch.py \
        output:=~/as52_data/maps/carte_2_sens \
        trajectory:=<bringup>/config/mapping_bidirectional_trajectory.yaml

Tout s'arrête automatiquement à la fin de la trajectoire ; la carte est dans <output>/rtabmap.db.
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
    method = LaunchConfiguration('method').perform(context)
    os.makedirs(output, exist_ok=True)

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'simulation.launch.py')),
        launch_arguments={name: LaunchConfiguration(name)
                          for name in ('gui', 'rviz', 'x', 'y', 'yaw')}.items())

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'slam.launch.py')),
        launch_arguments={'method': method,
                          'database_path': os.path.join(output, 'rtabmap.db')}.items())

    logger = Node(
        package='robot_mobile_evaluation', executable='trajectory_logger', output='screen',
        parameters=[{'output_dir': output, 'label': method, 'use_sim_time': True}])

    trajectory = Node(
        package='robot_mobile_evaluation', executable='reference_trajectory', output='screen',
        parameters=[LaunchConfiguration('trajectory')])

    stop_at_end = RegisterEventHandler(OnProcessExit(
        target_action=trajectory,
        # quelques secondes pour que le SLAM intègre les dernières images
        on_exit=[TimerAction(period=3.0, actions=[
            EmitEvent(event=Shutdown(reason='trajectoire de cartographie terminée'))])]))

    return [simulation, TimerAction(period=8.0, actions=[slam, logger]),
            TimerAction(period=12.0, actions=[trajectory]), stop_at_end]


def generate_launch_description():
    bringup = get_package_share_directory('robot_mobile_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('output', description='Dossier de la carte et des trajectoires'),
        DeclareLaunchArgument(
            'trajectory',
            default_value=os.path.join(bringup, 'config', 'mapping_bidirectional_trajectory.yaml'),
            description='Trajectoire scriptée'),
        DeclareLaunchArgument('method', default_value='rtabmap_rgbd_scan',
                              description='Méthode de SLAM (voir slam.launch.py)'),
        DeclareLaunchArgument('x', default_value='-2.5'),
        DeclareLaunchArgument('y', default_value='-1.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
