"""Navigation guidée par la vision (E3) : détection de la cible + comportement d'approche.

À lancer avec la simulation (ou le robot réel) déjà démarrée :
    ros2 launch robot_mobile_bringup simulation.launch.py
    ros2 launch robot_mobile_bringup target_approach.launch.py

Pour voir ce que détecte la caméra : ros2 run rqt_image_view rqt_image_view /target/debug_image
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('robot_mobile_bringup'), 'config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='true en simulation, false sur le robot réel'),
        Node(package='robot_mobile_perception', executable='target_detector', output='screen',
             parameters=[os.path.join(config, 'perception', 'target_detector.yaml'),
                         {'use_sim_time': use_sim_time}]),
        Node(package='robot_mobile_control', executable='target_follower', output='screen',
             parameters=[os.path.join(config, 'control', 'target_follower.yaml'),
                         {'use_sim_time': use_sim_time}]),
    ])
