"""Simulation du projet : Gazebo + salle as52_room + TurtleBot3 Waffle RGB-D + pont ROS 2 + RViz.

Usage :
    ros2 launch robot_mobile_bringup simulation.launch.py
    ros2 launch robot_mobile_bringup simulation.launch.py gui:=false rviz:=false   # sans fenêtre
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('robot_mobile_bringup')
    description = get_package_share_directory('robot_mobile_description')
    tb3_models = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    arguments = [
        DeclareLaunchArgument(
            'world', default_value=os.path.join(bringup, 'worlds', 'as52_room.sdf'),
            description='Fichier monde Gazebo'),
        DeclareLaunchArgument('x', default_value='-2.5', description='Position initiale x (m)'),
        DeclareLaunchArgument('y', default_value='-1.5', description='Position initiale y (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Cap initial (rad)'),
        DeclareLaunchArgument('gui', default_value='true', description='Afficher Gazebo'),
        DeclareLaunchArgument('rviz', default_value='true', description='Afficher RViz'),
    ]

    # Où Gazebo cherche les modèles « model://… » : nos modèles, puis ceux de TurtleBot3
    # (maillages turtlebot3_common, mobilier de turtlebot3_house).
    resource_paths = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', path)
        for path in (
            os.path.join(bringup, 'models'),
            os.path.join(description, 'models'),
            tb3_models,
            os.path.join(tb3_models, 'turtlebot3_house'),
        )
    ]

    # Serveur Gazebo (physique + capteurs). Sans interface, le rendu des caméras
    # se fait hors écran (--headless-rendering).
    gz_server_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(gui))
    gz_server_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': ['-r -s -v2 --headless-rendering ', world],
            'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(gui))
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(gui))

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(description, 'urdf', 'robot_mobile.urdf.xacro')]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot_mobile',
            '-file', os.path.join(description, 'models', 'robot_mobile', 'model.sdf'),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.01',
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(bringup, 'config', 'gz_bridge.yaml'),
            'use_sim_time': True,
        }],
        output='screen')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(bringup, 'rviz', 'simulation.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen')

    return LaunchDescription(arguments + resource_paths + [
        gz_server_gui,
        gz_server_headless,
        gz_client,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz_node,
    ])
