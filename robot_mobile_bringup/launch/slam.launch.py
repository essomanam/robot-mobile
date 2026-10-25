"""Lance un SLAM du projet, sur la simulation en direct ou sur un rosbag rejoué.

Méthodes (argument method) :
    rtabmap_rgbd       B. RTAB-Map, caméra RGB-D + odométrie des roues (configuration principale)
    rtabmap_rgbd_vo    C. RTAB-Map, caméra RGB-D seule (odométrie visuelle, sans les roues)
    rtabmap_rgbd_scan  D. RTAB-Map, caméra RGB-D + LiDAR + odométrie des roues
    slam_toolbox       E. slam_toolbox, LiDAR + odométrie des roues

Exemples :
    ros2 launch robot_mobile_bringup slam.launch.py
    ros2 launch robot_mobile_bringup slam.launch.py method:=slam_toolbox
    ros2 launch robot_mobile_bringup slam.launch.py localization:=true database_path:=<carte.db>
    ros2 launch robot_mobile_bringup slam.launch.py extra_params:=<réglage.yaml>  # essai
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

METHODS = ('rtabmap_rgbd', 'rtabmap_rgbd_vo', 'rtabmap_rgbd_scan', 'slam_toolbox')

CAMERA_REMAPPINGS = [
    ('rgb/image', '/camera/color/image_raw'),
    ('rgb/camera_info', '/camera/color/camera_info'),
    ('depth/image', '/camera/depth/image_raw'),
]


def launch_setup(context):
    config = os.path.join(get_package_share_directory('robot_mobile_bringup'), 'config', 'slam')
    method = LaunchConfiguration('method').perform(context)
    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    database = os.path.expanduser(LaunchConfiguration('database_path').perform(context))
    extra = LaunchConfiguration('extra_params').perform(context)
    if method not in METHODS:
        raise RuntimeError(f'method inconnue : {method} (choix : {", ".join(METHODS)})')

    if method == 'slam_toolbox':
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch',
                'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': os.path.join(config, 'slam_toolbox.yaml'),
                'use_sim_time': 'true'}.items())]

    parameters = [os.path.join(config, 'rtabmap.yaml')]
    if method == 'rtabmap_rgbd_scan':
        parameters.append(os.path.join(config, 'rtabmap_scan.yaml'))
    if extra:
        parameters.append(os.path.expanduser(extra))  # réglage testé, chargé en dernier
    overrides = {'use_sim_time': True, 'database_path': database}
    if localization:
        # carte figée : on se localise dans la carte enregistrée sans la modifier
        overrides.update({'Mem/IncrementalMemory': 'false', 'Mem/InitWMWithAllNodes': 'true'})
    parameters.append(overrides)

    remappings = CAMERA_REMAPPINGS + [('scan', '/scan'), ('map', '/map')]
    actions = []
    if method == 'rtabmap_rgbd_vo':
        # l'odométrie visuelle publie sur /rtabmap/odom, que rtabmap lit par défaut
        actions.append(Node(
            package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry',
            namespace='rtabmap', output='screen',
            parameters=[os.path.join(config, 'rtabmap_vo.yaml'), {'use_sim_time': True}],
            remappings=CAMERA_REMAPPINGS))
    else:
        remappings.append(('odom', '/odom'))

    actions.append(Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', namespace='rtabmap',
        output='screen', parameters=parameters, remappings=remappings,
        # -d : efface la base de données précédente (nouvelle carte)
        arguments=[] if localization else ['-d']))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='rtabmap_rgbd',
                              description='Méthode : ' + ', '.join(METHODS)),
        DeclareLaunchArgument('localization', default_value='false',
                              description='RTAB-Map : localisation sur une carte existante'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db',
                              description='RTAB-Map : base de données (carte) lue ou écrite'),
        DeclareLaunchArgument('extra_params', default_value='',
                              description='RTAB-Map : fichier YAML de réglage chargé en dernier'),
        OpaqueFunction(function=launch_setup),
    ])
