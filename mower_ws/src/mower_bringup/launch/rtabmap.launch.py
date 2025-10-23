#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    frame_id = LaunchConfiguration('frame_id', default='camera')
    map_file = LaunchConfiguration('map', default=os.path.join(os.path.expanduser('~'), 'mower', 'mower_ws', 'maps', 'map.yaml'))
    # Nuovo percorso del database: ~/maps/rtabmap.db
    # La funzione os.path.expanduser('~') espande il percorso a /home/user
    db_path = os.path.join(os.path.expanduser('~'), 'mower', 'mower_ws', 'maps', 'rtabmap.db')

    # Argomento di lancio per il database_path
    database_path = DeclareLaunchArgument(
        'database_path',
        default_value=db_path,
        description='Path to the RTAB-Map database file.'
    )

    # RTAB-Map SLAM node for mapping and localization with monocular camera
    rtabmap_slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            # Parametri ROS standard (tipi nativi)
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'camera_frame_id': 'camera_link',
            'subscribe_depth': False,
            'subscribe_rgbd': False,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            'subscribe_odom': True,
            'subscribe_imu': True,
            'queue_size': 10,
            # 🚀 Modifica 1: Imposta il percorso del database a ~/maps/rtabmap.db
            'database_path': db_path,
            # 🚀 Modifica 2: Forza l'inizializzazione del database (e la cancellazione del precedente)
            'Rtabmap/InitWMWithAllNodes': 'true',
            # Parametri RTAB-Map (con '/') come stringhe
            'Rtabmap/DetectionRate': '1.0',
            'Rtabmap/CreateIntermediateNodes': 'true',
            'RGBD/NeighborLinkRefining': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'Mem/STMSize': '30',
            'Mem/NotLinkedNodesKept': 'false',
            'RGBD/Enabled': 'false',
            'RGBD/SubscribeDepth': 'false',
            'RGB/Enabled': 'true',
            'Mem/IncrementalMemory': 'true',
            'RGBD/Localization': 'false',
            'Grid/FromDepth': 'false',
            'Grid/Sensor': '1',
            'Grid/RangeMax': '15.0',
            'Grid/Incremental': 'true',
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('imu', '/imu/data'),
            ('odom', '/odometry/filtered'),
        ]
    )

    return LaunchDescription([
        database_path,
        DeclareLaunchArgument('frame_id', default_value='camera', description='TF frame id for the images'),
        DeclareLaunchArgument('map', default_value=os.path.join(os.path.expanduser('~'), 'mower', 'mower_ws', 'maps', 'map.yaml'), description='Path to the map file in maps directory'),
        rtabmap_slam_node,
    ])
