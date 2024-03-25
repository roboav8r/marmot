import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Get configuration from yaml file
    exp_config = os.path.join(
        get_package_share_directory('marmot'),
        'config',
        'exp_bestval.yaml'
        )
    def_config = os.path.join(
        get_package_share_directory('marmot'),
        'config',
        'nusc_baseline.yaml'
    )

    # Manager node
    comp_node = Node(
        package='marmot',
        executable='nuscenes_exp_mgr.py',
        name='nusc_exp_mgr_node',
        output="screen",
        parameters=[exp_config]
    )
    ld.add_action(comp_node)

    # Detection conversion node
    det_node = Node(
        package='marmot',
        executable='nuscenes_preproc',
        name='nuscenes_preproc',
        remappings=[('/nuscenes_detections','/detections' )]
    )
    ld.add_action(det_node)

    # Tracker node
    trk_node = Node(
        package='marmot',
        executable='tbd_node.py',
        name='tbd_tracker_node',
        output='screen',
        remappings=[('/detections','/converted_detections')],
        parameters=[def_config]
    )
    ld.add_action(trk_node)

    return ld