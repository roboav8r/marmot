import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from tracetools_launch.action import Trace

def generate_launch_description():
    ld = LaunchDescription()

    # Get configuration from yaml file
    exp_config = os.path.join(
        get_package_share_directory('marmot'),
        'config',
        'mocap_robot_noheadset_exp.yaml'
        )
    def_config = os.path.join(
        get_package_share_directory('marmot'),
        'config',
        'mocap_robot_noheadset_exp.yaml'
    )

    # Manager node
    comp_node = Node(
        package='marmot',
        executable='mocap_robot_exp_mgr.py',
        name='mocap_robot_exp_mgr_node',
        output="screen",
        parameters=[exp_config]
    )
    ld.add_action(comp_node)

    # Detection preprocessing nodes
    oakd_preproc_node = Node(
        package='marmot',
        executable='depthai_preproc',
        name='depthai_preproc_node',
        remappings=[('/depthai_detections','/oak/nn/spatial_detections'), ('/converted_detections','/converted_detections_oakd')],
        parameters=[exp_config]
    )
    ld.add_action(oakd_preproc_node)

    # Tracker node
    trk_node = Node(
        package='marmot',
        executable='tbd_node.py',
        name='tbd_tracker_node',
        output='screen',
        parameters=[def_config]
    )
    ld.add_action(trk_node)

    return ld