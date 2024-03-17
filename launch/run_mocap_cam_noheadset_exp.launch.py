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
        'mocap_cam_noheadset_exp.yaml'
        )
    def_config = os.path.join(
        get_package_share_directory('marmot'),
        'config',
        'mocap_cam_noheadset_exp.yaml'
    )

    # Manager node
    comp_node = Node(
        package='marmot',
        executable='mocap_cam_exp_mgr.py',
        name='mocap_cam_exp_mgr_node',
        output="screen",
        parameters=[exp_config]
    )
    ld.add_action(comp_node)

    # Detection preprocessing nodes
    det_l_node = Node(
        package='marmot',
        executable='depthai_preproc',
        name='depthai_l_preproc_node',
        remappings=[('/depthai_detections','/oak_d_left/nn/spatial_detections'), ('/converted_detections','/converted_detections_left')],
        parameters=[exp_config]
    )
    ld.add_action(det_l_node)
    det_r_node = Node(
        package='marmot',
        executable='depthai_preproc',
        name='depthai_r_preproc_node',
        remappings=[('/depthai_detections','/oak_d_right/nn/spatial_detections'), ('/converted_detections','/converted_detections_right')],
        parameters=[exp_config]
    )
    ld.add_action(det_r_node)

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