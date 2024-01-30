import os

from ament_index_python import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ld = LaunchDescription()

    # Config files
    cam_config = os.path.join(
        get_package_share_directory('ros_tracking'),
        'config',
        'oakd_cam.yaml'
    )
    tracker_config = os.path.join(
        get_package_share_directory('ros_tracking'),
        'config',
        'oakd_tracker.yaml'
    )

    # Static TF node
    tf_node = Node(package = "tf2_ros", 
                    executable = "static_transform_publisher",
                    arguments = ["0", "0", "1.0", "0", "0", "0", "map", "oak-d-base-frame"]
    )
    ld.add_action(tf_node)

    # Detection conversion node
    conv_node = Node(
        package='ros_tracking',
        executable='depthai_converter',
        name='depthai_converter_node',
        remappings=[('/depthai_detections','/oak/nn/spatial_detections')],
        output='screen',
        parameters=[cam_config])    
    ld.add_action(conv_node)

    # Sensor node
    cam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('depthai_ros_driver'),
                'launch',
                'camera.launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': cam_config
        }.items()
    )
    ld.add_action(cam_node)

    # Tracker node
    trk_node = Node(
        package='ros_tracking',
        executable='py_tracker.py',
        name='tracker',
        output='screen',
        remappings=[('/detections','/converted_detections')],
        parameters=[tracker_config]
    )
    ld.add_action(trk_node)

    # Foxglove bridge for visualization
    viz_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch/foxglove_bridge_launch.xml'))
    )
    ld.add_action(viz_node)

    return ld