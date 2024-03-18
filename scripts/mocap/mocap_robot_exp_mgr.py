#!/usr/bin/env python3

import os
import time
import json
import subprocess
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.wait_for_message import wait_for_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from foxglove_msgs.msg import SceneUpdate
from tracking_msgs.msg import Tracks3D
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Empty

class MoCapRobotExpManager(Node):
    def __init__(self):
        super().__init__('mocap_robot_exp_mgr_node')

        # Declare and load ROS parameters from .yaml file
        self.declare_parameter('mcap_files', rclpy.Parameter.Type.STRING_ARRAY )
        self.declare_parameter('data_dir', rclpy.Parameter.Type.STRING )
        self.declare_parameter('results_dir', rclpy.Parameter.Type.STRING )
        self.declare_parameter('oakd_det_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('headset_1_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('headset_2_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('pc_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('odom_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('track_topic', rclpy.Parameter.Type.STRING)
        
        self.mcap_files = self.get_parameter('mcap_files').get_parameter_value().string_array_value
        self.data_dir = Path.home() / self.get_parameter('data_dir').get_parameter_value().string_value
        self.results_dir = Path.home() / self.get_parameter('results_dir').get_parameter_value().string_value
        self.oakd_det_topic = self.get_parameter('oakd_det_topic').get_parameter_value().string_value
        self.headset_1_topic = self.get_parameter('headset_1_topic').get_parameter_value().string_value
        self.headset_2_topic = self.get_parameter('headset_2_topic').get_parameter_value().string_value
        self.track_topic = self.get_parameter('track_topic').get_parameter_value().string_value
        self.pc_topic = self.get_parameter('pc_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        # Other parameters
        self.package_dir = get_package_share_directory('marmot')

        # Create ROS objects
        self.reset_tracker_client = self.create_client(Empty, "reset_tracker")
        self.reconf_tracker_client = self.create_client(Empty, "reconfigure_tracker")
        self.empty_req = Empty.Request()
        self.reader = rosbag2_py.SequentialReader()
        self.tf_pub = self.create_publisher(TFMessage,'/tf',10)
        self.tf_static_pub = self.create_publisher(TFMessage,'/tf_static',rclpy.qos.qos_profile_action_status_default)
        self.oakd_publisher = self.create_publisher(Detection3DArray,self.oakd_det_topic,10)
        self.headset_1_publisher = self.create_publisher(PoseStamped,self.headset_1_topic,10)
        self.headset_2_publisher = self.create_publisher(PoseStamped,self.headset_2_topic,10)
        self.pc_publisher = self.create_publisher(PointCloud2,self.pc_topic,10)
        self.headset_1_interval = 10 # only send one of every 10 messages to simulate 10Hz sensor
        self.headset_2_interval = 10 # only send one of every 10 messages to simulate 10Hz sensor
        self.gt_1_rcvd = False
        self.gt_1_rcvd = False
        self.odom_rcvd = False

        # If results directory doesn't exist, create it
        if not os.path.exists(self.results_dir):
            os.mkdir(self.results_dir)
                # If results directory doesn't exist, create it
        if not os.path.exists(os.path.join(self.results_dir)):
            os.mkdir(os.path.join(self.results_dir))

    def tracker_callback(self, msg):
        self.results_str=''

        # Compute timing information from metadata
        for kv in msg.metadata:
            if kv.key == 'num_dets_rcvd':
                n_dets = int(kv.value)
            elif kv.key == 'num_tracks_published':
                n_trks = int(kv.value)
            elif kv.key =='time_det_rcvd':
                t_start = int(kv.value)
            elif kv.key == 'time_tracks_published':
                t_end = int(kv.value)

        # Compute actual-estimated track matches. Use euclidean norm and hungarian algo to assign.
        match_thresh = 1.0
        cost_matrix = np.zeros((2,len(msg.tracks)))

        for idx, est_trk in enumerate(msg.tracks):
            cost_matrix[0,idx] = np.linalg.norm([self.gt1_msg.pose.position.x - est_trk.pose.pose.position.x,self.gt1_msg.pose.position.y - est_trk.pose.pose.position.y,self.gt1_msg.pose.position.z - est_trk.pose.pose.position.z])
            cost_matrix[1,idx] = np.linalg.norm([self.gt2_msg.pose.position.x - est_trk.pose.pose.position.x,self.gt2_msg.pose.position.y - est_trk.pose.pose.position.y,self.gt2_msg.pose.position.z - est_trk.pose.pose.position.z])

        gt_idx, est_idx = linear_sum_assignment(cost_matrix)
        
        # Now, write output string based on assignment
        match_1_str = ',,,,,'
        match_2_str = ',,,,,'

        self.n_true_matches=0
        for ii,idx in enumerate(est_idx):
            if cost_matrix[gt_idx[ii],idx]>match_thresh:
                continue

            if gt_idx[ii] == 0:
                self.n_true_matches +=1
                match_1_str = "%s,%s,%s,%s,%s," % (msg.tracks[idx].pose.pose.position.x,msg.tracks[idx].pose.pose.position.y,msg.tracks[idx].pose.pose.position.z,msg.tracks[idx].track_id,msg.tracks[idx].track_confidence)
            elif gt_idx[ii] == 1:
                self.n_true_matches +=1
                match_2_str = "%s,%s,%s,%s,%s," % (msg.tracks[idx].pose.pose.position.x,msg.tracks[idx].pose.pose.position.y,msg.tracks[idx].pose.pose.position.z,msg.tracks[idx].track_id,msg.tracks[idx].track_confidence)

        self.results_str += "%s,%s,%s,%s," % (self.config, self.odom_msg.pose.position.x, self.odom_msg.pose.position.y, self.odom_msg.pose.position.z)
        self.results_str += "%s,%s,%s," % (self.gt1_msg.pose.position.x, self.gt1_msg.pose.position.y, self.gt1_msg.pose.position.z)
        self.results_str += match_1_str
        self.results_str += "%s,%s,%s," % (self.gt2_msg.pose.position.x, self.gt2_msg.pose.position.y, self.gt2_msg.pose.position.z)
        self.results_str += match_2_str
        self.results_str += str(len(msg.tracks)-self.n_true_matches)
        self.results_str += ",%s,%s,%s\n" % (n_dets, n_trks, (t_end-t_start))

        with open(os.path.join(self.results_dir, self.config + "_results.csv"), "a") as outfile:
            outfile.write(self.results_str)
        outfile.close()


    def run_experiments(self):
        self.get_logger().info("Running experiments")

        # Play mcap files
        for mcap_file in self.mcap_files:
            self.config = os.path.split(mcap_file)[0]
            self.get_logger().info("Loading %s/%s" % (self.data_dir, mcap_file))

            with open(os.path.join(self.results_dir, self.config + "_results.csv"), "w") as outfile:
                outfile.write("experiment,robot_x,robot_y,robot_z,gt1_x,gt1_y,gt1_z,track1_x,track1_y,track1_z,track1_id,track1_conf,gt2_x,gt2_y,gt2_z,track2_x,track2_y,track2_z,track2_id,track2_conf,unmatched_trks,n_dets,n_trks,delta_t\n")
            outfile.close()

            # Load .mcap file for this scene
            storage_options = rosbag2_py.StorageOptions(
                uri= "%s/%s" % (self.data_dir, mcap_file),
                storage_id='mcap')
            converter_options = rosbag2_py.ConverterOptions('', '')
            self.reader.open(storage_options, converter_options)

            # Deal with type names
            topic_types = self.reader.get_all_topics_and_types()

            def typename(topic_name):
                for topic_type in topic_types:
                    if topic_type.name == topic_name:
                        return topic_type.type
                raise ValueError(f"topic {topic_name} not in bag")

            # Reset tracker
            self.future = self.reset_tracker_client.call_async(self.empty_req)
            rclpy.spin_until_future_complete(self, self.future, timeout_sec=1.)

            self.headset_1_count = 0 
            self.headset_2_count = 0
            self.gt_1_rcvd = False
            self.gt_2_rcvd = False
            self.odom_rcvd = False

            # Publish all the TF messages first for the buffer
            while self.reader.has_next():
                topic, data, _ = self.reader.read_next()
                if topic=='/tf_static':
                    msg_type = get_message(typename(topic))
                    msg = deserialize_message(data, msg_type)
                    self.tf_static_pub.publish(msg)
                            
            # Process detection messages and format tracking results
            self.reader.open(storage_options, converter_options)
            while self.reader.has_next():
                topic, data, _ = self.reader.read_next()

                if topic=='/tf':
                    new_tf_msg = TFMessage()
                    msg_type = get_message(typename(topic))
                    msg = deserialize_message(data, msg_type)

                    for tf in msg.transforms:
                        # Fix disconnected trees created during .mcap recording
                        # do not publish robot odom/map transforms, use mocap for odom instead
                        if tf.header.frame_id in ['philbart/odom', 'philbart/map']:
                            continue
                        else:
                            new_tf_msg.transforms.append(tf)
                          
                    self.tf_pub.publish(new_tf_msg)

                elif topic=='/tf_static':
                    msg_type = get_message(typename(topic))
                    msg = deserialize_message(data, msg_type)
                    self.tf_static_pub.publish(msg)

                elif topic in self.oakd_det_topic:
                    msg_type = get_message(typename(topic))
                    msg = deserialize_message(data, msg_type)

                    if self.gt_1_rcvd and self.gt_2_rcvd:
                        # Send the detection 
                        self.oakd_publisher.publish(msg)

                        # wait for the track response from the tracker
                        ret, trk_msg = wait_for_message(Tracks3D, self, self.track_topic,1)
                        if ret:
                            self.tracker_callback(trk_msg)

                elif topic in self.headset_1_topic:
                    msg_type = get_message(typename(topic))
                    msg = deserialize_message(data, msg_type)

                    if self.headset_1_count%self.headset_1_interval==0 and self.gt_1_rcvd and self.gt_2_rcvd:
                        # Send the detection if not empty 
                        self.headset_1_publisher.publish(msg)

                        # wait for the track response from the tracker
                        ret, trk_msg = wait_for_message(Tracks3D, self, self.track_topic,1)
                        if ret:
                            self.tracker_callback(trk_msg)

                    self.headset_1_count +=1

                elif topic in self.headset_2_topic:

                    msg_type = get_message(typename(topic))
                    msg = deserialize_message(data, msg_type)

                    if self.headset_2_count%self.headset_2_interval==0 and self.gt_1_rcvd and self.gt_2_rcvd:
                        # Send the detection if not empty 
                        self.headset_2_publisher.publish(msg)

                        # wait for the track response from the tracker
                        ret, trk_msg = wait_for_message(Tracks3D, self, self.track_topic,1)
                        if ret:
                            self.tracker_callback(trk_msg)

                    self.headset_2_count +=1

                elif topic in self.pc_topic:

                    msg_type = get_message(typename(topic))
                    msg = deserialize_message(data, msg_type)

                    # Send the detection if not empty 
                    # if self.odom_rcvd:
                    self.pc_publisher.publish(msg)

                    # wait for the track response from the tracker
                    ret, trk_msg = wait_for_message(Tracks3D, self, self.track_topic,1)
                    if ret:
                        self.tracker_callback(trk_msg)

                elif topic=='/vrpn_client_node/groundtruth_1/pose':
                    msg_type = get_message(typename(topic))
                    self.gt1_msg = deserialize_message(data, msg_type)
                    self.gt_1_rcvd = True

                elif topic=='/vrpn_client_node/groundtruth_2/pose':
                    msg_type = get_message(typename(topic))
                    self.gt2_msg = deserialize_message(data, msg_type)
                    self.gt_2_rcvd = True

                elif topic=='/vrpn_client_node/robot_vicon/pose':
                    msg_type = get_message(typename(topic))
                    self.odom_msg = deserialize_message(data, msg_type)
                    self.odom_rcvd = True

def main(args=None):
    rclpy.init(args=args)

    # Create experiment manager
    mocap_robot_exp_mgr = MoCapRobotExpManager()
    mocap_robot_exp_mgr.run_experiments()

    # Shut down the node
    mocap_robot_exp_mgr.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()