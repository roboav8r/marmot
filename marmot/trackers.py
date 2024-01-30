#!/usr/bin/env python3

import gtsam
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Empty
from foxglove_msgs.msg import SceneUpdate, SceneEntity
from tracking_msgs.msg import Tracks3D, Track3D, Detections3D, Detection3D

from ros_tracking.datatypes import GraphDet, GraphTrack
from ros_tracking.assignment import ComputeAssignment
from ros_tracking.track_management import CreateTracks, DeleteTracks
from ros_tracking.output import PublishTracks, PublishScene
from ros_tracking.sensors import CreateSensorModels, ReconfigureSensorModels
from ros_tracking.tracker import InitializeTracker, ReconfigureTracker


def InitializeTracker(tracker):

    # Declare parameters
    tracker.declare_parameter('tracker.object_classes', rclpy.Parameter.Type.STRING_ARRAY)
    tracker.declare_parameter('tracker.frame_id', rclpy.Parameter.Type.STRING)
    tracker.declare_parameter('tracker.asgn_thresh', rclpy.Parameter.Type.DOUBLE)
    tracker.declare_parameter('tracker.del_thresh_list', rclpy.Parameter.Type.DOUBLE_ARRAY)
    tracker.declare_parameter('tracker.pub_thresh_list', rclpy.Parameter.Type.DOUBLE_ARRAY)
    tracker.declare_parameter('tracker.publishers.names', rclpy.Parameter.Type.STRING_ARRAY)
    tracker.declare_parameter('tracker.n_age_max_list', rclpy.Parameter.Type.INTEGER_ARRAY)
    tracker.declare_parameter('tracker.n_birth_min_list', rclpy.Parameter.Type.INTEGER_ARRAY)
    tracker.declare_parameter('tracker.trk_mgmt_method', rclpy.Parameter.Type.STRING)
    tracker.declare_parameter('object_properties.object_classes', rclpy.Parameter.Type.STRING_ARRAY)

    # Read parameters and assign to tracker object
    tracker.object_classes = tracker.get_parameter('tracker.object_classes').get_parameter_value().string_array_value
    tracker.frame_id = tracker.get_parameter('tracker.frame_id').get_parameter_value().string_value
    tracker.asgn_thresh = tracker.get_parameter('tracker.asgn_thresh').get_parameter_value().double_value
    tracker.del_thresh_list = tracker.get_parameter('tracker.del_thresh_list').get_parameter_value().double_array_value
    tracker.pub_thresh_list = tracker.get_parameter('tracker.pub_thresh_list').get_parameter_value().double_array_value
    tracker.n_age_max_list = tracker.get_parameter('tracker.n_age_max_list').get_parameter_value().integer_array_value
    tracker.n_birth_min_list= tracker.get_parameter('tracker.n_birth_min_list').get_parameter_value().integer_array_value
    tracker.trk_mgmt_method = tracker.get_parameter('tracker.trk_mgmt_method').get_parameter_value().string_value
    tracker.obj_prop_classes = tracker.get_parameter('object_properties.object_classes').get_parameter_value().string_array_value

    tracker.class_idx_map = dict()
    for idx, class_name in enumerate(tracker.object_classes):
        tracker.class_idx_map[class_name] = idx

    tracker.object_properties = dict()
    for obj_name in tracker.obj_prop_classes:
        tracker.declare_parameter('object_properties.' + obj_name + '.model_type', rclpy.Parameter.Type.STRING)
        tracker.declare_parameter('object_properties.' + obj_name + '.proc_var', rclpy.Parameter.Type.DOUBLE_ARRAY)
        tracker.declare_parameter('object_properties.' + obj_name + '.init_vel_cov', rclpy.Parameter.Type.DOUBLE_ARRAY)

        tracker.object_properties[obj_name] = {'model_type': tracker.get_parameter('object_properties.' + obj_name + '.model_type').get_parameter_value().string_value, 
                                               'proc_var': tracker.get_parameter('object_properties.' + obj_name + '.proc_var').get_parameter_value().double_array_value,
                                               'init_vel_cov': tracker.get_parameter('object_properties.' + obj_name + '.init_vel_cov').get_parameter_value().double_array_value}

    # Create publishers
    tracker.pub_names = tracker.get_parameter('tracker.publishers.names').get_parameter_value().string_array_value
    tracker.pubs = dict()
    for pub in tracker.pub_names:
        pub_dict = dict()
        tracker.declare_parameter('tracker.publishers.' + pub + '.pub_topic', rclpy.Parameter.Type.STRING)
        tracker.declare_parameter('tracker.publishers.' + pub + '.msg_type', rclpy.Parameter.Type.STRING)
        tracker.declare_parameter('tracker.publishers.' + pub + '.routine', rclpy.Parameter.Type.STRING)
        pub_dict['topic'] = tracker.get_parameter('tracker.publishers.' + pub + '.pub_topic').get_parameter_value().string_value
        pub_dict['msg_type'] = tracker.get_parameter('tracker.publishers.' + pub + '.msg_type').get_parameter_value().string_value
        pub_dict['routine'] = tracker.get_parameter('tracker.publishers.' + pub + '.routine').get_parameter_value().string_value
    
        tracker.pubs[pub] = (pub_dict)
        exec('tracker.%s  = tracker.create_publisher(%s,\'%s\',10)' % (pub, pub_dict['msg_type'], pub_dict['topic']))

def ReconfigureTracker(tracker):

    # Read parameters and assign to tracker object
    tracker.object_classes = tracker.get_parameter('tracker.object_classes').get_parameter_value().string_array_value
    tracker.frame_id = tracker.get_parameter('tracker.frame_id').get_parameter_value().string_value
    tracker.asgn_thresh = tracker.get_parameter('tracker.asgn_thresh').get_parameter_value().double_value
    tracker.del_thresh_list = tracker.get_parameter('tracker.del_thresh_list').get_parameter_value().double_array_value
    tracker.pub_thresh_list = tracker.get_parameter('tracker.pub_thresh_list').get_parameter_value().double_array_value
    tracker.n_age_max_list = tracker.get_parameter('tracker.n_age_max_list').get_parameter_value().integer_array_value
    tracker.n_birth_min_list= tracker.get_parameter('tracker.n_birth_min_list').get_parameter_value().integer_array_value
    tracker.trk_mgmt_method = tracker.get_parameter('tracker.trk_mgmt_method').get_parameter_value().string_value
    tracker.obj_prop_classes = tracker.get_parameter('object_properties.object_classes').get_parameter_value().string_array_value

    tracker.class_idx_map = dict()
    for idx, class_name in enumerate(tracker.object_classes):
        tracker.class_idx_map[class_name] = idx

    tracker.object_properties = dict()

    for obj_name in tracker.obj_prop_classes:

        tracker.object_properties[obj_name] = {'model_type': tracker.get_parameter('object_properties.' + obj_name + '.model_type').get_parameter_value().string_value, 
                                               'proc_var': tracker.get_parameter('object_properties.' + obj_name + '.proc_var').get_parameter_value().double_array_value,
                                               'init_vel_cov': tracker.get_parameter('object_properties.' + obj_name + '.init_vel_cov').get_parameter_value().double_array_value}



class Tracker(Node):
    def __init__(self):
        super().__init__('tracker')

        # Configure tracker from .yaml
        InitializeTracker(self)

        # Generate sensor models from .yaml
        CreateSensorModels(self)          

        # Track and detection variables
        self.dets_msg = Detections3D()
        self.trk_id_count = 0
        self.dets = []
        self.trks = []

        # Assignment variables
        self.cost_matrix = np.empty(0)
        self.det_asgn_idx = [] 
        self.trk_asgn_idx = []

        # Create publisher objects and empty messages
        self.trks_msg = Tracks3D()
        self.scene_msg = SceneUpdate()

        # Declare services
        self.reset_srv = self.create_service(Empty, 'reset_tracker', self.reset_tracker)
        self.reconfigure_srv = self.create_service(Empty, 'reconfigure_tracker', self.reconfigure_tracker)

    def reset_tracker(self, req, resp):

        self.get_logger().info("Resetting tracker")

        # Clear track, detection, and assignment variables
        self.dets = []
        self.trks = []

        self.cost_matrix = np.empty(0)
        self.det_asgn_idx = [] 
        self.trk_asgn_idx = []       

        return resp

    def reconfigure_tracker(self, req, resp):

        self.get_logger().info("Reconfiguring tracker")

        # Configure tracker from .yaml
        ReconfigureTracker(self)

        # Generate sensor models from .yaml
        ReconfigureSensorModels(self)  

        return resp

    def propagate_tracks(self):
        for trk in self.trks:
            trk.propagate(self.dets_msg.header.stamp)

    def update_tracks(self, det_params):
        for det_idx, trk_idx in zip(self.det_asgn_idx, self.trk_asgn_idx):
            self.trks[trk_idx].update(self.dets[det_idx], det_params)

    def det_callback(self, det_array_msg, det_params):
        self.dets_msg = det_array_msg
        metadata = self.dets_msg.detections[0].metadata
        self.dets = []
       
        # POPULATE detections list from detections message
        self.get_logger().info("DETECT: received %i detections" % (len(self.dets_msg.detections)))
        for det in self.dets_msg.detections:
            self.dets.append(GraphDet(self.dets_msg,det))

        # PROPAGATE existing tracks
        self.propagate_tracks()

        # ASSIGN detections to tracks
        ComputeAssignment(self, det_params['p_class_label'], det_params)
        self.get_logger().info("ASSIGN: cost matrix has shape %lix%li \n" % (self.cost_matrix.shape[0],self.cost_matrix.shape[1]))
        self.get_logger().info("ASSIGN: det assignment vector has length %li \n" % (len(self.det_asgn_idx)))
        self.get_logger().info("ASSIGN: trk assignment vector has length %li \n" % (len(self.trk_asgn_idx)))

        # UPDATE tracks with assigned detections
        self.update_tracks(det_params)

        # UPDATE unmatched tracks (missed detections)
        for i, trk in enumerate(self.trks):
            if i not in self.trk_asgn_idx: # If track is unmatched, handle it as a missed detection

                p_missed = det_params['p_missed_det'][trk.class_dist.argmax()]

                trk.track_conf = gtsam.DiscreteDistribution([gtsam.symbol('e',trk.trk_id),2],[(1-p_missed)*trk.track_conf(0),p_missed*trk.track_conf(1)])

                trk.metadata = metadata
                trk.n_missed += 1
                trk.n_matched = 0

        DeleteTracks(self)

        # CREATE tracks from unmatched detections, as appropriate
        CreateTracks(self, det_params)

        # OUTPUT tracker results
        self.get_logger().info("PUBLISH: have %i tracks, %i detections \n" % (len(self.trks), len(self.dets)))
        for pub_name in self.pubs:
            exec('%s(self,\'%s\')' % (self.pubs[pub_name]['routine'],pub_name))