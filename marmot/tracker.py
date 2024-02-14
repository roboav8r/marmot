#!/usr/bin/env python3

import gtsam
import numpy as np

import rclpy
from rclpy.node import Node

from foxglove_msgs.msg import SceneUpdate, SceneEntity
from tracking_msgs.msg import Tracks3D, Track3D, Detections3D, Detection3D
from std_srvs.srv import Empty

from marmot.management import create_tracks, delete_tracks
from marmot.datatypes import Detection
from marmot.assignment import compute_assignment
from marmot.output import publish_tracks, publish_scene

class TBDTracker(Node):
    def __init__(self):

        super().__init__('tbd_tracker_node')
        self.get_logger().info("Creating tracking-by-detection tracker node")

        # Configure tracker params from .yaml
        self.declare_parameter('tracker.frame_id', rclpy.Parameter.Type.STRING)
        self.frame_id = self.get_parameter('tracker.frame_id').get_parameter_value().string_value
        self.declare_parameter('tracker.mismatch_penalty', rclpy.Parameter.Type.DOUBLE)
        self.mismatch_penalty = self.get_parameter('tracker.mismatch_penalty').get_parameter_value().double_value    
        self.declare_parameter('tracker.assignment_algo', rclpy.Parameter.Type.STRING)
        self.assignment_algo = self.get_parameter('tracker.assignment_algo').get_parameter_value().string_value
        self.declare_parameter('tracker.yaw_corr', rclpy.Parameter.Type.BOOL)
        self.yaw_corr = self.get_parameter('tracker.yaw_corr').get_parameter_value().bool_value

        # Configure tracker's object properties dictionary from .yaml
        self.declare_parameter('object_properties.object_classes', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_obj_params()
        self.set_obj_properties()

        # Create publishers
        self.declare_parameter('tracker.publishers.names', rclpy.Parameter.Type.STRING_ARRAY)
        self.pub_names = self.get_parameter('tracker.publishers.names').get_parameter_value().string_array_value
        self.pubs = dict()
        for pub in self.pub_names:
            pub_dict = dict()
            self.declare_parameter('tracker.publishers.' + pub + '.pub_topic', rclpy.Parameter.Type.STRING)
            self.declare_parameter('tracker.publishers.' + pub + '.msg_type', rclpy.Parameter.Type.STRING)
            self.declare_parameter('tracker.publishers.' + pub + '.function', rclpy.Parameter.Type.STRING)
            self.declare_parameter('tracker.publishers.' + pub + '.queue_size', rclpy.Parameter.Type.INTEGER)
            pub_dict['topic'] = self.get_parameter('tracker.publishers.' + pub + '.pub_topic').get_parameter_value().string_value
            pub_dict['msg_type'] = self.get_parameter('tracker.publishers.' + pub + '.msg_type').get_parameter_value().string_value
            pub_dict['function'] = self.get_parameter('tracker.publishers.' + pub + '.function').get_parameter_value().string_value
            pub_dict['queue_size'] = self.get_parameter('tracker.publishers.' + pub + '.queue_size').get_parameter_value().integer_value
        
            self.pubs[pub] = (pub_dict)
            exec('self.%s  = self.create_publisher(%s,\'%s\',%s)' % (pub, pub_dict['msg_type'], pub_dict['topic'], pub_dict['queue_size']))

        # Generate detector models from .yaml
        self.declare_parameter('detectors.detector_names', rclpy.Parameter.Type.STRING_ARRAY)
        detector_names = self.get_parameter('detectors.detector_names').get_parameter_value().string_array_value
        self.detectors = dict()
        self.subs = []

        for detector_idx, detector in enumerate(detector_names):

            # Declare parameters for detector
            self.declare_parameter('detectors.' + detector + '.topic', rclpy.Parameter.Type.STRING)
            self.declare_parameter('detectors.' + detector + '.msg_type', rclpy.Parameter.Type.STRING)
            self.declare_parameter('detectors.' + detector + '.detector_type',rclpy.Parameter.Type.STRING)
            self.declare_parameter('detectors.' + detector + '.detection_classes', rclpy.Parameter.Type.STRING_ARRAY)

            # Form parameter dictionary for detector
            detector_params = dict()
            detector_params['topic'] = self.get_parameter('detectors.' + detector + '.topic').get_parameter_value().string_value
            detector_params['msg_type'] = self.get_parameter('detectors.' + detector + '.msg_type').get_parameter_value().string_value
            detector_params['detector_type'] = self.get_parameter('detectors.' + detector + '.detector_type').get_parameter_value().string_value
            detector_params['detection_classes'] = self.get_parameter('detectors.' + detector + '.detection_classes').get_parameter_value().string_array_value

            detector_params['detection_params'] = dict()
            for det_cls in detector_params['detection_classes']:
                detector_params['detection_params'][det_cls] = dict()

                self.declare_parameter('detectors.' + detector + '.detection_properties.' + det_cls + '.pos_obs_var',rclpy.Parameter.Type.DOUBLE_ARRAY)
                self.declare_parameter('detectors.' + detector + '.detection_properties.' + det_cls + '.yaw_obs_var',rclpy.Parameter.Type.DOUBLE_ARRAY)
                self.declare_parameter('detectors.' + detector + '.detection_properties.' + det_cls + '.size_obs_var',rclpy.Parameter.Type.DOUBLE_ARRAY)
                self.declare_parameter('detectors.' + detector + '.detection_properties.' + det_cls + '.ignore',rclpy.Parameter.Type.BOOL)
                self.declare_parameter('detectors.' + detector + '.detection_properties.' + det_cls + '.object_class',rclpy.Parameter.Type.STRING)
                
                detector_params['detection_params'][det_cls]['pos_obs_var'] = self.get_parameter('detectors.' + detector + '.detection_properties.' + det_cls + '.pos_obs_var').get_parameter_value().double_array_value
                detector_params['detection_params'][det_cls]['yaw_obs_var'] = self.get_parameter('detectors.' + detector + '.detection_properties.' + det_cls + '.yaw_obs_var').get_parameter_value().double_array_value
                detector_params['detection_params'][det_cls]['size_obs_var'] = self.get_parameter('detectors.' + detector + '.detection_properties.' + det_cls + '.size_obs_var').get_parameter_value().double_array_value
                detector_params['detection_params'][det_cls]['ignore'] = self.get_parameter('detectors.' + detector + '.detection_properties.' + det_cls + '.ignore').get_parameter_value().bool_value
                detector_params['detection_params'][det_cls]['obj_class'] = self.get_parameter('detectors.' + detector + '.detection_properties.' + det_cls + '.object_class').get_parameter_value().string_value
                

            self.detectors[detector] = detector_params # Add to tracker's detectors dictionary

            # Create ROS2 subscription for this detector
            self.subs.append(self.create_subscription(eval(detector_params['msg_type']),
                    detector_params['topic'],
                    lambda msg: self.det_callback(msg, detector), 
                    1
            ))

        # Initialize tracker Track and Detection set variables
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

    def declare_obj_params(self):

        self.obj_classes = self.get_parameter('object_properties.object_classes').get_parameter_value().string_array_value

        for obj_name in self.obj_classes:

            self.declare_parameter('object_properties.' + obj_name + '.model_type', rclpy.Parameter.Type.STRING)
            self.declare_parameter('object_properties.' + obj_name + '.length', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter('object_properties.' + obj_name + '.width', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter('object_properties.' + obj_name + '.height', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter('object_properties.' + obj_name + '.create_method', rclpy.Parameter.Type.STRING)
            self.declare_parameter('object_properties.' + obj_name + '.delete_method', rclpy.Parameter.Type.STRING)
            # TODO add multiple cases based on create/delete method
            self.declare_parameter('object_properties.' + obj_name + '.n_create_min', rclpy.Parameter.Type.INTEGER)
            self.declare_parameter('object_properties.' + obj_name + '.n_delete_max', rclpy.Parameter.Type.INTEGER)
            self.declare_parameter('object_properties.' + obj_name + '.sim_metric', rclpy.Parameter.Type.STRING)
            self.declare_parameter('object_properties.' + obj_name + '.match_thresh', rclpy.Parameter.Type.DOUBLE)
            model_type = self.get_parameter('object_properties.' + obj_name + '.model_type').get_parameter_value().string_value

            # Add model-specific parameters
            if model_type in ['cp']:
                self.declare_parameter('object_properties.' + obj_name + '.yaw_proc_var', rclpy.Parameter.Type.DOUBLE)
                self.declare_parameter('object_properties.' + obj_name + '.size_proc_var', rclpy.Parameter.Type.DOUBLE_ARRAY)
                self.declare_parameter('object_properties.' + obj_name + '.pos_proc_var', rclpy.Parameter.Type.DOUBLE_ARRAY)
            elif model_type in ['cvcy', 'cvcy_obj']:
                self.declare_parameter('object_properties.' + obj_name + '.yaw_proc_var', rclpy.Parameter.Type.DOUBLE)
                self.declare_parameter('object_properties.' + obj_name + '.size_proc_var', rclpy.Parameter.Type.DOUBLE_ARRAY)
                self.declare_parameter('object_properties.' + obj_name + '.vel_proc_var', rclpy.Parameter.Type.DOUBLE_ARRAY)
            else:
                raise TypeError('No process model for type: %s' % model_type)

    def set_obj_properties(self):

        self.obj_classes = self.get_parameter('object_properties.object_classes').get_parameter_value().string_array_value

        self.obj_props = dict()
        
        for obj_name in self.obj_classes:

            temp_dict = dict()
            temp_dict['model_type'] = self.get_parameter('object_properties.' + obj_name + '.model_type').get_parameter_value().string_value
            temp_dict['length'] = self.get_parameter('object_properties.' + obj_name + '.length').get_parameter_value().double_value
            temp_dict['width'] = self.get_parameter('object_properties.' + obj_name + '.width').get_parameter_value().double_value
            temp_dict['height'] = self.get_parameter('object_properties.' + obj_name + '.height').get_parameter_value().double_value
            temp_dict['create_method'] = self.get_parameter('object_properties.' + obj_name + '.create_method').get_parameter_value().string_value
            temp_dict['delete_method'] = self.get_parameter('object_properties.' + obj_name + '.delete_method').get_parameter_value().string_value
            # TODO add multiple cases based on create/delete method
            temp_dict['n_create_min'] = self.get_parameter('object_properties.' + obj_name + '.n_create_min').get_parameter_value().integer_value
            temp_dict['n_delete_max'] = self.get_parameter('object_properties.' + obj_name + '.n_delete_max').get_parameter_value().integer_value
            temp_dict['sim_metric'] = self.get_parameter('object_properties.' + obj_name + '.sim_metric').get_parameter_value().string_value
            temp_dict['match_thresh'] = self.get_parameter('object_properties.' + obj_name + '.match_thresh').get_parameter_value().double_value

            # Add model-specific parameters
            if temp_dict['model_type'] in ['cp']:
                temp_dict['yaw_proc_var'] = self.get_parameter('object_properties.' + obj_name + '.yaw_proc_var').get_parameter_value().double_value
                temp_dict['size_proc_var'] = self.get_parameter('object_properties.' + obj_name + '.size_proc_var').get_parameter_value().double_array_value
                temp_dict['pos_proc_var'] = self.get_parameter('object_properties.' + obj_name + '.pos_proc_var').get_parameter_value().double_array_value
            elif temp_dict['model_type'] in ['cvcy', 'cvcy_obj']:
                temp_dict['yaw_proc_var'] = self.get_parameter('object_properties.' + obj_name + '.yaw_proc_var').get_parameter_value().double_value
                temp_dict['size_proc_var'] = self.get_parameter('object_properties.' + obj_name + '.size_proc_var').get_parameter_value().double_array_value
                temp_dict['vel_proc_var'] = self.get_parameter('object_properties.' + obj_name + '.vel_proc_var').get_parameter_value().double_array_value
            else:
                raise TypeError('No process model for type: %s' % temp_dict['model_type'])

            self.obj_props[obj_name] = temp_dict

    def reset_tracker(self, _, resp):

        self.get_logger().info("Resetting tracker")

        # Clear track, detection, and assignment variables
        self.dets = []
        self.trks = []

        self.cost_matrix = np.empty(0)
        self.det_asgn_idx = [] 
        self.trk_asgn_idx = []       

        return resp

    def reconfigure_tracker(self, _, resp):

        self.get_logger().info("Reconfiguring tracker")

        # Configure tracker from .yaml
        self.obj_classes = self.get_parameter('object_properties.object_classes').get_parameter_value().string_array_value
        self.frame_id = self.get_parameter('tracker.frame_id').get_parameter_value().string_value

        # Reconfigure objects
        self.set_obj_properties()

        # Generate detector models from .yaml
        # TODO 

        return resp

    # def propagate_tracks(self):
    #     for trk in self.trks:
    #         trk.propagate(self.dets_msg.header.stamp)

    def update_tracks(self):
        for det_idx, trk_idx in zip(self.det_asgn_idx, self.trk_asgn_idx):
            self.trks[trk_idx].update(self.dets[det_idx],self)

    def det_callback(self, det_array_msg, detector_name):
       
        # POPULATE detections list from detections message
        self.dets_msg = det_array_msg
        self.get_logger().info("DETECT: received %i detections from %s detector" % (len(self.dets_msg.detections), detector_name))
        self.dets = []
        for det in self.dets_msg.detections:
            self.dets.append(Detection(self, self.dets_msg, det, detector_name))

        # # PROPAGATE existing tracks
        # self.propagate_tracks()

        # ASSIGN detections to tracks
        compute_assignment(self)
        self.get_logger().info("ASSIGN: cost matrix has shape %lix%li \n" % (self.cost_matrix.shape[0],self.cost_matrix.shape[1]))
        self.get_logger().info("ASSIGN: det assignment vector has length %li \n" % (len(self.det_asgn_idx)))
        self.get_logger().info("ASSIGN: trk assignment vector has length %li \n" % (len(self.trk_asgn_idx)))

        # UPDATE tracks with assigned detections
        self.update_tracks(det_pa)

        # UPDATE unmatched tracks (missed detections)
        for i, trk in enumerate(self.trks):
            if i not in self.trk_asgn_idx: # If track is unmatched, handle it as a missed detection                
                trk.metadata = det_array_msg.metadata
                trk.n_cons_misses += 1
                trk.n_cons_matches = 0

        # Manage unmatched tracks and detections
        delete_tracks(self)
        create_tracks(self, detector_name)

        # OUTPUT tracker results
        self.get_logger().info("PUBLISH: have %i tracks, %i detections \n" % (len(self.trks), len(self.dets)))
        for pub_name in self.pub_names:
            exec('%s(self,\'%s\')' % (self.pubs[pub_name]['function'],pub_name))