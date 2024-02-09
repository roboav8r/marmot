#!/usr/bin/env python3

import numpy as np
import gtsam

from rclpy.time import Time

class Detection():
    def __init__(self, trkr, dets_msg, det_msg, det_name):
        # Admin
        self.timestamp = Time.from_msg(dets_msg.header.stamp)
        self.det_name = det_name

        # Semantic Properties
        self.metadata = det_msg.metadata
        self.det_class_str = det_msg.class_string
        self.obj_class_str = trkr.detectors[det_name]['detection_params'][self.det_class_str]['obj_class']
        self.class_conf = det_msg.class_confidence
       
        # Spatial properties
        self.pose = det_msg.pose
        self.pos = np.array([[det_msg.pose.position.x], [det_msg.pose.position.y], [det_msg.pose.position.z]])
        self.yaw = np.array([[np.arctan2(2*det_msg.pose.orientation.w*det_msg.pose.orientation.z, 1-2*det_msg.pose.orientation.z**2)]])

        # If bbox info available, use it. Otherwise use default values from yaml
        if trkr.detectors[det_name]['detector_type'] in ['pos_bbox_3d']:
            self.size = np.array([[det_msg.bbox.size.x], [det_msg.bbox.size.y], [det_msg.bbox.size.z]])
        else:           
            self.size = np.array([[trkr.obj_props[self.obj_class_str]['length']], 
                                  [trkr.obj_props[self.obj_class_str]['width']], 
                                  [trkr.obj_props[self.obj_class_str]['height']]])

class Track():
    def __init__(self, trkr, det):

        # Admin
        self.trk_id = trkr.trk_id_count
        self.n_cons_matches = 1
        self.n_cons_misses = 0
        self.metadata = det.metadata

        # Semantic
        self.det_class_str = det.det_class_str # TODO - remove this if it isn't needed later on
        self.obj_class_str = det.obj_class_str
        self.class_conf = det.class_conf

        # Spatial
        self.pose = det.pose
        self.pos = det.pos
        self.yaw = det.yaw
        self.size = det.size

        # Initialize state depending on process model type
        if trkr.obj_props[self.obj_class_str]['model_type'] in ['cp']:

            # Kalman filter & state
            self.kf = gtsam.KalmanFilter(7) # pos_x, pos_y, pos_z, yaw, length, width, height
            self.cov = np.diag(np.concatenate((trkr.detectors[det.det_name]['detection_params'][self.det_class_str]['pos_obs_var'], 
                                               trkr.detectors[det.det_name]['detection_params'][self.det_class_str]['yaw_obs_var'], 
                                               trkr.detectors[det.det_name]['detection_params'][self.det_class_str]['size_obs_var'])))**2 
            self.spatial_state = self.kf.init(np.vstack((self.pos, self.yaw, self.size)), self.cov)

        elif trkr.obj_props[self.obj_class_str]['model_type'] in ['cvcy','cvcy_obj']:

            # Kalman filter & state
            self.kf = gtsam.KalmanFilter(10) # pos_x, pos_y, pos_z, yaw, length, width, height, vel_x, vel_y, vel_z
            self.cov = np.diag(np.concatenate((trkr.detectors[det.det_name]['detection_params'][self.det_class_str]['pos_obs_var'], 
                                               trkr.detectors[det.det_name]['detection_params'][self.det_class_str]['yaw_obs_var'], 
                                               trkr.detectors[det.det_name]['detection_params'][self.det_class_str]['size_obs_var'],
                                               trkr.obj_props[self.obj_class_str]['vel_proc_var'])))**2
            self.spatial_state = self.kf.init(np.vstack((self.pos, self.yaw, self.size, np.array([[0], [0], [0]]))), self.cov)
        else:
            raise TypeError('No process model for type: %s' % trkr.obj_props[self.obj_class_str]['model_type'])